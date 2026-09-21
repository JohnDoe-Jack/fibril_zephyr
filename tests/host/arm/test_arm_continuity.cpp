// 関節角の連続化。**罠 6 がここの主題。**
//
//   kin::inverse() は −π〜+π を返す → そのまま tx::to_motor() へ渡すと
//   θ1 が ±π を跨いだ瞬間に肩のモータ角が 2π×3 = 6π 飛ぶ
//
// 「連続化を挟まないと本当に壊れる」ことを先に示してから、挟せば直ることを示す。
// 直った側だけを検査すると、その工程が要るのかどうかが分からなくなる。
//
// **制御ループそのものの検証は test_arm_controller.cpp。** 以前ここにあった
// 「制御ループ 1 周期の結線」は、出荷されるコードではなくその写しを検証していた。

#include "test_util.hpp"

#include "arm/continuity.hpp"
#include "arm/kinematics.hpp"
#include "arm/motion.hpp"
#include "arm/transmission.hpp"
#include "arm/workspace.hpp"

namespace {

using arm::JointAngles;
using arm::JointVels;
using arm::Point;
using arm::Velocity;
namespace kin = arm::kin;
namespace mo  = arm::motion;
namespace tx  = arm::tx;
namespace ws  = arm::ws;

constexpr float kDt = 0.001f;

constexpr kin::Config kKin{600.0f,          600.0f, kin::ElbowSide::Positive,
                           10.0f * arm::DEG, 170.0f * arm::DEG, 1.0e-3f};
constexpr tx::Config kTx{};

uint64_t kcode(kin::Status s) { return static_cast<uint64_t>(s); }
uint64_t tcode(tx::Status s) { return static_cast<uint64_t>(s); }

}  // namespace

int main() {
    test::section("設定が起動時の検査を通る");
    {
        test::checkEq(kcode(kin::validate(kKin)), kcode(kin::Status::Ok), "運動学の設定");
        test::checkEq(tcode(tx::validate(kTx)), tcode(tx::Status::Ok), "伝達比の設定");
    }

    test::section("罠 6: 連続化を挟まないとモータ角が 6π 飛ぶ");
    {
        // 半径 800 の円周を反時計回りに回すと θ1 が ±π を跨ぐ。
        // 肩の減速比が 3 なので、跨いだ瞬間のモータ角の跳びは 2π×3 = 6π になる。
        mo::ArcMove arc;
        test::check(arc.start(Point{0.0f, 0.0f}, 800.0f, 190.0f * arm::DEG, 270.0f * arm::DEG,
                              mo::TrapezoidProfile{400.0f, 1500.0f, 0.0f}),
                    "円弧を開始できる");

        arm::JointContinuity cont;
        float naive_max_jump = 0.0f;
        float safe_max_jump  = 0.0f;
        float prev_naive_m1  = 0.0f;
        float prev_safe_m1   = 0.0f;
        bool  first          = true;
        int   steps          = 0;

        while (!arc.is_done() && steps < 100000) {
            const Point p = arc.update(kDt);

            JointAngles q{};
            if (kin::inverse(kKin, p, q) != kin::Status::Ok) break;

            // (a) そのまま渡す = 元の統合例。**これはバグ。**
            tx::MotorAngles naive{};
            if (tx::to_motor(kTx, q, naive) != tx::Status::Ok) break;

            // (b) 連続化してから渡す
            tx::MotorAngles safe{};
            if (tx::to_motor(kTx, cont.track(q), safe) != tx::Status::Ok) break;

            if (!first) {
                const float jn = fabsf(naive.m1 - prev_naive_m1);
                const float js = fabsf(safe.m1 - prev_safe_m1);
                if (jn > naive_max_jump) naive_max_jump = jn;
                if (js > safe_max_jump) safe_max_jump = js;
            }
            prev_naive_m1 = naive.m1;
            prev_safe_m1  = safe.m1;
            first         = false;
            ++steps;
        }

        test::check(steps > 100, "十分な周期数を回した");
        // 跳びは 2π × 減速比 3 = 6π ≈ 18.85 rad
        test::checkNearF(naive_max_jump, 6.0f * arm::PI, 0.1f,
                         "連続化しないと 6π 飛ぶ (罠 6 が実在する)");
        test::check(safe_max_jump < 0.05f, "連続化すれば 1 周期の変化が小さいまま");
    }

    test::section("連続化は肘角の関係を壊さない");
    {
        arm::JointContinuity cont;
        float max_elbow_err = 0.0f;

        for (int i = 0; i <= 2000; ++i) {
            const float phi = arm::TWO_PI * static_cast<float>(i) / 500.0f;  // 4 周させる
            const Point p{800.0f * cosf(phi), 800.0f * sinf(phi)};

            JointAngles q{};
            if (kin::inverse(kKin, p, q) != kin::Status::Ok) continue;

            const JointAngles c = cont.track(q);
            // 連続値から取り直した肘角が、正規化値のそれと一致すること。
            // θ1 と θ2 を独立に連続化すると、ここが 2π ずれても気づけない。
            const float want = kin::elbow_angle(q);
            const float got  = arm::normalize_angle(c.theta2 - c.theta1);
            const float err  = fabsf(arm::normalize_angle(got - want));
            if (err > max_elbow_err) max_elbow_err = err;
        }
        test::checkNearF(max_elbow_err, 0.0f, 1e-5f, "肘角がずれない");

        // 肩が 4 周すれば前腕も 4 周ぶん担がれている
        test::check(fabsf(cont.value().theta1) > 3.0f * arm::TWO_PI, "θ1 が多回転している");
        test::check(fabsf(cont.value().theta2) > 3.0f * arm::TWO_PI, "θ2 も一緒に多回転している");
    }

    test::section("seed で実測値から始められる");
    {
        arm::JointContinuity cont;
        test::check(!cont.has_reference(), "初期状態では基準がない");

        // 電源投入時、肩は 3 周した位置に居るとする
        cont.seed(JointAngles{3.0f * arm::TWO_PI + 0.5f, 3.0f * arm::TWO_PI + 2.0f});
        test::check(cont.has_reference(), "基準ができた");

        const JointAngles c = cont.track(JointAngles{0.5f, 2.0f});
        test::checkNearF(c.theta1, 3.0f * arm::TWO_PI + 0.5f, 1e-3f,
                         "同じ姿勢なら多回転ぶんを保つ");
        test::checkNearF(arm::normalize_angle(c.theta2 - c.theta1), 1.5f, 1e-5f, "肘角は 1.5 rad");
    }

    test::section("到達不能な点は運動学が止める (作業領域とは別の理由)");
    {
        ws::Workspace<4> space;  // 制約なし = 全面許可
        const Point      far{1500.0f, 0.0f};
        test::check(space.contains(far), "作業領域は許している");

        JointAngles q{};
        test::checkEq(kcode(kin::inverse(kKin, far, q)), kcode(kin::Status::TooExtended),
                      "運動学が届かないと言う");

        const Point near_shoulder{50.0f, 0.0f};
        test::check(space.contains(near_shoulder), "作業領域は許している");
        test::checkEq(kcode(kin::inverse(kKin, near_shoulder, q)), kcode(kin::Status::TooFolded),
                      "肩に近すぎる点も運動学が止める");
    }

    return test::report("arm_continuity");
}

