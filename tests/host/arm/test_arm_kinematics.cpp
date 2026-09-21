// ② 運動学の検証。
//
// 期待値は §2 の式から手で立てる。実装に計算させた値を期待値にすると、
// 実装が幾何と食い違っていても往復するだけで通ってしまう。
//
//   d = √(L1² + L2² + 2·L1·L2·cos e)      L1 = L2 のとき 2·L1·|cos(e/2)|
//   det J = L1·L2·sin e
//
// **リンクしているのは kinematics.cpp 1 本だけ。** 層が単独でテストできることの実演で、
// pico-sdk のスタブも他の層も要らない。

#include "test_util.hpp"

#include "arm/kinematics.hpp"

namespace {

using arm::DEG;
using arm::JointAngles;
using arm::JointVels;
using arm::Point;
using arm::Velocity;
namespace kin = arm::kin;

constexpr kin::Config cfg_default{};  // L=600, Positive, 10..170 deg

uint64_t code(kin::Status s) { return static_cast<uint64_t>(s); }

// 手計算側の d。実装を通さない。
float hand_reach(float l1, float l2, float e) {
    return sqrtf(l1 * l1 + l2 * l2 + 2.0f * l1 * l2 * cosf(e));
}

}  // namespace

int main() {
    test::section("1. 順運動学 → 逆運動学の往復 (誤差 < 0.01 mm)");
    {
        // 到達可能な円環の中を格子で舐める。
        float max_err = 0.0f;
        int   checked = 0;
        for (int ix = -12; ix <= 12; ++ix) {
            for (int iy = -12; iy <= 12; ++iy) {
                const Point p{static_cast<float>(ix) * 90.0f, static_cast<float>(iy) * 90.0f};
                JointAngles  q{};
                if (kin::inverse(cfg_default, p, q) != kin::Status::Ok) continue;
                const Point  back = kin::forward(cfg_default, q);
                const float  err  = arm::dist(p, back);
                if (err > max_err) max_err = err;
                ++checked;
            }
        }
        test::check(checked > 300, "十分な点数を検査した");
        test::checkNearF(max_err, 0.0f, 0.01f, "往復誤差が 0.01 mm 未満");
    }

    test::section("2. 到達可能領域の境界 (内 104 mm / 外 1195 mm)");
    {
        // reach_* が手計算と一致すること。**|e| が小さいほど遠い。**
        test::checkNearF(kin::reach_outer(cfg_default), hand_reach(600.0f, 600.0f, 10.0f * DEG),
                         0.01f, "外側限界 = elbow_min のときの d");
        test::checkNearF(kin::reach_inner(cfg_default), hand_reach(600.0f, 600.0f, 170.0f * DEG),
                         0.01f, "内側限界 = elbow_max のときの d");
        test::checkNearF(kin::reach_outer(cfg_default), 1195.43f, 0.05f, "外側は約 1195 mm");
        test::checkNearF(kin::reach_inner(cfg_default), 104.59f, 0.05f, "内側は約 104 mm");

        const float ro = kin::reach_outer(cfg_default);
        const float ri = kin::reach_inner(cfg_default);
        test::checkEq(code(kin::check_reachable(cfg_default, Point{ro - 1.0f, 0.0f})),
                      code(kin::Status::Ok), "外側限界のすぐ内は届く");
        test::checkEq(code(kin::check_reachable(cfg_default, Point{ro + 1.0f, 0.0f})),
                      code(kin::Status::TooExtended), "外側限界の外は伸びきりすぎ");
        test::checkEq(code(kin::check_reachable(cfg_default, Point{ri + 1.0f, 0.0f})),
                      code(kin::Status::Ok), "内側限界のすぐ外は届く");
        test::checkEq(code(kin::check_reachable(cfg_default, Point{ri - 1.0f, 0.0f})),
                      code(kin::Status::TooFolded), "内側限界の内は折り畳みすぎ");
        test::checkEq(code(kin::check_reachable(cfg_default, Point{0.0f, 0.0f})),
                      code(kin::Status::TooFolded), "原点は折り畳み側 (ゼロ除算を踏まない)");
        test::checkEq(code(kin::check_reachable(cfg_default, Point{5000.0f, 0.0f})),
                      code(kin::Status::TooExtended), "遠すぎる点は伸びきり側");
    }

    test::section("3 / 4. 肘角の限界で返る Status");
    {
        // FK で肘角を作り、その手先点を IK に戻す。名前と幾何の対応をここで固定する。
        struct Case {
            float       elbow_deg;
            kin::Status want;
            const char* what;
        };
        const Case cases[] = {
            {5.0f, kin::Status::TooExtended, "|e| = 5 deg は伸びきりすぎ (d ≈ 1199 mm)"},
            {9.5f, kin::Status::TooExtended, "|e| = 9.5 deg も伸びきりすぎ"},
            {11.0f, kin::Status::Ok, "|e| = 11 deg は可"},
            {90.0f, kin::Status::Ok, "|e| = 90 deg は可"},
            {169.0f, kin::Status::Ok, "|e| = 169 deg は可"},
            {171.0f, kin::Status::TooFolded, "|e| = 171 deg は折り畳みすぎ (d ≈ 100 mm)"},
            {178.0f, kin::Status::TooFolded, "|e| = 178 deg も折り畳みすぎ"},
        };
        for (const Case& c : cases) {
            const float       t1 = 0.4f;  // θ1 は何でもよい
            const JointAngles q{t1, t1 + c.elbow_deg * DEG};
            const Point       p = kin::forward(cfg_default, q);
            test::checkEq(code(kin::check_reachable(cfg_default, p)), code(c.want), c.what);
        }

        // 名前が幾何と合っていること: 伸びきり側は遠い、折り畳み側は近い。
        const JointAngles q_ext{0.0f, 5.0f * DEG};
        const JointAngles q_fold{0.0f, 175.0f * DEG};
        const float d_ext  = arm::hypotf2(kin::forward(cfg_default, q_ext).x,
                                          kin::forward(cfg_default, q_ext).y);
        const float d_fold = arm::hypotf2(kin::forward(cfg_default, q_fold).x,
                                          kin::forward(cfg_default, q_fold).y);
        test::check(d_ext > 1190.0f, "TooExtended になる姿勢は手先が遠い");
        test::check(d_fold < 110.0f, "TooFolded になる姿勢は手先が肩の近く");
    }

    test::section("elbow_min / elbow_max は設定から読まれている");
    {
        kin::Config wide = cfg_default;
        wide.elbow_min   = 3.0f * DEG;
        wide.elbow_max   = 177.0f * DEG;

        const JointAngles q{0.4f, 0.4f + 5.0f * DEG};
        const Point       p = kin::forward(cfg_default, q);
        test::checkEq(code(kin::check_reachable(cfg_default, p)), code(kin::Status::TooExtended),
                      "既定では弾かれる");
        test::checkEq(code(kin::check_reachable(wide, p)), code(kin::Status::Ok),
                      "elbow_min を下げると通る (ハードコードされていない)");

        kin::Config narrow = cfg_default;
        narrow.elbow_min   = 30.0f * DEG;
        const JointAngles q20{0.4f, 0.4f + 20.0f * DEG};
        test::checkEq(code(kin::check_reachable(narrow, kin::forward(cfg_default, q20))),
                      code(kin::Status::TooExtended), "elbow_min を上げると弾かれる");
    }

    test::section("姿勢そのものを検査する (関節ジョグ・校正で使う)");
    {
        // 運動学を通さずに関節角を作る経路には、点ではなく姿勢の検査が要る。
        const float t1 = 0.4f;
        test::checkEq(code(kin::check_joints(cfg_default, JointAngles{t1, t1 + 90.0f * DEG})),
                      code(kin::Status::Ok), "範囲内の姿勢");
        test::checkEq(code(kin::check_joints(cfg_default, JointAngles{t1, t1 + 5.0f * DEG})),
                      code(kin::Status::TooExtended), "伸びきりすぎ");
        test::checkEq(code(kin::check_joints(cfg_default, JointAngles{t1, t1 + 175.0f * DEG})),
                      code(kin::Status::TooFolded), "折り畳みすぎ");

        // **向きが違う姿勢は範囲内でも作れない。** 組み立てで片側に固定されている。
        // ここを見ないと、鏡像の姿勢を「範囲内」として通してしまう。
        test::checkEq(code(kin::check_joints(cfg_default, JointAngles{t1, t1 - 90.0f * DEG})),
                      code(kin::Status::WrongElbowSide), "負側の姿勢は正側の機体では作れない");

        kin::Config neg = cfg_default;
        neg.elbow_side  = kin::ElbowSide::Negative;
        test::checkEq(code(kin::check_joints(neg, JointAngles{t1, t1 - 90.0f * DEG})),
                      code(kin::Status::Ok), "負側の機体なら通る");
        test::checkEq(code(kin::check_joints(neg, JointAngles{t1, t1 + 90.0f * DEG})),
                      code(kin::Status::WrongElbowSide), "正側の姿勢は逆に弾かれる");

        // IK が返した姿勢は必ず check_joints を通る (2 つの判定が食い違わない)
        bool consistent = true;
        for (int i = 0; i < 400; ++i) {
            const float phi = arm::TWO_PI * static_cast<float>(i) / 400.0f;
            const Point p{700.0f * cosf(phi), 700.0f * sinf(phi)};
            JointAngles q{};
            if (kin::inverse(cfg_default, p, q) != kin::Status::Ok) continue;
            if (kin::check_joints(cfg_default, q) != kin::Status::Ok) consistent = false;
        }
        test::check(consistent, "IK の解は必ず姿勢検査を通る");

        const float nan_v = sqrtf(-1.0f);
        test::checkEq(code(kin::check_joints(cfg_default, JointAngles{nan_v, 0.0f})),
                      code(kin::Status::NotFinite), "NaN を弾く");
    }

    test::section("正規化可操作度が det_eps と比べる量");
    {
        const JointAngles q{0.3f, 0.3f + 30.0f * DEG};
        test::checkNearF(kin::normalized_manipulability(cfg_default, q), sinf(30.0f * DEG), 1e-5f,
                         "|sin e| そのもの");
        test::checkNearF(kin::manipulability(cfg_default, q),
                         600.0f * 600.0f * sinf(30.0f * DEG), 1.0f, "生のほうは l1·l2 倍");

        // **リンク長を変えても正規化側は動かない。** これが det_eps と比べられる理由。
        kin::Config big = cfg_default;
        big.l1 = big.l2 = 6000.0f;
        test::checkNearF(kin::normalized_manipulability(big, q),
                         kin::normalized_manipulability(cfg_default, q), 1e-6f,
                         "リンク長 10 倍でも同じ");
        test::check(kin::manipulability(big, q) > 50.0f * kin::manipulability(cfg_default, q),
                    "生のほうは 100 倍になる");
    }

    test::section("5. elbow_side を反転すると目標方向の直線に対する鏡像になる");
    {
        kin::Config neg = cfg_default;
        neg.elbow_side  = kin::ElbowSide::Negative;

        const Point p{500.0f, 300.0f};
        JointAngles qp{}, qn{};
        test::checkEq(code(kin::inverse(cfg_default, p, qp)), code(kin::Status::Ok), "正側で解ける");
        test::checkEq(code(kin::inverse(neg, p, qn)), code(kin::Status::Ok), "負側で解ける");

        const float phi = atan2f(p.y, p.x);
        test::checkNearF(arm::normalize_angle(qn.theta1 - (2.0f * phi - qp.theta1)), 0.0f, 1e-5f,
                         "θ1' = 2φ − θ1");
        test::checkNearF(arm::normalize_angle(qn.theta2 - (2.0f * phi - qp.theta2)), 0.0f, 1e-5f,
                         "θ2' = 2φ − θ2");
        test::checkNearF(kin::elbow_angle(qp), -kin::elbow_angle(qn), 1e-5f, "肘角の符号が反転する");
        test::check(kin::elbow_angle(qp) > 0.0f, "正側の肘角は正");

        // どちらも同じ点に届いていること
        test::checkNearF(arm::dist(kin::forward(neg, qn), p), 0.0f, 0.01f, "負側も同じ点に届く");
    }

    test::section("6. 伸びきり近傍で det J が小さくなる");
    {
        const JointAngles q_far{0.0f, 90.0f * DEG};
        const JointAngles q_near{0.0f, 10.0f * DEG};
        const JointAngles q_dead{0.0f, 0.0f};

        // det J = l1·l2·sin e。手計算で立てる。
        test::checkNearF(kin::det_jacobian(cfg_default, q_far), 600.0f * 600.0f, 1.0f,
                         "e = 90 deg で det J = l1·l2");
        test::checkNearF(kin::det_jacobian(cfg_default, q_near),
                         600.0f * 600.0f * sinf(10.0f * DEG), 1.0f, "e = 10 deg で l1·l2·sin e");
        test::checkNearF(kin::det_jacobian(cfg_default, q_dead), 0.0f, 1e-2f, "e = 0 は死点");
        test::check(kin::manipulability(cfg_default, q_near) <
                        kin::manipulability(cfg_default, q_far),
                    "伸びきりに近いほど可操作度が下がる");

        // 折り畳み側の死点でも det J は 0 になる。
        // ここは正規化して見る — 生の det は l1·l2 = 360000 のスケールなので、
        // float の π が真の π とずれるぶんだけで 0.03 ほど残る。
        const JointAngles q_fold{0.0f, 180.0f * DEG};
        test::checkNearF(kin::det_jacobian(cfg_default, q_fold) / (600.0f * 600.0f), 0.0f, 1e-6f,
                         "e = 180 deg も死点 (伸びきりだけではない)");

        // ヤコビの成分そのもの
        float J[2][2];
        kin::jacobian(cfg_default, q_far, J);
        test::checkNearF(J[0][0], -600.0f * sinf(0.0f), 1e-3f, "J00 = −l1·sin θ1");
        test::checkNearF(J[1][0], 600.0f * cosf(0.0f), 1e-3f, "J10 = l1·cos θ1");
        test::checkNearF(J[0][1], -600.0f * sinf(90.0f * DEG), 1e-3f, "J01 = −l2·sin θ2");
        test::checkNearF(J[1][1], 600.0f * cosf(90.0f * DEG), 1e-3f, "J11 = l2·cos θ2");
    }

    test::section("7. θ1 が ±π を跨いでも連続に扱える");
    {
        // 半径 800 の円周を 1 周させる。θ1 = φ − psi で psi は半径だけで決まるので、
        // φ をどこから始めても 1 周のあいだに必ず ±π を跨ぐ。
        // 跨ぐ位置を狙い撃ちにしないのは、psi の値に依存したテストにしないため。
        const float r = 800.0f;
        float       prev_cont = 0.0f;
        float       max_step  = 0.0f;
        bool        crossed   = false;
        float       prev_raw  = 0.0f;
        bool        first     = true;

        for (int i = 0; i <= 3600; ++i) {
            const float phi = (0.1f * static_cast<float>(i)) * DEG;
            const Point p{r * cosf(phi), r * sinf(phi)};
            JointAngles q{};
            if (kin::inverse(cfg_default, p, q) != kin::Status::Ok) continue;

            if (first) {
                prev_cont = q.theta1;
                prev_raw  = q.theta1;
                first     = false;
                continue;
            }
            // 正規化されたままだと ±π で 2π 近い跳びが出る
            if (fabsf(q.theta1 - prev_raw) > 5.0f) crossed = true;
            prev_raw = q.theta1;

            const float cont = arm::nearest_angle(prev_cont, q.theta1);
            const float step = fabsf(cont - prev_cont);
            if (step > max_step) max_step = step;
            prev_cont = cont;
        }
        test::check(crossed, "生の θ1 は途中で ±π を跨いでいる");
        test::check(max_step < 0.01f, "nearest_angle を通すと 1 ステップの変化が小さいまま");
    }

    test::section("8. inverse_velocity と forward_velocity の往復");
    {
        const JointAngles q{20.0f * DEG, 110.0f * DEG};  // e = 90 deg、特異点から遠い
        const Velocity    v{123.0f, -45.0f};

        JointVels dq{};
        test::checkEq(code(kin::inverse_velocity(cfg_default, q, v, dq)), code(kin::Status::Ok),
                      "速度 IK が解ける");
        const Velocity back = kin::forward_velocity(cfg_default, q, dq);
        test::checkNearF(back.vx, v.vx, 1e-2f, "vx が戻る");
        test::checkNearF(back.vy, v.vy, 1e-2f, "vy が戻る");

        // 特異点近傍は拒否する
        const JointAngles q_sing{0.0f, 0.0f};
        JointVels         dq_s{};
        test::checkEq(code(kin::inverse_velocity(cfg_default, q_sing, v, dq_s)),
                      code(kin::Status::Singular), "死点では Singular");

        // det_eps は無次元 (|sin e| と比べる) なので、リンク長を変えても閾値が動かない。
        kin::Config big = cfg_default;
        big.l1 = big.l2 = 6000.0f;
        test::checkEq(code(kin::inverse_velocity(big, q_sing, v, dq_s)),
                      code(kin::Status::Singular), "リンク長を 10 倍しても同じ判定");
        JointVels dq_b{};
        test::checkEq(code(kin::inverse_velocity(big, q, v, dq_b)), code(kin::Status::Ok),
                      "良い姿勢ならリンク長に依らず解ける");
    }

    test::section("非対称リンク (l1 != l2) でも往復する");
    {
        kin::Config asym = cfg_default;
        asym.l1          = 500.0f;
        asym.l2          = 350.0f;

        float max_err = 0.0f;
        int   checked = 0;
        for (int ix = -10; ix <= 10; ++ix) {
            for (int iy = -10; iy <= 10; ++iy) {
                const Point p{static_cast<float>(ix) * 70.0f, static_cast<float>(iy) * 70.0f};
                JointAngles q{};
                if (kin::inverse(asym, p, q) != kin::Status::Ok) continue;
                const float err = arm::dist(p, kin::forward(asym, q));
                if (err > max_err) max_err = err;
                ++checked;
            }
        }
        test::check(checked > 100, "非対称でも十分な点数が到達可能");
        test::checkNearF(max_err, 0.0f, 0.01f, "非対称でも往復誤差が 0.01 mm 未満");

        // 非対称では d = 2L|cos(e/2)| は使えない。一般式で確かめる。
        test::checkNearF(kin::reach_outer(asym), hand_reach(500.0f, 350.0f, 10.0f * DEG), 0.01f,
                         "外側限界 (非対称)");
        test::checkNearF(kin::reach_inner(asym), hand_reach(500.0f, 350.0f, 170.0f * DEG), 0.01f,
                         "内側限界 (非対称)");
    }

    test::section("NaN / ∞ を入口で弾く");
    {
        const float nan_v = sqrtf(-1.0f);
        const float inf_v = 1.0f / 0.0f;

        JointAngles q{1.0f, 2.0f};
        test::checkEq(code(kin::inverse(cfg_default, Point{nan_v, 0.0f}, q)),
                      code(kin::Status::NotFinite), "NaN の目標点を拒否する");
        test::checkNearF(q.theta1, 1.0f, 1e-6f, "拒否したときは出力を汚さない");

        test::checkEq(code(kin::check_reachable(cfg_default, Point{0.0f, nan_v})),
                      code(kin::Status::NotFinite), "check_reachable も同じ");

        // ∞ も入口で弾く。到達できないのではなく、呼び出し側が壊れている。
        test::checkEq(code(kin::check_reachable(cfg_default, Point{inf_v, 0.0f})),
                      code(kin::Status::NotFinite), "∞ の目標点も拒否する");

        // **有限だが巨大**な座標は別扱い。d2 が ∞ へ飽和しても、クランプ経路が
        // 正しく伸びきり側へ落とす。ここは NaN と違って比較が成立する。
        test::checkEq(code(kin::check_reachable(cfg_default, Point{1.0e20f, 1.0e20f})),
                      code(kin::Status::TooExtended), "巨大な有限座標は伸びきり側として落ちる");

        JointVels dq{9.0f, 9.0f};
        test::checkEq(code(kin::inverse_velocity(cfg_default, JointAngles{0.0f, 1.5f},
                                                 Velocity{nan_v, 0.0f}, dq)),
                      code(kin::Status::NotFinite), "NaN の速度を拒否する");
        test::checkNearF(dq.dq1, 9.0f, 1e-6f, "拒否したときは出力を汚さない");
    }

    test::section("設定の検査");
    {
        test::checkEq(code(kin::validate(cfg_default)), code(kin::Status::Ok), "既定は妥当");

        kin::Config bad = cfg_default;
        bad.l1          = 0.0f;
        test::checkEq(code(kin::validate(bad)), code(kin::Status::BadConfig), "リンク長 0 を弾く");
        JointAngles q{};
        test::checkEq(code(kin::inverse(bad, Point{100.0f, 0.0f}, q)), code(kin::Status::BadConfig),
                      "IK も NaN を返さず BadConfig で落ちる");

        kin::Config rev = cfg_default;
        rev.elbow_min   = 100.0f * DEG;
        rev.elbow_max   = 20.0f * DEG;
        test::checkEq(code(kin::validate(rev)), code(kin::Status::BadConfig), "min > max を弾く");
    }

    return test::report("arm_kinematics");
}

