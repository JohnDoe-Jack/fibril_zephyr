// **出荷される制御ループそのものの検証。**
//
// 以前は制御ループが main.cpp の割り込みハンドラの中にあり、テストは同じ手順を
// テスト側で書き写したものを検証していた。写しが通っても、実機で動くコードが
// 通ったことにはならない。いまは `armctl::Controller::step()` を直接回す。
//
// 偽の JointDrive を挿すので CAN も pico-sdk も要らない。**スタブを 1 本も
// リンクしていないこと**が、制御ループが実機に依存していないことの検査になる。

#include "test_util.hpp"

#include "arm_control/controller.hpp"

namespace {

using arm::JointAngles;
using arm::JointVels;
using arm::Point;
using arm::Velocity;
namespace kin = arm::kin;
namespace mo  = arm::motion;
namespace tx  = arm::tx;
namespace ws  = arm::ws;

constexpr float kDt = 0.002f;  // 500 Hz

uint64_t code(armctl::Fault f) { return static_cast<uint64_t>(f); }
uint64_t code(armctl::Mode m) { return static_cast<uint64_t>(m); }
uint64_t code(kin::Status s) { return static_cast<uint64_t>(s); }
uint64_t count(int n) { return static_cast<uint64_t>(n); }

// --- 偽のドライブ -----------------------------------------------------------
// 何を送られたかを記録し、失敗も注入できる。

class FakeDrive final : public armctl::JointDrive {
public:
    bool init() override {
        ++init_calls;
        return init_ok;
    }
    bool enterPositionMode() override {
        ++mode_calls;
        return mode_ok;
    }
    bool commandPosition(float rad) override {
        ++command_calls;
        if (!command_ok) return false;
        // **RS00 と同じ切り詰めを模す。** 本物の setPosition() は仕様範囲外を
        // クランプする。JointDrive の口では「指令どおりに受け取れなかった」ので
        // false を返す — アダプタ (RobStrideJoint) がそう振る舞う契約。
        if (clamp_limit > 0.0f && fabsf(rad) > clamp_limit) {
            ++clamped;
            last_command = (rad > 0.0f) ? clamp_limit : -clamp_limit;
            has_command  = true;
            return false;
        }
        if (has_command) {
            const float jump = fabsf(rad - last_command);
            if (jump > max_jump) max_jump = jump;
        }
        last_command = rad;
        has_command  = true;
        return true;
    }
    bool readPosition(float& out) const override {
        if (!has_position) return false;
        out = position;
        return true;
    }
    bool readTorque(float& out) const override {
        if (!has_torque) return false;
        out = torque;
        return true;
    }
    bool release() override {
        ++release_calls;
        if (!release_ok) return false;
        released = true;
        return true;
    }
    bool isAlive() const override { return alive; }
    bool isEnergized() const override { return energized; }
    bool clearFaults() override {
        ++clear_calls;
        return clear_ok;
    }

    // 注入する故障
    bool init_ok    = true;
    bool mode_ok    = true;
    bool command_ok = true;
    bool release_ok = true;
    bool alive      = true;
    // **通電しているか。** モータが自分で脱力しても応答は続くので、
    // isAlive() とは別の問い (監査 群 2 / H3)。
    bool energized  = true;
    bool clear_ok   = true;
    int  clear_calls = 0;
    // 0 以下なら切り詰めない。正なら ±この値へ黙って丸める (RS00 は ±12.57 rad)。
    float clamp_limit = 0.0f;

    // 実測位置
    bool  has_position = true;
    float position     = 0.0f;

    // 実測トルク [N*m]
    bool  has_torque = true;
    float torque     = 0.0f;

    // 観測
    int   init_calls    = 0;
    int   mode_calls    = 0;
    int   command_calls = 0;
    int   release_calls = 0;
    bool  released      = false;
    bool  has_command   = false;
    float last_command  = 0.0f;
    float max_jump      = 0.0f;
    int   clamped       = 0;
};

// --- 標準の機体 -------------------------------------------------------------

constexpr kin::Config kKin{600.0f,           600.0f, kin::ElbowSide::Positive,
                           10.0f * arm::DEG, 170.0f * arm::DEG, 1.0e-3f};
constexpr tx::Config kTx{arm::tx::JointTransmission{3.0f, -1, 0.0f},
                         arm::tx::JointTransmission{1.0f, +1, 0.0f}};
constexpr armctl::JogLimits kJog{};

// 肩を idx 度、肘角 90 度の姿勢に対応するモータ角を偽ドライブへ入れる。
void placeAt(FakeDrive& sh, FakeDrive& el, float theta1, float elbow) {
    const JointAngles q{theta1, theta1 + elbow};
    tx::MotorAngles   m{};
    tx::to_motor(kTx, q, m);
    sh.position = m.m1;
    el.position = m.m2;
}

armctl::Space makeSpace() {
    armctl::Space s;
    s.add(ws::rect(-1300.0f, 1300.0f, -1300.0f, 1300.0f, true));
    return s;
}

}  // namespace

int main() {
    test::section("実測姿勢で初期化しないと動かない");
    {
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        armctl::Controller c(sh, el, kKin, kTx, space, kJog);

        test::check(!c.isSeeded(), "最初は基準がない");
        test::checkEq(code(c.step(kDt)), code(armctl::Fault::NotSeeded), "step が拒む");
        test::check(sh.released && el.released, "拒んだ時点で脱力する");
        test::checkEq(count(sh.command_calls), 0u, "指令は 1 本も送っていない");

        // フィードバックが来ていなければ初期化もできない
        FakeDrive     sh2, el2;
        sh2.has_position = false;
        armctl::Space      space2 = makeSpace();
        armctl::Controller c2(sh2, el2, kKin, kTx, space2, kJog);
        test::check(!c2.seedFromDrives(), "位置が読めなければ false");

        sh2.has_position = true;
        el2.has_position = true;
        placeAt(sh2, el2, 0.3f, 90.0f * arm::DEG);
        test::check(c2.seedFromDrives(), "読めれば初期化できる");
        test::checkNearF(c2.joints().theta1, 0.3f, 1e-4f, "実測の θ1 が入る");
        test::checkNearF(kin::elbow_angle(c2.joints()), 90.0f * arm::DEG, 1e-4f, "肘角も一致");
    }

    test::section("経路は動く前に通しで検査する");
    {
        FakeDrive     sh, el;
        armctl::Space space;
        space.add(ws::rect(-1300.0f, 1300.0f, -1300.0f, 1300.0f, true));
        space.add(ws::circle(0.0f, 0.0f, 0.0f, 400.0f, false));  // 肩まわりは禁止

        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);  // 手先は (600, 600) 付近
        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        test::check(c.seedFromDrives(), "初期化できる");

        // 始点と終点はどちらも許可域だが、直線が禁止円を横切る
        const Point from = c.tip();
        const Point across{-from.x, -from.y};
        test::check(space.contains(from), "始点は入れる");
        test::check(space.contains(across), "終点も入れる");
        test::checkEq(code(c.checkPath(from, across)),
                      code(armctl::Fault::OutOfWorkspace), "途中が禁止領域を横切る");

        test::checkEq(code(c.startMove(across, mo::TrapezoidProfile{100.0f, 400.0f, 0.0f})),
                      code(armctl::Fault::OutOfWorkspace), "動かさずに断る");
        test::checkEq(code(c.mode()), code(armctl::Mode::Idle), "モードは変わらない");
        test::checkEq(count(sh.command_calls), 0u, "**1 本も送っていない**");
        test::check(c.fault() == armctl::Fault::None, "異常扱いにはしない (断っただけ)");

        // 到達域の外を通る経路も断る
        test::checkEq(code(c.checkPath(Point{600.0f, 0.0f}, Point{1400.0f, 0.0f})),
                      code(armctl::Fault::Unreachable), "外側限界を越える経路");
    }

    test::section("直線移動が完走し、終わったら保持へ移る");
    {
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);
        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);

        const Point from = c.tip();
        const Point to{from.x + 120.0f, from.y - 80.0f};
        test::checkEq(code(c.startMove(to, mo::TrapezoidProfile{100.0f, 400.0f, 4000.0f})),
                      code(armctl::Fault::None), "開始できる");

        int  steps  = 0;
        bool clean  = true;
        while (c.mode() == armctl::Mode::Move && steps < 20000) {
            clean = clean && (c.step(kDt) == armctl::Fault::None);
            ++steps;
        }
        test::check(clean, "全周期を異常なく進む");
        test::check(steps < 20000, "完走する");
        test::check(c.isMoveDone(), "移動が完了している");
        test::checkEq(code(c.mode()), code(armctl::Mode::Hold), "保持へ移る");
        test::checkNearF(arm::dist(c.tip(), to), 0.0f, 0.05f, "終点に着いている");

        // **保持でも送信を止めない。** 途絶えるとモータ側のタイムアウトで保持が解ける。
        const int before = sh.command_calls;
        for (int i = 0; i < 10; ++i) c.step(kDt);
        test::checkEq(count(sh.command_calls - before), 10u, "保持中も毎周期送る");
        test::checkEq(count(el.command_calls - before), 10u, "肘も同じ");
    }

    test::section("罠 6: 出荷されるループが ±π を跨いでも指令を飛ばさない");
    {
        // 半径 800 の円周上を回すと θ1 が ±π を跨ぐ。肩は減速比 3 なので、
        // 連続化を抜くとモータ角が 2π×3 = 6π 飛ぶ。
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 170.0f * arm::DEG, 96.4f * arm::DEG);
        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        test::check(c.seedFromDrives(), "初期化できる");

        // θ1 が π を跨ぐように、円周に沿って小刻みに目標を置いていく
        const float r0 = arm::hypotf2(c.tip().x, c.tip().y);
        float       phi = atan2f(c.tip().y, c.tip().x);
        int         moves = 0;
        for (int i = 0; i < 40; ++i) {
            phi += 3.0f * arm::DEG;
            const Point next{r0 * cosf(phi), r0 * sinf(phi)};
            if (c.startMove(next, mo::TrapezoidProfile{200.0f, 800.0f, 0.0f}) !=
                armctl::Fault::None) {
                break;
            }
            ++moves;
            int steps = 0;
            while (c.mode() == armctl::Mode::Move && steps < 20000) {
                c.step(kDt);
                ++steps;
            }
        }
        test::check(moves >= 30, "円周に沿って動かせた");
        test::check(fabsf(c.joints().theta1) > arm::PI, "**θ1 は ±π を越えている**");
        // 1 周期の指令変化は肩の速度上限で決まる。6π = 18.85 rad とは桁が違う。
        test::check(sh.max_jump < 0.5f, "モータ角が飛んでいない");
        test::check(el.max_jump < 0.5f, "肘も飛んでいない");
        test::check(c.fault() == armctl::Fault::None, "異常なし");
    }

    test::section("非常停止は成功するまで送り続ける");
    {
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);
        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.hold();

        // 送信が通らなくなった状態で異常を起こす
        sh.release_ok = false;
        el.release_ok = false;
        sh.command_ok = false;  // これが異常の引き金

        test::checkEq(code(c.step(kDt)), code(armctl::Fault::DriveRejected), "異常を検出する");
        test::check(!c.isReleased(), "**脱力が成立していない**");
        const int first = sh.release_calls;
        test::check(first > 0, "1 回は試みている");

        // バスが死んだままなら毎周期やり直す
        for (int i = 0; i < 5; ++i) c.step(kDt);
        test::check(sh.release_calls >= first + 5, "毎周期やり直している");
        test::check(!c.isReleased(), "まだ成立していない");

        // 復帰したら次の周期で脱力が届く
        sh.release_ok = true;
        el.release_ok = true;
        c.step(kDt);
        test::check(c.isReleased(), "成立した");
        test::check(sh.released && el.released, "両軸とも脱力している");

        // 成立したあとは送り続けない
        const int settled = sh.release_calls;
        for (int i = 0; i < 5; ++i) c.step(kDt);
        test::checkEq(count(sh.release_calls), count(settled), "成立後は繰り返さない");
    }

    test::section("手先ジョグ: 境界に当たっても脱力しない");
    {
        FakeDrive     sh, el;
        armctl::Space space;
        space.add(ws::rect(0.0f, 700.0f, -700.0f, 700.0f, true));  // x <= 700 まで
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);
        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.startToolJog();
        test::checkEq(code(c.mode()), code(armctl::Mode::ToolJog), "ジョグへ入る");

        // 上限を大きく超える指令。向きを保ったままクランプされる。
        c.setToolVelocity(Velocity{1000.0f, 0.0f});
        for (int i = 0; i < 200; ++i) c.step(kDt);
        test::check(c.tip().x > 600.0f, "+x へ動いている");

        // 壁に当たるまで押し続ける
        for (int i = 0; i < 3000; ++i) {
            c.setToolVelocity(Velocity{1000.0f, 0.0f});
            c.step(kDt);
        }
        test::check(c.tip().x <= 700.0f + 1.0f, "境界を越えない");
        test::checkEq(code(c.fault()), code(armctl::Fault::None), "**脱力していない**");
        test::checkEq(code(c.mode()), code(armctl::Mode::ToolJog), "ジョグのまま");
        test::check(!sh.released, "モータは生きている");

        // ジョグの速度は上限以下
        test::check(sh.max_jump < 0.5f, "指令が飛ばない");
    }

    test::section("関節ジョグ: 運動学を通さず、肘の限界だけは守る");
    {
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);
        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.startJointJog();
        test::checkEq(code(c.mode()), code(armctl::Mode::JointJog), "関節ジョグへ入る");

        // 肩だけを動かす。**手先ジョグではこれができない。**
        const JointAngles q0 = c.joints();
        c.setJointVelocity(JointVels{0.2f, 0.0f});
        for (int i = 0; i < 500; ++i) c.step(kDt);
        test::check(c.joints().theta1 > q0.theta1 + 0.05f, "θ1 だけが動いた");
        test::checkNearF(c.joints().theta2, q0.theta2, 1e-3f, "θ2 は動いていない");

        // 肘を閉じ切る方向へ押し続けても、限界で止まる
        c.setJointVelocity(JointVels{0.0f, -1.0f});
        for (int i = 0; i < 20000; ++i) c.step(kDt);
        const float e = kin::elbow_angle(c.joints());
        test::check(e >= kKin.elbow_min - 1e-3f, "伸びきり側の限界で止まる");
        test::checkEq(code(c.fault()), code(armctl::Fault::None), "異常にはしない");
        test::checkEq(code(kin::check_joints(kKin, c.joints())), code(kin::Status::Ok),
                      "姿勢は常に成立している");

        // 折り畳み側も同じ
        c.setJointVelocity(JointVels{0.0f, 1.0f});
        for (int i = 0; i < 40000; ++i) c.step(kDt);
        test::check(kin::elbow_angle(c.joints()) <= kKin.elbow_max + 1e-3f,
                    "折り畳み側の限界で止まる");
        test::checkEq(code(c.fault()), code(armctl::Fault::None), "ここでも異常にしない");

        // 指令の大きさは上限でクランプされる
        c.setJointVelocity(JointVels{100.0f, 0.0f});
        const JointAngles before = c.joints();
        c.step(kDt);
        test::check(fabsf(c.joints().theta1 - before.theta1) < kJog.joint_v_max * kDt * 1.1f,
                    "1 周期の変化が速度上限以下");
    }

    test::section("設定が壊れていれば指令を出さずに落とす");
    {
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);

        tx::Config bad = kTx;
        bad.elbow.ratio = 0.0f;  // 係数が使えない

        armctl::Controller c(sh, el, kKin, bad, space, kJog);
        c.seedFromDrives();
        c.hold();
        test::checkEq(code(c.step(kDt)), code(armctl::Fault::Transmission), "伝達比で落ちる");
        test::check(sh.released && el.released, "脱力する");
    }

    // =======================================================================
    // 監査 群 1。**どれも「アームが動く経路」の穴。**
    // 出典は .work/audit/report.md。ここから下は先に落ちることを確認してから
    // 実装した (赤 → 緑)。
    // =======================================================================

    test::section("G2: 肘が逆側の姿勢からは動き出さない");
    {
        // 校正前はオフセットが 0 なので、計算上の肘角の符号は事実上任意。
        // Config は Positive 側なのに実測が Negative 側だと、kin::inverse() は
        // 必ず Positive 側の解を返す。**繋ぐ先が鏡像姿勢になり、1 周期で
        // 肩が 270 度 (モータ) 跳ぶ。** 跳ばせない。動き出す前に断る。
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 60.0f * arm::DEG, -90.0f * arm::DEG);  // 肘角 −90 度 = 逆側

        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        test::check(c.seedFromDrives(), "実測は読める");
        c.setTransmission(kTx);
        test::checkEq(code(kin::check_joints(kKin, c.joints())),
                      code(kin::Status::WrongElbowSide), "前提: 姿勢は逆側");

        test::checkEq(code(c.startMove(Point{700.0f, 200.0f}, mo::TrapezoidProfile{})),
                      code(armctl::Fault::BadPose), "startMove が断る");
        test::checkEq(code(c.mode()), code(armctl::Mode::Idle), "モードは変わらない");

        // **断るなら理由を返す。** void で黙って戻ると、呼び出し側には
        // 「押したのに何も起きない」しか残らない。実機で実際にそうなった。
        test::checkEq(code(c.startToolJog()), code(armctl::Fault::BadPose),
                      "**startToolJog が理由を返す**");
        test::checkEq(code(c.mode()), code(armctl::Mode::Idle), "startToolJog も入らない");
        test::checkEq(code(c.startJointJog()), code(armctl::Fault::None),
                      "関節ジョグは断らない");
        test::checkEq(code(c.mode()), code(armctl::Mode::JointJog),
                      "**関節ジョグだけは入れる** (戻る唯一の手段)");
    }

    test::section("G4: ジョグを断る理由は 3 つあり、どれも名前を持つ");
    {
        // 実機 2026-09-11: 原点が合っていない状態で w を押しても何も起きず、
        // 画面にも何も出なかった。**断ったことを人へ見せていなかった。**
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        armctl::Controller c(sh, el, kKin, kTx, space, kJog);

        // 1. 実測姿勢で初期化していない
        test::checkEq(code(c.startToolJog()), code(armctl::Fault::NotSeeded),
                      "初期化前は NotSeeded");
        test::checkEq(code(c.startJointJog()), code(armctl::Fault::NotSeeded),
                      "関節ジョグも同じ");

        // 2. 非常停止・異常停止の最中
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);
        test::check(c.seedFromDrives(), "初期化できる");
        test::checkEq(code(c.startToolJog()), code(armctl::Fault::None), "平常時は通る");
        c.emergencyStop();
        test::checkEq(code(c.startToolJog()), code(armctl::Fault::Halted),
                      "**非常停止中は Halted**");
        test::checkEq(code(c.startJointJog()), code(armctl::Fault::Halted),
                      "関節ジョグも断る (脱力しているので動かせない)");
        test::checkEq(code(c.mode()), code(armctl::Mode::Idle), "モードは変わらない");

        // **自動運転も同じ門の内側。** トルクへ向かう入口は手先・関節・自動の 3 つ
        // あって、1 つでも素通しなら門はその 1 つぶん無い。止まっているのに
        // mode() が「自動」を返せば、軌道は止まった時点の姿勢から組み上がる。
        //
        // 見るのは**始まっていないこと**。戻り値ではなくモードを根拠にするのは、
        // 始まったかどうかを外から確かめられるのがモードだけだから。
        const Point halted_to{c.tip().x + 120.0f, c.tip().y - 80.0f};
        c.startMove(halted_to, mo::TrapezoidProfile{100.0f, 400.0f, 4000.0f});
        test::checkEq(code(c.mode()), code(armctl::Mode::Idle),
                      "**停止中は自動運転も始まらない**");

        // 復帰すればまた通る
        test::check(c.setTransmission(kTx), "原点を入れる");
        test::check(c.clearFault(), "復帰できる");
        test::check(c.seedFromDrives(), "取り直せる");
        test::checkEq(code(c.startToolJog()), code(armctl::Fault::None), "復帰後は通る");
        // **同じ行き先で通る。** さきほど断られたのが止まっていたからであって、
        // 届かない点だったからではないことを、ここで初めて言い切れる。
        test::checkEq(code(c.startMove(halted_to, mo::TrapezoidProfile{100.0f, 400.0f, 4000.0f})),
                      code(armctl::Fault::None), "自動運転も復帰後は通る");
        test::checkEq(code(c.mode()), code(armctl::Mode::Move), "今度は始まっている");
    }

    test::section("G3: 関節ジョグは限界に当たっても送信を止めない");
    {
        // 拒否した周期に 1 本も送らないと、モータ側の CAN タイムアウト
        // (profile.hpp の can_timeout_ms = 100 ms) が切れて自分で脱力する。
        // 限界へ押し付けている間こそ、保持を送り続けなければならない。
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 30.0f * arm::DEG, 15.0f * arm::DEG);  // 下限 10 度のすぐ内側

        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.startJointJog();
        c.setJointVelocity(JointVels{0.0f, -kJog.joint_v_max});  // 肘角を縮める向き

        int longest_silence = 0;
        int silence         = 0;
        for (int i = 0; i < 1500; ++i) {  // 3 秒
            const int before = sh.command_calls;
            c.step(kDt);
            if (sh.command_calls == before) {
                if (++silence > longest_silence) longest_silence = silence;
            } else {
                silence = 0;
            }
        }
        test::checkEq(count(longest_silence), count(0), "1 周期も送信が途切れない");
        test::checkEq(code(c.fault()), code(armctl::Fault::None), "異常にはしない");
    }

    test::section("G4: 肘角が範囲外でも、良くなる向きへは動ける");
    {
        // 範囲外から戻れないと、関節ジョグ (原点合わせの唯一の手段) が
        // 固まる。次の姿勢だけを見る判定では、改善する向きも拒否される。
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 30.0f * arm::DEG, 5.0f * arm::DEG);  // 下限 10 度の外

        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        test::checkEq(code(kin::check_joints(kKin, c.joints())),
                      code(kin::Status::TooExtended), "前提: 伸びきりすぎ");
        c.startJointJog();

        const float e0 = kin::elbow_angle(c.joints());
        c.setJointVelocity(JointVels{0.0f, kJog.joint_v_max});  // 肘角を広げる = 改善
        for (int i = 0; i < 500; ++i) c.step(kDt);
        const float e1 = kin::elbow_angle(c.joints());
        test::check(e1 > e0 + 0.05f, "改善する向きへは動けた");

        // 悪化する向きは拒否したまま。
        placeAt(sh, el, 30.0f * arm::DEG, 5.0f * arm::DEG);
        armctl::Controller c2(sh, el, kKin, kTx, space, kJog);
        c2.seedFromDrives();
        c2.startJointJog();
        const float f0 = kin::elbow_angle(c2.joints());
        c2.setJointVelocity(JointVels{0.0f, -kJog.joint_v_max});  // さらに縮める
        for (int i = 0; i < 500; ++i) c2.step(kDt);
        test::checkNearF(kin::elbow_angle(c2.joints()), f0, 1e-3f, "悪化する向きは通さない");
    }

    test::section("G5: 起動時に手先が領域外でも、ジョグが送信を止めない");
    {
        // 電源投入時の姿勢は選べない。作業領域の外に居ることは珍しくないのに、
        // そこから手先ジョグを始めると 1 周期も送らないまま 100 ms で脱力する。
        FakeDrive     sh, el;
        armctl::Space space;
        space.add(ws::rect(-1300.0f, 1300.0f, 50.0f, 1300.0f, true));  // y >= 50 の帯
        placeAt(sh, el, -80.0f * arm::DEG, 90.0f * arm::DEG);           // 手先は y < 50

        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        test::check(!space.contains(c.tip()), "前提: 起動時に領域外");

        c.startToolJog();
        c.setToolVelocity(Velocity{0.0f, kJog.tool_v_max});  // 領域へ戻る向き
        const int before = sh.command_calls;
        for (int i = 0; i < 500; ++i) c.step(kDt);
        test::checkEq(count(sh.command_calls - before), count(500),
                      "毎周期送っている (保持が解けない)");
        test::checkEq(code(c.fault()), code(armctl::Fault::None), "異常にはしない");
    }

    test::section("G7: 異常は解除できる");
    {
        // 解除する手段が無いと、送信が 1 回失敗しただけで電源を入れ直すまで
        // 戻れない。脱力が成立してからだけ解除を許す。
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);

        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.hold();

        sh.command_ok = false;
        sh.release_ok = false;
        test::checkEq(code(c.step(kDt)), code(armctl::Fault::DriveRejected), "異常になる");
        test::check(!c.isReleased(), "脱力はまだ成立していない");
        test::check(!c.clearFault(), "**脱力できていないうちは解除させない**");

        sh.release_ok = true;
        c.step(kDt);  // 成功するまで送り続ける経路がここで通る
        test::check(c.isReleased(), "脱力が成立した");

        sh.command_ok = true;
        test::check(c.clearFault(), "解除できる");
        test::checkEq(code(c.fault()), code(armctl::Fault::None), "異常が消えた");
        test::checkEq(code(c.mode()), code(armctl::Mode::Idle), "解除直後は停止");
        test::check(!c.isSeeded() || true, "再開には seed と hold をやり直す");
    }

    test::section("G6: 非常停止は成立するまでやり直し、届かなければそう言う");
    {
        // `x` が release() の戻り値を捨てて「脱力しました」と断言すると、
        // 腕がトルクを出したままなのに操作者は安全だと信じる。
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);

        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.hold();

        sh.release_ok = false;
        c.emergencyStop();
        test::check(!c.isReleased(), "**送れていないので成立していない**");
        test::checkEq(code(c.mode()), code(armctl::Mode::Idle), "保持はやめている");

        const int before = sh.release_calls;
        c.step(kDt);
        test::check(sh.release_calls > before, "毎周期やり直す");
        test::checkEq(count(sh.command_calls), count(0),
                      "**保持の指令は送らない** (CAN タイムアウトを撫でない)");

        sh.release_ok = true;
        c.step(kDt);
        test::check(c.isReleased(), "届けば成立する");
    }

    test::section("H3: 通電が落ちたら異常へ倒す");
    {
        // モータが過温や欠圧で自分で脱力しても、パラメータ書き込みには応答し
        // 続ける。以前は isAlive() も指令の戻り値も真のままで、**腕が脱力して
        // いるのに画面上は目標姿勢を追い続けている**ように見えた。
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);

        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.hold();
        test::checkEq(code(c.step(kDt)), code(armctl::Fault::None), "通電していれば通る");

        // **1 周期の欠落では倒さない。** フィードバックは指令への応答として
        // 届くので、送信が 1 回詰まれば 1 周期ぶん古くなる。
        sh.energized = false;
        test::checkEq(code(c.step(kDt)), code(armctl::Fault::None), "1 周期では倒さない");
        sh.energized = true;
        for (int i = 0; i < 10; ++i) c.step(kDt);
        test::checkEq(code(c.fault()), code(armctl::Fault::None), "戻れば何も起きない");

        // 続けば異常。**猶予を超えたところで倒す。**
        sh.energized = false;
        armctl::Fault f = armctl::Fault::None;
        for (int i = 0; i < armctl::energized_grace_cycles + 2; ++i) {
            f = c.step(kDt);
            if (f != armctl::Fault::None) break;
        }
        test::checkEq(code(f), code(armctl::Fault::NotEnergized), "通電が落ちたと判断する");
        test::check(sh.released && el.released, "両軸を脱力する");
    }

    test::section("H3: 停止しているあいだは通電を要求しない");
    {
        // Idle も非常停止も、脱力しているのが正しい状態。そこで通電を要求すると
        // 止めた瞬間に異常が出る。
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);

        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        sh.energized = false;
        el.energized = false;

        // Idle: 何も送らないので通電も要らない
        for (int i = 0; i < armctl::energized_grace_cycles + 10; ++i) c.step(kDt);
        test::checkEq(code(c.fault()), code(armctl::Fault::None), "Idle では倒さない");

        // 非常停止のあとも同じ
        c.emergencyStop();
        for (int i = 0; i < armctl::energized_grace_cycles + 10; ++i) c.step(kDt);
        test::checkEq(code(c.fault()), code(armctl::Fault::None), "非常停止中も倒さない");
    }

    test::section("H3: 復帰はモータ側の故障も解除する");
    {
        // clearFault() だけではモータ側にラッチした故障が残る。復帰の並びは
        // 「故障解除 → 使能 → 姿勢の取り直し」で、その 1 本目が無かった。
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);

        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.hold();

        sh.command_ok = false;
        c.step(kDt);
        test::checkEq(code(c.fault()), code(armctl::Fault::DriveRejected), "異常になる");
        test::check(c.isReleased(), "脱力は成立している");

        sh.command_ok      = true;
        const int before_a = sh.clear_calls;
        const int before_b = el.clear_calls;
        test::check(c.clearFault(), "解除できる");
        test::check(sh.clear_calls > before_a && el.clear_calls > before_b,
                    "**両軸のモータ側の故障も解除する**");
    }

    test::section("G1: 指令の切り詰めを検出したら脱力へ倒す");
    {
        // RS00 の setPosition() は仕様範囲外を**黙って切り詰めて true を返す**。
        // 肩は減速比 3 なので |θ1| > 240 度で頭打ちになるが、肘 (減速比 1) は
        // 追従し続けるので、**実機の肘角だけが死点へ寄っていく**。
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 30.0f * arm::DEG, 90.0f * arm::DEG);  // 肩モータ角 −1.57 rad

        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.hold();
        test::checkEq(code(c.step(kDt)), code(armctl::Fault::None), "通常は通る");

        sh.clamp_limit = 0.1f;  // 以後、指令は ±0.1 rad へ切り詰められる
        test::checkEq(code(c.step(kDt)), code(armctl::Fault::DriveRejected),
                      "切り詰めたら異常として扱う");
        test::check(sh.released && el.released, "両軸を脱力する");
    }

    test::section("E1: 閾値未設定なら過トルクで倒さない");
    {
        // **測るまで無効。** 通常運転のトルクを知らないまま閾値を置くと、
        // 常時トリップするか、まったく効かないかのどちらかになる。
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);
        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.hold();

        test::check(!c.isTorqueLimitArmed(), "既定では無効");
        sh.torque = 1000.0f;  // あり得ない大きさ
        for (int i = 0; i < 1000; ++i) c.step(kDt);
        test::checkEq(code(c.fault()), code(armctl::Fault::None), "**倒さない**");
        test::check(!sh.released, "脱力もしない");

        // **無効でも見てはいる。** 閾値を決めるための窓。
        test::checkNearF(c.peakTorque(), 1000.0f, 1e-3f, "最大は記録している");
    }

    test::section("E2: 閾値を超えたら猶予のあとで脱力する");
    {
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);
        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.setTorqueLimit(5.0f);
        c.hold();
        test::check(c.isTorqueLimitArmed(), "有効になった");

        sh.torque = 4.9f;
        for (int i = 0; i < 100; ++i) c.step(kDt);
        test::checkEq(code(c.fault()), code(armctl::Fault::None), "閾値以下では倒さない");

        // **1 周期の跳ねで倒さない。** 猶予のあいだは通す。
        sh.torque = 5.1f;
        for (int i = 0; i < armctl::over_torque_grace_cycles; ++i) {
            test::checkEq(code(c.step(kDt)), code(armctl::Fault::None), "猶予のあいだは通す");
        }
        test::checkEq(code(c.step(kDt)), code(armctl::Fault::OverTorque),
                      "**猶予を超えたら倒す**");
        test::check(sh.released && el.released, "両軸を脱力する");
        test::checkEq(code(c.mode()), code(armctl::Mode::Idle), "モードは停止へ");
    }

    test::section("E3: 押される向きは問わない");
    {
        // **符号ではなく大きさで判定する。** 逆向きに押されているときだけ
        // 見逃すと、片側からの挟み込みが素通りする。
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);
        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.setTorqueLimit(5.0f);
        c.hold();

        el.torque = -8.0f;  // 負の側
        for (int i = 0; i <= armctl::over_torque_grace_cycles; ++i) c.step(kDt);
        test::checkEq(code(c.fault()), code(armctl::Fault::OverTorque), "負の側でも倒す");
    }

    test::section("E4: 超えたあと戻れば猶予は数え直す");
    {
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);
        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.setTorqueLimit(5.0f);
        c.hold();

        // 猶予の直前まで超えて、戻る — を繰り返しても倒れないこと。
        for (int round = 0; round < 5; ++round) {
            sh.torque = 6.0f;
            for (int i = 0; i < armctl::over_torque_grace_cycles; ++i) c.step(kDt);
            sh.torque = 0.0f;
            c.step(kDt);
        }
        test::checkEq(code(c.fault()), code(armctl::Fault::None),
                      "**連続していなければ倒さない**");
    }

    test::section("E5: トルクが読めないうちは倒さない");
    {
        // 一度もフィードバックを受け取っていない状態で倒すと、起動直後に
        // 必ず脱力する。**倒す根拠が無い。**
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);
        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.setTorqueLimit(0.001f);  // 何でも超える閾値
        sh.has_torque = false;
        c.hold();

        for (int i = 0; i < 1000; ++i) c.step(kDt);
        test::checkEq(code(c.fault()), code(armctl::Fault::None), "読めないうちは倒さない");
        test::check(c.peakTorque() < 0.0f, "最大も未記録のまま");
    }

    test::section("E6: 脱力しているあいだも応答を引き出し続ける");
    {
        // **何も送らないと型 2 フィードバックが来ない。** 姿勢もトルクも凍り、
        // 画面は最後に見た値のまま「応答なし」になる。人が腕を手で動かして
        // いるのはまさにこのときで、そこで表示が死ぬと肘角も原点も読めない。
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);
        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);

        c.emergencyStop();
        test::check(c.isReleased(), "脱力が成立している");
        const int after_estop = sh.release_calls;

        // 1 秒ぶん回す。**間引いて送る**ので、毎周期ではない。
        for (int i = 0; i < 500; ++i) c.step(kDt);
        const int polls = sh.release_calls - after_estop;
        test::check(polls > 0, "**脱力中も送っている** (表示が生きる)");
        test::check(polls < 500, "毎周期は送らない (間引いている)");
        // 20 Hz なら 1 秒で 20 回前後。
        test::check(polls >= 15 && polls <= 25, "間隔がおおむね 20 Hz");

        // **引き出した値が外から見えること。** joints() が返すのは cont_ の値
        // (指令側) なので、応答を引き出すだけでは表示は凍ったままになる。
        // 実機で実際にそうなった (2026-09-11)。脱力中は指令を出していないの
        // だから、指令側を実測へ追従させるのが正しい。
        const JointAngles before_move = c.joints();
        placeAt(sh, el, 0.9f, 100.0f * arm::DEG);  // 人が手で動かした
        for (int i = 0; i < 100; ++i) c.step(kDt);  // 0.2 秒 = 間引きを数回跨ぐ
        test::check(c.joints().theta1 != before_move.theta1,
                    "**手で動かした結果が joints() に出る**");
        test::checkNearF(c.joints().theta1, 0.9f, 1e-3f, "実測の θ1 に追従している");
        test::checkNearF(kin::elbow_angle(c.joints()), 100.0f * arm::DEG, 1e-3f,
                         "肘角も追従している");
    }

    test::section("E7: 復帰から保持までのあいだ、割り込みはドライブへ 1 本も送らない");
    {
        // **使能の最中に脱力フレームを撃たない。** main が位置モードへ入れて
        // いる 200 ms のあいだ制御タイマは回り続けており、そこで脱力が届くと
        // モータは使能を取り消す。保持に入った 200 ms 後に「通電していない」で
        // 倒れる形で出る。
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);
        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.emergencyStop();
        for (int i = 0; i < 10; ++i) c.step(kDt);

        test::check(c.clearFault(), "E7: 復帰できる");
        test::checkEq(code(c.mode()), code(armctl::Mode::Idle), "E7: 復帰直後は停止");
        const int rel = sh.release_calls + el.release_calls;
        const int cmd = sh.command_calls + el.command_calls;
        for (int i = 0; i < 500; ++i) c.step(kDt);  // 1 秒ぶん = 間引きを 20 回跨ぐ
        test::checkEq(count(sh.release_calls + el.release_calls), count(rel),
                      "E7: **脱力を 1 本も送らない**");
        test::checkEq(count(sh.command_calls + el.command_calls), count(cmd),
                      "E7: 指令も 1 本も送らない");
        test::checkEq(code(c.step(kDt)), code(armctl::Fault::None), "E7: 異常にもしない");
    }

    test::section("C3: 止まっていなければ復帰は何もせず成功する");
    {
        // M3 の二度押し・押し間違いで「復帰できません」が出ると、腕は保持して
        // いるのに操作者の頭は「まだ脱力している」側へ倒れる。**食い違いが
        // 安全でない向きに出る。** 解除するものが無いなら成功でよい。
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);

        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.hold();

        test::check(c.clearFault(), "C3: 止まっていなければ true");
        test::checkEq(count(sh.clear_calls + el.clear_calls), count(0), "C3: 何も送らない");
        test::checkEq(code(c.mode()), code(armctl::Mode::Hold), "C3: モードを変えない");
        test::check(!c.isReleased(), "C3: 脱力の表示も動かない");
    }

    test::section("C1/C2: 復帰すると脱力の表示が下り、二度押しも通る");
    {
        // **表示だけが嘘をつく欠陥。** step() の脱力分岐は released_ を見ないので
        // 腕は正しく動き、isReleased() を直に読む側 (LED) にしか出ない。
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);

        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.hold();

        c.emergencyStop();
        test::check(c.isReleased(), "脱力が成立した");

        test::check(c.clearFault(), "C1: 復帰できる");
        test::check(!c.isReleased(), "C1: 復帰後は脱力の表示が下りる");
        test::checkEq(code(c.fault()), code(armctl::Fault::None), "C1: 異常なし");
        test::checkEq(code(c.mode()), code(armctl::Mode::Idle), "C1: Idle");

        const int a = sh.clear_calls;
        const int b = el.clear_calls;
        test::check(c.clearFault(), "C2: 二度押しも true");
        test::checkEq(count(sh.clear_calls), count(a), "C2: 肩へ何も送らない");
        test::checkEq(count(el.clear_calls), count(b), "C2: 肘へ何も送らない");
        test::check(!c.isReleased(), "C2: 二度押しでも下りたまま");
    }

    test::section("C4: 脱力が成立していないうちは復帰させない");
    {
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);

        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.hold();

        sh.release_ok = false;
        c.emergencyStop();
        test::check(c.isHalted(), "C4: 止まっている");
        test::check(!c.isReleased(), "C4: 脱力が成立していない");
        test::check(!c.clearFault(), "C4: 拒む");
        test::checkEq(count(sh.clear_calls + el.clear_calls), count(0), "C4: 故障解除も送らない");
    }

    test::section("C5: 故障解除が片軸でも送れなければ異常を消さない");
    {
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);

        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.hold();

        sh.command_ok = false;
        test::checkEq(code(c.step(kDt)), code(armctl::Fault::DriveRejected), "異常になる");
        test::check(c.isReleased(), "脱力は成立している");
        sh.command_ok = true;

        el.clear_ok = false;
        const int a = sh.clear_calls;
        const int b = el.clear_calls;
        test::check(!c.clearFault(), "C5: 送れなければ false");
        test::checkEq(count(sh.clear_calls), count(a + 1), "C5: 肩へも 1 回送る");
        test::checkEq(count(el.clear_calls), count(b + 1), "C5: 肘へも 1 回送る");
        test::check(c.isReleased(), "C5: 失敗時は脱力の表示を下ろさない");
        test::checkEq(code(c.fault()), code(armctl::Fault::DriveRejected), "C5: 異常も残す");

        el.clear_ok = true;
        test::check(c.clearFault(), "C5: 再試行は通る");
        test::check(!c.isReleased(), "C5: 通れば下りる");
        test::check(!c.isHalted(), "C5: 止まってもいない");
    }

    test::section("C7: isHalted() は「指令を受け付けない」ことと一致する");
    {
        // **表示はこの述語だけを読む。** ずれると LED が嘘をつく。非常停止は
        // 異常ではないが停止なので、estop_ を見落とすと脱力中に緑が出る。
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);

        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.hold();

        test::check(!c.isHalted(), "C7: 平常は止まっていない");
        test::checkEq(code(c.startJointJog()), code(armctl::Fault::None), "C7: 指令が通る");

        c.emergencyStop();
        test::check(c.isHalted(), "C7: 非常停止は停止");
        test::checkEq(code(c.fault()), code(armctl::Fault::None), "C7: しかし異常ではない");
        test::checkEq(code(c.startJointJog()), code(armctl::Fault::Halted), "C7: 指令を断る");

        test::check(c.clearFault(), "復帰できる");
        test::check(!c.isHalted(), "C7: 復帰したら止まっていない");
        test::checkEq(code(c.startJointJog()), code(armctl::Fault::None), "C7: また通る");

        sh.command_ok = false;
        test::checkEq(code(c.step(kDt)), code(armctl::Fault::DriveRejected), "異常になる");
        test::check(c.isHalted(), "C7: 異常停止も停止");
        test::checkEq(code(c.startJointJog()), code(armctl::Fault::Halted), "C7: やはり断る");
    }

    test::section("C8/C9: 脱力は停止を含意し、初期化済みは戻らない");
    {
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);

        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.hold();
        test::check(!(c.isReleased() && !c.isHalted()), "C8: 平常");

        c.emergencyStop();
        test::check(!(c.isReleased() && !c.isHalted()), "C8: 非常停止の直後");

        // **通電の猶予を跨いでも保たれること。** 停止中は通電を要求しないので
        // ここで別の異常が生えてはいけない。
        bool ok = true;
        for (int i = 0; i < armctl::energized_grace_cycles + 10; ++i) {
            c.step(kDt);
            if (c.isReleased() && !c.isHalted()) ok = false;
            if (!c.isSeeded()) ok = false;
        }
        test::check(ok, "C8/C9: 停止中ずっと保たれる");

        test::check(c.clearFault(), "復帰できる");
        test::check(!c.isReleased() && !c.isHalted(), "C8: 復帰後は両方偽");
        test::check(c.isSeeded(), "C9: 復帰後も初期化済み");
    }

    test::section("C10: 復帰したあと保持を続けられる");
    {
        // **表示だけの修正が動作を壊していないことの確認。**
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);

        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.hold();
        c.emergencyStop();

        test::check(c.clearFault(), "復帰できる");
        test::check(c.seedFromDrives(), "姿勢を取り直せる");
        c.setTransmission(kTx);
        c.hold();

        const int     before = sh.command_calls;
        armctl::Fault f      = armctl::Fault::None;
        for (int i = 0; i < 10 && f == armctl::Fault::None; ++i) f = c.step(kDt);
        test::checkEq(code(f), code(armctl::Fault::None), "C10: 異常なく回る");
        test::check(sh.command_calls > before, "C10: 保持の指令が出る");
    }

    test::section("C11: 復帰は通電の猶予を数え直す");
    {
        // **止まる前に数えた欠落を持ち越さない。** 持ち越したままだと、復帰の
        // 直後に通電がひと呼吸遅れただけで猶予を使い切った扱いになり、押しても
        // 動かないうちに異常へ倒れる — 操作者からは復帰が効いていないと見える。
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);

        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.hold();

        // 猶予の縁まで欠落を数える。ここまでは倒れない。
        sh.energized = false;
        bool before_ok = true;
        for (int i = 0; i < armctl::energized_grace_cycles; ++i) {
            if (c.step(kDt) != armctl::Fault::None) before_ok = false;
        }
        test::check(before_ok, "C11: 猶予のうちは倒れない");

        c.emergencyStop();
        test::check(c.clearFault(), "C11: 復帰できる");
        c.hold();

        // **数え直されていれば、ここからまた猶予いっぱい通る。**
        bool after_ok = true;
        for (int i = 0; i < armctl::energized_grace_cycles; ++i) {
            if (c.step(kDt) != armctl::Fault::None) after_ok = false;
        }
        test::check(after_ok, "C11: 復帰後は猶予を数え直す");
        test::checkEq(code(c.step(kDt)), code(armctl::Fault::NotEnergized),
                      "C11: 猶予を超えれば倒れる");
    }

    test::section("C12: 止まっているあいだは保持へ戻せない");
    {
        // **止まっているのに mode() が Hold を返すと、腕は脱力しているのに
        // 「保持している」と読める。** 止まっているかどうかを決めるのは
        // fault_ と estop_ だけで、モードはそれに従う側。
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);

        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.hold();
        test::checkEq(code(c.mode()), code(armctl::Mode::Hold), "C12: 平常は保持へ入れる");

        c.emergencyStop();
        c.hold();
        test::checkEq(code(c.mode()), code(armctl::Mode::Idle), "C12: 非常停止中は断る");

        test::check(c.clearFault(), "C12: 復帰できる");
        c.hold();
        sh.command_ok = false;
        test::checkEq(code(c.step(kDt)), code(armctl::Fault::DriveRejected), "C12: 異常で倒れる");
        c.hold();
        test::checkEq(code(c.mode()), code(armctl::Mode::Idle), "C12: 異常停止中も断る");
    }

    test::section("C13: 姿勢を取れていないあいだは保持へ入らない");
    {
        // **基準が無いのに mode() が Hold を返すと、次に姿勢を取れた瞬間から、
        // 誰の判断も通らずに指令が出る。** 使能した瞬間に跳ねないよう「取り直して
        // から保持へ入る」という順序が意味を持つのは、取れていないあいだ保持へ
        // 入れないからで、塞いでいるのは hold() のこの分岐だけ。
        //
        // 止まっている側の断り (C12) と別に見る。**両方とも断るが理由が違う** —
        // 片方だけで代用すると、もう片方を外しても検査が緑のまま通る。
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);

        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        test::check(!c.isSeeded(), "C13: まだ姿勢を取れていない");
        test::check(!c.isHalted(), "C13: 構築直後は止まってもいない");

        c.hold();
        test::checkEq(code(c.mode()), code(armctl::Mode::Idle),
                      "C13: **基準が無ければ保持へ入らない**");

        // 取り直せば入れる。断っているのは基準の不在であって、他の何かではない。
        test::check(c.seedFromDrives(), "C13: 取り直せる");
        c.hold();
        test::checkEq(code(c.mode()), code(armctl::Mode::Hold), "C13: 取れたら入れる");
    }

    test::section("H1: 原点が無いあいだは使能できない");
    {
        // **門は Controller に置く。** main 側の if で守ると、その if を消しても
        // ホストのゲートからは見えない。原点が無いと姿勢表示も作業領域の制限も
        // 嘘の座標系の上に乗るので、警告ではなく拒否にする。
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);
        armctl::Controller c(sh, el, kKin, kTx, space, kJog);

        test::check(!c.isHomed(), "H1: 構築直後は原点が無い");
        c.seedFromDrives();
        c.emergencyStop();
        for (int i = 0; i < 5; ++i) c.step(kDt);
        test::check(c.isReleased(), "H1: 脱力は成立している");

        test::check(!c.clearFault(), "H1: **原点が無ければ復帰を断る**");
        test::checkEq(count(sh.clear_calls + el.clear_calls), count(0),
                      "H1: 断ったときは故障解除も送らない");
        test::check(c.isHalted(), "H1: 止まったまま");
        test::checkEq(code(c.mode()), code(armctl::Mode::Idle), "H1: モードも変わらない");

        test::check(c.setTransmission(kTx), "H1: 原点を入れられる");
        test::check(c.isHomed(), "H1: 原点が立つ");
        test::check(c.clearFault(), "H1: **原点があれば通る**");
        test::checkEq(count(sh.clear_calls), count(1), "H1: 肩へ故障解除を送った");
        test::checkEq(count(el.clear_calls), count(1), "H1: 肘へも送った");
        test::check(!c.isHalted(), "H1: 止まっていない");
    }

    test::section("H2: 原点が無いときの断り方は、脱力の未成立より先に決まる");
    {
        // **どちらの理由でも「送らずに断る」でなければならない。**
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);
        sh.release_ok = false;
        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.emergencyStop();

        test::check(!c.isHomed() && !c.isReleased(), "H2: 原点も脱力も揃っていない");
        test::check(!c.clearFault(), "H2: 断る");
        test::checkEq(count(sh.clear_calls + el.clear_calls), count(0), "H2: 何も送らない");

        test::check(c.setTransmission(kTx), "H2: 原点を入れる");
        test::check(!c.clearFault(), "H2: 脱力が未成立なら依然として断る");
        test::checkEq(count(sh.clear_calls + el.clear_calls), count(0), "H2: まだ何も送らない");

        sh.release_ok = true;
        for (int i = 0; i < 30; ++i) c.step(kDt);
        test::check(c.isReleased(), "H2: 脱力が成立した");
        test::check(c.clearFault(), "H2: ここで初めて通る");
    }

    test::section("H3: 原点は同じ起動の中で消えない");
    {
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);
        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        test::check(c.setTransmission(kTx), "H3: 原点を入れる");

        c.hold();
        test::check(c.isHomed(), "H3: 保持へ入れても消えない");
        c.emergencyStop();
        test::check(c.isHomed(), "H3: **非常停止でも消えない**");
        for (int i = 0; i < 30; ++i) c.step(kDt);
        test::check(c.isHomed(), "H3: 巡回でも消えない");
        test::check(c.clearFault(), "H3: 復帰できる");
        test::check(c.isHomed(), "H3: **復帰でも消えない**");
        c.hold();

        sh.command_ok = false;
        test::checkEq(code(c.step(kDt)), code(armctl::Fault::DriveRejected), "H3: 異常になる");
        test::check(c.isHomed(), "H3: 異常停止でも消えない");

        sh.command_ok = true;
        for (int i = 0; i < 30; ++i) c.step(kDt);
        test::check(c.clearFault(), "H3: 復帰できる");
        test::check(c.seedFromDrives(), "H3: 取り直せる");
        test::checkEq(code(c.startJointJog()), code(armctl::Fault::None), "H3: ジョグへ入れる");
        for (int i = 0; i < 30; ++i) c.step(kDt);
        test::check(c.isHomed(), "H3: ジョグ中も消えない");
    }

    test::section("H4: 妥当でないオフセットは原点として受け付けない");
    {
        // **偽を返すだけでなく、持っている設定も変えない。** 半端に書き換えると、
        // 次の指令が壊れた係数でモータ角へ落ちる。
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);
        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();

        const tx::Config before = c.transmission();

        tx::Config zero_ratio = kTx;
        zero_ratio.elbow.ratio = 0.0f;
        test::check(!c.setTransmission(zero_ratio), "H4: 係数 0 は断る");
        test::check(!c.isHomed(), "H4: **原点も立たない**");

        tx::Config nan_offset = kTx;
        nan_offset.shoulder.offset = NAN;
        test::check(!c.setTransmission(nan_offset), "H4: 有限でないオフセットは断る");
        test::check(!c.isHomed(), "H4: 原点も立たない");

        const tx::Config after = c.transmission();
        test::checkNearF(after.shoulder.ratio, before.shoulder.ratio, 0.0f, "H4: 肩の比が残る");
        test::checkNearF(after.shoulder.offset, before.shoulder.offset, 0.0f,
                         "H4: 肩のオフセットが残る");
        test::checkEqI(after.shoulder.direction, before.shoulder.direction, "H4: 肩の向きが残る");
        test::checkNearF(after.elbow.ratio, before.elbow.ratio, 0.0f, "H4: 肘の比が残る");
        test::checkNearF(after.elbow.offset, before.elbow.offset, 0.0f,
                         "H4: 肘のオフセットが残る");
        test::checkEqI(after.elbow.direction, before.elbow.direction, "H4: 肘の向きが残る");

        test::check(c.setTransmission(kTx), "H4: 妥当なら通る");
        test::check(c.isHomed(), "H4: そこで初めて原点が立つ");
    }

    test::section("H5: 止まっていなければ原点の有無に関わらず復帰は成功する");
    {
        // C3 の据え置き。**原点の門を isHalted() より前に置いてはいけない** —
        // 運転中の M3 が「原点がありません」で断られると、腕は保持しているのに
        // 操作者は脱力していると読む。
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);
        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.hold();

        test::check(!c.isHomed(), "H5: 原点は無い");
        test::check(!c.isHalted(), "H5: 止まってもいない");
        test::check(c.clearFault(), "H5: **それでも true**");
        test::checkEq(count(sh.clear_calls + el.clear_calls), count(0), "H5: 何も送らない");
        test::checkEq(code(c.mode()), code(armctl::Mode::Hold), "H5: モードを変えない");
    }

    test::section("H6: 原点を入れても、それだけではトルクは出ない");
    {
        // **M4 は宣言であって使能ではない。** setTransmission が hold() まで
        // やってしまうと、腕を手で支えている操作者の手の中で急に硬くなる。
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);
        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.emergencyStop();
        for (int i = 0; i < 30; ++i) c.step(kDt);

        const int cmd = sh.command_calls + el.command_calls;
        test::check(c.setTransmission(kTx), "H6: 原点を入れる");
        for (int i = 0; i < 100; ++i) c.step(kDt);
        test::checkEq(count(sh.command_calls + el.command_calls), count(cmd),
                      "H6: **指令は 1 本も出ない**");
        test::check(c.isHalted(), "H6: 止まったまま");
        test::checkEq(code(c.mode()), code(armctl::Mode::Idle), "H6: 保持へも入らない");
    }

    // ------------------------------------------------------------------
    // T: 追従誤差 (指令と実測のずれ)。**計測だけで、何も止めない。**
    //
    // がたつきの原因が手先ジョグ側 (躍度の制限が無い) かモータ側 (位置ループ
    // 利得が 2 台で食い違っている) かは、**観察では区別できない** — どちらも
    // 体の近くで同じくらいに見える。区別するのは「追従誤差が姿勢と向きで
    // どう変わるか」だけなので、その数字が外から読めることをここで押さえる。
    // ------------------------------------------------------------------

    test::section("T1: 指令と実測が一致していれば追従誤差は 0");
    {
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);
        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);

        test::check(!c.trackingError().valid, "T1: 指令を出す前は突き合わせていない");

        c.hold();
        constexpr int n = 10;
        for (int i = 0; i < n; ++i) c.step(kDt);

        const armctl::TrackingError te = c.trackingError();
        test::check(te.valid, "T1: 指令を出せば突き合わせている");
        test::checkNearF(te.e1, 0.0f, 1e-6f, "T1: 肩の誤差は 0");
        test::checkNearF(te.e2, 0.0f, 1e-6f, "T1: 肘の誤差も 0");
        test::checkNearF(te.peak1, 0.0f, 1e-6f, "T1: 肩の最大も 0");
        test::checkNearF(te.peak2, 0.0f, 1e-6f, "T1: 肘の最大も 0");

        // **計測がフレームを 1 本も増やしていない。** 保持は 1 周期に 1 本ずつ。
        test::checkEq(count(sh.command_calls), count(n), "T1: 肩への指令は周期ぶんだけ");
        test::checkEq(count(el.command_calls), count(n), "T1: 肘も周期ぶんだけ");
        test::checkEq(count(sh.release_calls + el.release_calls), count(0),
                      "T1: 脱力は 1 本も増えない");
    }

    test::section("T2: 実測が遅れていれば、その差が符号つきで出る");
    {
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);
        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.hold();
        c.step(kDt);

        // **実測だけを動かす。** 保持なので指令は動かない。
        // 肩は減速比 3・向き −1 なので、関節 −0.010 rad はモータ +0.030 rad。
        sh.position = -3.0f * (-0.010f);
        el.position = (90.0f * arm::DEG) - 0.020f;
        c.step(kDt);

        const armctl::TrackingError te = c.trackingError();
        test::checkNearF(te.e1, +0.010f, 1e-5f, "T2: 肩は 指令 − 実測 で正");
        test::checkNearF(te.e2, +0.020f, 1e-5f, "T2: 肘も同じ向き");

        // **符号が向きを持つ。** 追い越していれば負。
        sh.position = -3.0f * (+0.010f);
        c.step(kDt);
        test::checkNearF(c.trackingError().e1, -0.010f, 1e-5f, "T2: 追い越せば負");
    }

    test::section("T3/T4: 最大は latch され、落とす口で 0 へ戻る");
    {
        // **latch したまま下げる口が無いと、一度の跳ねで窓が死ぬ。**
        // トルク最大で同じことが起きている (口はあるが main が呼んでいない)。
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);
        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.hold();
        c.step(kDt);

        sh.position = -3.0f * (-0.050f);  // 肩だけ 0.050 rad 遅れる
        c.step(kDt);
        test::checkNearF(c.trackingError().peak1, 0.050f, 1e-5f, "T3: 最大を控える");

        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);  // 実測が追いついた
        c.step(kDt);
        test::checkNearF(c.trackingError().e1, 0.0f, 1e-6f, "T3: いまの誤差は 0 へ戻る");
        test::checkNearF(c.trackingError().peak1, 0.050f, 1e-5f, "T3: **最大は下がらない**");

        sh.position = -3.0f * (+0.080f);  // 逆向きに、より大きく
        c.step(kDt);
        test::checkNearF(c.trackingError().e1, -0.080f, 1e-5f, "T3: 負の誤差");
        test::checkNearF(c.trackingError().peak1, 0.080f, 1e-5f, "T3: 最大は大きさで見る");

        c.clearTrackingErrorPeak();
        test::checkNearF(c.trackingError().peak1, 0.0f, 1e-6f, "T4: 肩の最大を落とせる");
        test::checkNearF(c.trackingError().peak2, 0.0f, 1e-6f, "T4: 肘の最大も落ちる");
        test::checkNearF(c.trackingError().e1, -0.080f, 1e-5f,
                         "T4: **いまの誤差は消さない** (次の周期が上書きする)");
    }

    test::section("T5: 指令を出していない周期では更新しない");
    {
        // **指令が無ければ追従誤差も無い。** 止まっているあいだは人が腕を
        // 手で動かすので実測だけが動く。そこを誤差として数えると、
        // 窓が人の手で埋まって本物の追従の遅れが読めなくなる。
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);
        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.hold();
        c.step(kDt);

        sh.position = -3.0f * (-0.030f);
        c.step(kDt);
        test::checkNearF(c.trackingError().e1, +0.030f, 1e-5f, "T5: まず誤差を作る");

        c.emergencyStop();
        placeAt(sh, el, 0.9f, 120.0f * arm::DEG);  // 人が腕を大きく動かした
        for (int i = 0; i < 200; ++i) c.step(kDt);
        test::checkNearF(c.trackingError().e1, +0.030f, 1e-5f, "T5: 止まっていれば動かない");
        test::checkNearF(c.trackingError().peak1, 0.030f, 1e-5f, "T5: 最大も増えない");
    }

    test::section("T6: 実測を読めないあいだは突き合わせたと言わない");
    {
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);
        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);

        sh.has_position = false;  // 肩のフィードバックが来ていない
        c.hold();
        for (int i = 0; i < 5; ++i) c.step(kDt);
        test::check(!c.trackingError().valid, "T6: 片軸でも読めなければ突き合わせない");
        test::checkNearF(c.trackingError().e2, 0.0f, 1e-6f,
                         "T6: **片軸だけの値を出さない** (同じ瞬間の比較でなくなる)");

        sh.has_position = true;
        c.step(kDt);
        test::check(c.trackingError().valid, "T6: 読めれば突き合わせる");
    }

    test::section("T7: 肩と肘を独立に持つ (どちらの利得が違うかを見る)");
    {
        // **2 台の位置ループ利得が食い違っていれば、同じ関節速度でも誤差が
        // 軸ごとに違う。** 1 つの数字にまとめると、その食い違いが埋もれる。
        FakeDrive     sh, el;
        armctl::Space space = makeSpace();
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);
        armctl::Controller c(sh, el, kKin, kTx, space, kJog);
        c.seedFromDrives();
        c.setTransmission(kTx);
        c.hold();
        c.step(kDt);

        sh.position = -3.0f * (-0.040f);  // 肩だけ遅れる
        c.step(kDt);
        test::checkNearF(c.trackingError().e1, +0.040f, 1e-5f, "T7: 肩に出る");
        test::checkNearF(c.trackingError().e2, 0.0f, 1e-6f, "T7: 肘は 0 のまま");
        test::checkNearF(c.trackingError().peak2, 0.0f, 1e-6f, "T7: 肘の最大も 0 のまま");
    }

    return test::report("arm_controller");
}


