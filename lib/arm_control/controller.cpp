#include "arm_control/controller.hpp"

namespace armctl {
namespace {

namespace kin = arm::kin;
namespace tx  = arm::tx;

// 経路検査の最小刻み。目標が近ければ端点だけで済ませない。
constexpr int min_path_samples = 2;

float clampSym(float v, float limit) { return arm::clampf(v, -limit, limit); }

// 1 軸ぶんの加速度制限つき追従。関節ジョグ用。
// 手先ジョグと違って**向きを保つ必要がない** — 関節どうしは独立に動かす。
float rampTo(float current, float target, float a_max, float dt) {
    const float step = a_max * dt;
    return current + arm::clampf(target - current, -step, step);
}

// 肘角が許容範囲からどれだけ外れているか [rad]。範囲内なら 0。
//
// **「良くなる向きなら通す」判定のための物差し。** 次の姿勢が合法かどうかだけを
// 見ると、いま範囲外に居るときは改善する向きまで拒否され、そこから戻れなくなる。
float elbowViolation(const kin::Config& cfg, arm::JointAngles q) {
    const float s  = (cfg.elbow_side == kin::ElbowSide::Positive) ? 1.0f : -1.0f;
    const float se = s * kin::elbow_angle(q);
    if (se < cfg.elbow_min) return cfg.elbow_min - se;
    if (se > cfg.elbow_max) return se - cfg.elbow_max;
    return 0.0f;
}

// 2 つの姿勢がどちらも組み立てどおりの側に居るか。
bool onAssembledSide(const kin::Config& cfg, arm::JointAngles a, arm::JointAngles b) {
    const bool want = (cfg.elbow_side == kin::ElbowSide::Positive);
    return ((kin::elbow_angle(a) > 0.0f) == want) && ((kin::elbow_angle(b) > 0.0f) == want);
}

}  // namespace

const char* faultText(Fault f) {
    switch (f) {
        case Fault::None: return "なし";
        case Fault::NotSeeded: return "実測姿勢で初期化していない";
        case Fault::OutOfWorkspace: return "生成点が作業領域の外";
        case Fault::Unreachable: return "生成点に手が届かない";
        case Fault::BadPose: return "その姿勢は機構として作れない";
        case Fault::Transmission: return "モータ角に落とせない";
        case Fault::DriveRejected: return "モータが指令を受け付けない";
        case Fault::NotEnergized: return "指令は通るがトルクが出ていない";
        case Fault::Halted: return "異常停止・非常停止の最中 (`c` で復帰)";
        case Fault::OverTorque: return "出力トルクが閾値を超えた (何かに当たっている)";
    }
    return "不明";
}

const char* modeText(Mode m) {
    switch (m) {
        case Mode::Idle: return "停止";
        case Mode::Hold: return "保持";
        case Mode::Move: return "直線移動";
        case Mode::ToolJog: return "手先ジョグ";
        case Mode::JointJog: return "関節ジョグ";
    }
    return "不明";
}

Controller::Controller(JointDrive& shoulder, JointDrive& elbow, const kin::Config& kin_cfg,
                       const tx::Config& tx_cfg, const Space& space, const JogLimits& jog)
    : shoulder_(shoulder), elbow_(elbow), kin_(kin_cfg), tx_(tx_cfg), space_(space), jog_(jog) {
    tool_jog_.set_limits(jog_.tool_v_max, jog_.tool_a_max);
}

// ---------------------------------------------------------------------------
// 立ち上げ
// ---------------------------------------------------------------------------

bool Controller::seedFromDrives() {
    float m1 = 0.0f;
    float m2 = 0.0f;
    if (!shoulder_.readPosition(m1) || !elbow_.readPosition(m2)) return false;

    const arm::JointAngles measured = tx::to_joint(tx_, tx::MotorAngles{m1, m2});
    if (!arm::is_finite(measured)) return false;

    cont_.seed(measured);
    joint_target_ = measured;
    return true;
}

arm::Point Controller::tip() const { return kin::forward(kin_, cont_.value()); }

kin::Status Controller::poseStatus() const {
    if (!isSeeded()) return kin::Status::NotFinite;
    return kin::check_joints(kin_, cont_.value());
}

void Controller::emergencyStop() {
    estop_ = true;
    mode_  = Mode::Idle;  // **保持を止める。** 送り続けると CAN タイムアウトを撫でる
    move_.abort();
    released_ = false;  // 成立したかは releaseNow() が決める
    releaseNow();
}

// **オフセットを入れることが原点の宣言そのもの。** 別に「原点を入れた」と
// 立てる口を置くと、呼び忘れても誰も気づけない。
bool Controller::setTransmission(const arm::tx::Config& cfg) {
    // **検査を通らない設定は 1 バイトも書かない。** 半端に書き換えると、
    // 次の指令が壊れた係数でモータ角へ落ちる。
    if (arm::tx::validate(cfg) != arm::tx::Status::Ok) return false;
    tx_    = cfg;
    homed_ = true;
    return true;
}

bool Controller::clearFault() {
    // **止まっていないなら解除するものが無い。** ここで断ると、腕は保持して
    // いるのに操作者は「まだ脱力している」と読む — 食い違いが安全でない向きに
    // 出る。何もせずに成功を返すほうが、表示と腕と文言が揃う。
    if (!isHalted()) return true;
    // **原点が無いままトルクを出させない。** 姿勢表示も作業領域の制限も肘角の
    // 限界も原点オフセットの上に乗っているので、入れずに通電すると嘘の座標系で
    // 守ることになる。ここはトルクへ至る唯一の門 — 使能は位置モードへ入れる
    // ことでしか起きず、その手前に必ずこの関数がある。
    if (!homed_) return false;
    // **トルクが出たままの軸を抱えたまま復帰させない。**
    if (!released_) return false;

    // **アクチュエータ側にラッチした故障も解除する。** これを送らないと、
    // 使能し直しても同じ故障で落ちる。**両方に送る** — 片方が失敗しても
    // もう片方は解除しておく (残す理由がない)。
    const bool a = shoulder_.clearFaults();
    const bool b = elbow_.clearFaults();
    if (!a || !b) return false;  // 送れていない。異常は消さない

    fault_            = Fault::None;
    estop_            = false;
    mode_             = Mode::Idle;
    energized_misses_ = 0;
    // **停止を下ろしたあとで下ろす。** 先に下ろすと、2 つの書き込みの隙に
    // 割り込みが step() を回して「止まっているのに脱力が成立していない」と見え、
    // 脱力を送り直して真へ戻す — 復帰したのに脱力中と表示する状態が組み上がる。
    released_         = false;
    return true;
}

// ---------------------------------------------------------------------------
// 指令
// ---------------------------------------------------------------------------

Fault Controller::checkPath(arm::Point from, arm::Point to, int samples) const {
    if (!arm::is_finite(from) || !arm::is_finite(to)) return Fault::Unreachable;

    const int n = (samples < min_path_samples) ? min_path_samples : samples;
    for (int i = 0; i <= n; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(n);
        const arm::Point p{from.x + (to.x - from.x) * t, from.y + (to.y - from.y) * t};

        if (kin::check_reachable(kin_, p) != kin::Status::Ok) return Fault::Unreachable;
        if (!space_.contains(p)) return Fault::OutOfWorkspace;
    }
    return Fault::None;
}

Fault Controller::startMove(arm::Point to, const arm::motion::TrapezoidProfile& profile) {
    if (!isSeeded()) return Fault::NotSeeded;
    if (isHalted()) return fault_;
    // **いま居る姿勢そのものを検査する。** checkPath が見るのは点の距離だけで、
    // 肘の曲がる向きを見ない。逆側の姿勢から始めると kin::inverse() は必ず
    // 組み立て側の解を返すので、繋ぐ先が鏡像になり 1 周期で肩が 270 度飛ぶ。
    if (poseStatus() != kin::Status::Ok) return Fault::BadPose;

    const arm::Point from = tip();
    const Fault      bad  = checkPath(from, to);
    if (bad != Fault::None) return bad;  // **動かさない。** 走り出してから止めない

    if (!move_.start(from, to, profile)) return Fault::Unreachable;
    mode_ = Mode::Move;
    return Fault::None;
}

Fault Controller::startToolJog() {
    if (!isSeeded()) return Fault::NotSeeded;
    if (isHalted()) return Fault::Halted;
    // 手先ジョグは運動学を通るので、姿勢が逆側だと同じ跳びが出る。
    // **作業領域の外に居ることは拒まない** — そこから戻るのに使うため。
    // 領域は設置環境の話、姿勢は機構の話で、後者だけがここで効く。
    if (poseStatus() != kin::Status::Ok) return Fault::BadPose;
    tool_jog_.set_limits(jog_.tool_v_max, jog_.tool_a_max);
    tool_last_ = tip();
    tool_jog_.set_position(tool_last_);  // 現在位置から。跳ねさせない
    mode_ = Mode::ToolJog;
    return Fault::None;
}

Fault Controller::startJointJog() {
    if (!isSeeded()) return Fault::NotSeeded;
    if (isHalted()) return Fault::Halted;
    joint_target_ = cont_.value();
    joint_cmd_    = arm::JointVels{};
    joint_vel_    = arm::JointVels{};
    mode_         = Mode::JointJog;
    return Fault::None;
}

void Controller::hold() {
    if (isHalted()) return;
    mode_ = isSeeded() ? Mode::Hold : Mode::Idle;
}

void Controller::setToolVelocity(arm::Velocity v) { tool_jog_.set_velocity(v); }

void Controller::setJointVelocity(arm::JointVels dq) {
    if (!arm::is_finite(dq)) return;  // 壊れた指令は無視。前の指令が残るほうが安全
    // 関節ごとに独立にクランプする。手先ジョグと違って向きの意味がない。
    joint_cmd_ = arm::JointVels{clampSym(dq.dq1, jog_.joint_v_max),
                                clampSym(dq.dq2, jog_.joint_v_max)};
}

// ---------------------------------------------------------------------------
// 制御周期
// ---------------------------------------------------------------------------

void Controller::releaseNow() {
    // **両方に送る。片方が失敗しても止めない。**
    const bool a = shoulder_.release();
    const bool b = elbow_.release();
    released_    = a && b;
}

Fault Controller::trip(Fault f) {
    fault_ = f;
    mode_  = Mode::Idle;
    move_.abort();
    releaseNow();
    return f;
}

// 指令と実測のずれを控える。**計測だけ。制御は一切これを見ない。**
//
// 読むのは既に届いている型 2 フィードバックなので、**この関数が増やす
// フレームは 1 本も無い。** 定義・符号・1 周期の遅れは TrackingError の宣言
// (controller.hpp) に書いてある。
void Controller::noteTrackingError(arm::JointAngles commanded) {
    float m1 = 0.0f;
    float m2 = 0.0f;
    // **片軸でも読めなければ何も変えない。** 2 軸を並べて比べるための数字なので、
    // 片方だけ新しくすると同じ瞬間の比較でなくなり、軸ごとの差が読めなくなる。
    if (!shoulder_.readPosition(m1) || !elbow_.readPosition(m2)) return;

    const arm::JointAngles measured = tx::to_joint(tx_, tx::MotorAngles{m1, m2});
    const float            e1       = commanded.theta1 - measured.theta1;
    const float            e2       = commanded.theta2 - measured.theta2;
    // **壊れた値で最大を汚さない。** 最大は latch するので、一度 NaN を入れると
    // 比較がすべて偽になり、それ以降どの周期も最大を更新できなくなる。
    if (!arm::is_finite(e1) || !arm::is_finite(e2)) return;

    tracking_.valid = true;
    tracking_.e1    = e1;
    tracking_.e2    = e2;

    // 最大は大きさで持つ。**どちらへ遅れているかは向きの話で、窓の話ではない。**
    const float a1 = e1 < 0.0f ? -e1 : e1;
    const float a2 = e2 < 0.0f ? -e2 : e2;
    if (a1 > tracking_.peak1) tracking_.peak1 = a1;
    if (a2 > tracking_.peak2) tracking_.peak2 = a2;
}

Fault Controller::commandJoints(arm::JointAngles q) {
    // **指令を出す周期にだけ測る。** ここが指令の唯一の通り道なので、
    // 置く場所もここしかない。送信の成否より前に測るのは、届かなかった周期も
    // 「どれだけ離れていたか」は本当だから。
    noteTrackingError(q);

    tx::MotorAngles m{};
    if (tx::to_motor(tx_, q, m) != tx::Status::Ok) return trip(Fault::Transmission);
    if (!shoulder_.commandPosition(m.m1)) return trip(Fault::DriveRejected);
    if (!elbow_.commandPosition(m.m2)) return trip(Fault::DriveRejected);
    return Fault::None;
}

Fault Controller::stepCartesian(arm::Point target) {
    if (!space_.contains(target)) return trip(Fault::OutOfWorkspace);

    arm::JointAngles q{};
    if (kin::inverse(kin_, target, q) != kin::Status::Ok) return trip(Fault::Unreachable);

    // **連続化を挟む。** ここを抜くと ±π の折り返しでモータ角が 6π 飛ぶ。
    return commandJoints(cont_.track(q));
}

Fault Controller::stepToolJog(float dt) {
    const arm::Point next = tool_jog_.update(dt);

    arm::JointAngles q{};
    const bool       reachable = (kin::inverse(kin_, next, q) == kin::Status::Ok);
    const bool       allowed   = reachable && acceptToolPoint(next);

    if (!allowed) {
        // **境界に当たっただけで脱力しない。** ここは操作者が運転している。
        // 直前の通った点まで戻して止める。指令も落とすので、続けるには
        // もう一度こちらから velocity を送る必要がある — 押しっぱなしで
        // 境界に貼り付いたままにしない。
        tool_jog_.set_position(tool_last_);
        // **拒否した周期も保持を送る。** 電源投入時の姿勢は選べないので、
        // 手先が領域外にあるとこの経路が毎周期通る。1 本も送らないと
        // モータ側の CAN タイムアウト (≈1 秒) が切れて自分で脱力する。
        return commandJoints(cont_.value());
    }

    tool_last_ = next;
    return commandJoints(cont_.track(q));
}

// 手先ジョグの行き先を許すか。**領域の外なら、戻る向きだけ通す。**
bool Controller::acceptToolPoint(arm::Point next) const {
    const float d = space_.distance_to_boundary(next);
    if (d >= 0.0f) return true;                            // 中に居る
    return d > space_.distance_to_boundary(tool_last_);     // 外だが、近づいている
}

// 関節ジョグの行き先を許すか。
//
// **次の姿勢が合法かどうかだけを見てはいけない。** いま範囲外に居ると、
// 微小な 1 歩先も範囲外なので改善する向きまで拒否され、関節ジョグ
// (原点合わせの唯一の手段) が固まって戻れなくなる。
bool Controller::acceptJointStep(arm::JointAngles next) const {
    const arm::JointAngles now = cont_.value();

    // 作業領域。校正中に柱へ当てない。外に居るなら戻る向きだけ通す。
    const float d = space_.distance_to_boundary(kin::forward(kin_, next));
    if (d < 0.0f && d <= space_.distance_to_boundary(kin::forward(kin_, now))) return false;

    if (kin::check_joints(kin_, next) == kin::Status::Ok) return true;

    // **逆側はジョグでは直せない。** どちらへ回っても死点を通るので、
    // 「良くなる向き」が存在しない。これは校正 (`h`) の問題であって
    // 姿勢の問題ではない。poseStatus() で人へ見せる。
    if (!onAssembledSide(kin_, now, next)) return false;

    return elbowViolation(kin_, next) <= elbowViolation(kin_, now);
}

Fault Controller::stepJointJog(float dt) {
    joint_vel_.dq1 = rampTo(joint_vel_.dq1, joint_cmd_.dq1, jog_.joint_a_max, dt);
    joint_vel_.dq2 = rampTo(joint_vel_.dq2, joint_cmd_.dq2, jog_.joint_a_max, dt);

    const arm::JointAngles next{joint_target_.theta1 + joint_vel_.dq1 * dt,
                                joint_target_.theta2 + joint_vel_.dq2 * dt};

    // **運動学を通さないので、姿勢の検査は自分でやる。**
    // 肘角の範囲と曲がる向きは機構の限界で、校正中でも譲れない。
    if (!acceptJointStep(next)) {
        joint_vel_ = arm::JointVels{};
        // **拒否した周期も直前の合法姿勢を送る。** ここを無送信にすると、
        // 限界へ押し付けているあいだモータ側の CAN タイムアウトが切れ、
        // 人が腕の近くでジョグを押している最中に両軸が自分で脱力する。
        return commandJoints(joint_target_);
    }

    joint_target_ = next;
    cont_.seed(next);  // 連続値をそのまま基準にする (この経路は既に連続)
    return commandJoints(next);
}

// **脱力しているあいだも応答を引き出し続ける。**
//
// RobStride の能動報告は既定で切なので、こちらから送らないと型 2 フィードバックは
// 来ない。何も送らないと姿勢もトルクも凍り、画面は最後に見た値を出したまま
// 「応答なし」になる。**人が腕を手で動かしているのはまさにこのとき**で、
// そこで表示が死んでいると、肘角も原点も読めない。
//
// 送るのは脱力そのもの。効果が「トルクを出さない状態にする」なので、
// 既に脱力していれば何も変わらない。制御周期で送る必要はないので間引く。
//
// **引き出すだけでは足りない。取り込んで初めて外から見える。**
// joints() / tip() が返すのは cont_ の値、つまり**指令側の状態**であって
// 実測ではない。応答が届いても cont_ は動かないので、表示は凍ったままになる。
// 脱力中は指令を出していないのだから、指令側を実測へ追従させるのが正しい —
// そうしておくと `c` で復帰したときも、いま居る場所から続けられる。
void Controller::pollWhileReleased(float dt) {
    idle_poll_s_ += dt;
    if (idle_poll_s_ < idle_poll_period_s) return;
    idle_poll_s_ = 0.0f;
    shoulder_.release();
    elbow_.release();
    // 読めなければ何も変えない (seedFromDrives が自分で判断する)。
    (void)seedFromDrives();
}

// 出力トルクを見る。**閾値未設定なら見るだけで倒さない。**
Fault Controller::checkTorque() {
    float t1 = 0.0f;
    float t2 = 0.0f;
    const bool have = shoulder_.readTorque(t1) && elbow_.readTorque(t2);
    if (!have) return Fault::None;  // まだ受け取っていない。倒す根拠が無い

    const float a1   = t1 < 0.0f ? -t1 : t1;
    const float a2   = t2 < 0.0f ? -t2 : t2;
    const float peak = a1 > a2 ? a1 : a2;
    if (peak > torque_peak_nm_) torque_peak_nm_ = peak;

    if (!isTorqueLimitArmed()) return Fault::None;
    if (peak <= torque_limit_nm_) {
        over_torque_hits_ = 0;
        return Fault::None;
    }
    if (++over_torque_hits_ <= over_torque_grace_cycles) return Fault::None;
    return trip(Fault::OverTorque);
}

Fault Controller::step(float dt) {
    if (fault_ != Fault::None || estop_) {
        // **成功するまで送り続ける。** 1 回の送信失敗で非常停止が成立しないのは
        // 一番あってはならない。バスが戻れば次の周期で脱力が届く。
        // 操作者が押した停止 (estop_) も同じ経路を通る — `x` を 1 回試して
        // 「脱力しました」と言い切ると、届いていないときに人が安全だと信じる。
        if (!released_) {
            releaseNow();
        } else {
            pollWhileReleased(dt);  // 成立したあとも表示を生かす
        }
        return fault_;
    }
    if (!(dt > 0.0f)) return Fault::None;
    if (!isSeeded()) return trip(Fault::NotSeeded);

    // **指令を出すモードのときだけ通電を要求する。**
    // Idle は脱力しているのが正しい状態なので、ここで要求すると止めた瞬間に
    // 異常が出る。モータが自分で保護動作に入って脱力しても通信には応答し続ける
    // ので、指令の戻り値では気づけない — 別に問う必要がある。
    if (mode_ != Mode::Idle) {
        if (shoulder_.isEnergized() && elbow_.isEnergized()) {
            energized_misses_ = 0;
        } else if (++energized_misses_ > energized_grace_cycles) {
            return trip(Fault::NotEnergized);
        }
    } else {
        energized_misses_ = 0;
    }

    // **指令を出すモードのときだけ過トルクを見る。** Idle は脱力しているので
    // トルクは出ておらず、見ても意味がない。
    if (mode_ != Mode::Idle) {
        const Fault t = checkTorque();
        if (t != Fault::None) return t;
    } else {
        over_torque_hits_ = 0;
    }

    switch (mode_) {
        case Mode::Idle:
            // **1 本も送らない。** 止まっていない Idle は復帰から保持までの
            // 過渡状態にしかならず、そこは呼ぶ側がモータを使能している最中に
            // あたる。脱力のフレームが使能の後に届くとモータは使能を取り消し、
            // 保持へ入った直後に「通電していない」で倒れる。
            // 人が腕を手で動かすのは止まっているあいだで、そちらの表示は上の
            // 停止の分岐が巡回して生かしている。
            return Fault::None;

        case Mode::Hold:
            // **送信を止めない。** 途絶えるとモータ側の CAN タイムアウトで
            // 保持が解ける。同じ姿勢を送り続けるだけなので運動学は要らない。
            return commandJoints(cont_.value());

        case Mode::Move: {
            const arm::Point target = move_.update(dt);
            const Fault      f      = stepCartesian(target);
            if (f == Fault::None && move_.is_done()) mode_ = Mode::Hold;
            return f;
        }

        case Mode::ToolJog:
            return stepToolJog(dt);

        case Mode::JointJog:
            return stepJointJog(dt);
    }
    return Fault::None;
}

}  // namespace armctl

