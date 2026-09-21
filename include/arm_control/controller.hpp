#pragma once
// 4 層を 1 本につないだ制御ステップと、その状態機械。
//
// ============================================================================
// **ハードウェアに触れない。だからテストできる。**
// ============================================================================
//
// ここが実機で動く制御ループそのもので、`main.cpp` はこれを呼ぶだけ。
// ホスト側テストは偽の JointDrive を挿して同じコードを回す。
//
// 以前は制御ループが main.cpp の割り込みハンドラの中にあり、テストは
// **同じ手順をテスト側で書き写したもの**を検証していた。写しが通っても
// 出荷されるコードが通ったことにはならない。
//
// ============================================================================
// 実行コンテキスト
// ============================================================================
//
//   main のみ (待ちを含む / 大きい):
//     seedFromDrives / checkPath / startMove / startToolJog / startJointJog / hold /
//     clearFault
//   割り込みから呼んでよい:
//     step / emergencyStop / setToolVelocity / setJointVelocity /
//     各種の読み出し (mode / fault / isHalted / isSeeded / isReleased など)
//
// ジョグの指令 (main が書き、割り込みが読む) はロックで守っていない。
// **2 つの float の読み出しが割り込みに割られると、新旧が混ざった向きが 1 周期だけ
// 出る。** それでも大きさは必ず上限以下になる — 混ざる前のどちらも上限以下で、
// 受け取った側がもう一度クランプするため。1 周期 (2 ms) で正しい値に戻る。
// ここにロックを置くと、この層が pico-sdk の臨界区間に依存してテストできなくなる。

#include "arm_control/joint_drive.hpp"

#include "arm/continuity.hpp"
#include "arm/kinematics.hpp"
#include "arm/motion.hpp"
#include "arm/transmission.hpp"
#include "arm/workspace.hpp"

namespace armctl {

// 作業領域の容量。設置環境の形状はそう多くない。
inline constexpr std::size_t max_shapes = 8;
using Space                             = arm::ws::Workspace<max_shapes>;

enum class Fault : uint8_t {
    None = 0,
    NotSeeded,       // 実測姿勢で初期化していない。どこに居るか知らないまま動かせない
    OutOfWorkspace,  // 生成点が作業領域の外
    Unreachable,     // 逆運動学が解けない (伸びきり / 折り畳みすぎ)
    BadPose,         // 関節ジョグの姿勢が機構として成立しない
    Transmission,    // モータ角に落とせない (設定が壊れている)
    DriveRejected,   // ドライブが指令を受け付けない
    NotEnergized,    // 指令は通るが、トルクが出ていない (モータが自分で脱力した)
    OverTorque,      // 出力トルクが閾値を超えた。何かに当たっている
    // 異常停止か非常停止の最中。**断った理由であって、新しい異常ではない。**
    // これを fault() が返すことはない — 受け付けなかったことを呼び出し側へ返すだけ。
    Halted,
};

// **通電していないと判断するまでの連続周期数。** 500 Hz で 100 周期 = 200 ms。
//
// 1 周期の欠落で異常にしない。フィードバックは指令への応答として届くので、
// 送信が 1 回詰まれば 1 周期ぶん古くなる。逆に長く待つと、脱力したまま
// 指令を送り続ける時間が伸びる。
inline constexpr int energized_grace_cycles = 100;

// **過トルクとみなすまでの連続周期数。** 500 Hz で 10 周期 = 20 ms。
//
// 1 周期の跳ねで倒さない。加速の立ち上がりや指令の段差で瞬間的に超えることは
// あり、そこで毎回脱力すると使えない。逆に長く待つと、押し付けている時間が
// そのまま伸びる — 人の手が挟まっている場面がこれにあたる。
// **通電の猶予 (200 ms) より 1 桁短いのは、こちらが「当たっている」の検出で、
// 待つほど害が増えるため。** 閾値を実測するとき一緒に見直す。
inline constexpr int over_torque_grace_cycles = 10;

// 脱力しているあいだ、応答を引き出すために脱力を送り直す間隔 [s]。
// **isOnline / isEnergized の既定タイムアウト (200 ms) より十分短いこと** —
// 長いと「応答なし」と「本当に応答が無い」が区別できなくなる。
inline constexpr float idle_poll_period_s = 0.05f;  // 20 Hz

enum class Mode : uint8_t {
    Idle = 0,  // 何も送らない
    Hold,      // 直近の姿勢を送り続ける
    Move,      // 直線補間
    ToolJog,   // 手先ジョグ (4 層を通る)
    JointJog,  // 関節ジョグ (**運動学を通さない**。原点合わせ用)
};

struct JogLimits {
    float tool_v_max  = 60.0f;   // mm/s
    float tool_a_max  = 300.0f;  // mm/s^2
    float joint_v_max = 0.3f;    // rad/s
    float joint_a_max = 1.5f;    // rad/s^2
    // 差動ねじ。**Controller は使わない** — ペンダントが翻訳に使うだけで、
    // 手首の制御は別の口 (wrist.hpp) が持つ。ここに置くのは、操作の上限が
    // 1 か所にまとまっているほうが読めるから。
    float lift_mm_s  = 20.0f;    // mm/s
    // 回転は歩進なので JogLimits には無い — 1 歩の角は wrist.hpp が持つ。
};

// 指令と実測のずれ。**計測のためだけに持つ値で、制御は一切これを見ない。**
//
// がたつきの原因が手先ジョグ側 (躍度の制限が無い) かモータ側 (位置ループ利得が
// 2 台で食い違っている) かは観察では区別できない。区別できるのは「追従誤差が
// 姿勢と向きでどう変わるか」だけなので、そのための窓をここに開ける。
//
// **定義:** `誤差 = 指令 − 実測` [rad]、**関節空間** (伝達比を通したあと)。
//   正 = 実測が指令に追いついていない (遅れている)
//   負 = 実測が指令を追い越している
// 関節空間で持つのは、減速比の違う 2 軸を同じ物差しで並べるため。位置ループ
// 利得の食い違いを見るときも `誤差 ÷ 関節速度 = 1/利得` で減速比が約分される
// ので、モータ角へ直さなくても 2 台を比べられる。
//
// **更新は指令を出した周期だけ。** commandJoints() の入口で、いま送ろうと
// している指令と、その時点で手元にある最新の実測を突き合わせる。指令を出して
// いない周期 (停止中・Idle) では更新しない — 指令が無ければ追従誤差も無いし、
// そこは人が腕を手で動かしている時間なので、数えると窓が人の手で埋まる。
//
// **実測は少なくとも 1 周期ぶん古い。** フィードバックは指令への応答としてしか
// 来ないので、周期 N で手元にある最新は周期 N−1 の指令に対する答えでしかない。
// そのぶんが関節速度に比例した見かけの誤差 (v·dt) として必ず乗る — 500 Hz で
// 関節 0.3 rad/s なら 0.0006 rad。位置ループ利得 40 の追従誤差 v/40 = 0.0075 rad
// の 1 割弱なので 2 軸の比較は壊さないが、**誤差が厳密に 0 になるのは腕が
// 止まっているときだけ**である。
struct TrackingError {
    bool  valid = false;  // 一度でも実測と突き合わせたか。偽なら下は全部 0
    float e1    = 0.0f;   // 肩 [rad] = 指令 − 実測
    float e2    = 0.0f;   // 肘 [rad] = 指令 − 実測
    float peak1 = 0.0f;   // 起動以来の |e1| の最大
    float peak2 = 0.0f;   // 起動以来の |e2| の最大
};

class Controller {
public:
    Controller(JointDrive& shoulder, JointDrive& elbow, const arm::kin::Config& kin,
               const arm::tx::Config& tx, const Space& space, const JogLimits& jog);

    Controller(const Controller&)            = delete;
    Controller& operator=(const Controller&) = delete;

    // --- 立ち上げ ------------------------------------------------------------

    // 両軸の実測位置で連続化を初期化する。**これを通らないと step が動かない。**
    // アームは電源投入時どこかに居るので、決め打ちの姿勢から始めると跳ねる。
    bool seedFromDrives();
    bool isSeeded() const { return cont_.has_reference(); }

    // 校正で原点オフセットを入れ替える。**オフセットを入れることが原点の宣言
    // そのもの** なので、ここが原点合わせ済みの印も立てる。別に markHomed() を
    // 置くと、呼び忘れても誰も気づけない口ができる。
    // **妥当でない設定は入れず、原点も立てない。** 半端に書き換えると、
    // 次の指令が壊れた係数でモータ角へ落ちる。
    bool setTransmission(const arm::tx::Config& tx);

    // 原点合わせ済みか。**同じ起動の中で偽へ戻らない。**
    // ヘッダの中に置くのは、pendant の ledFor が controller.cpp をリンクしない
    // テストから呼ぶため。
    bool isHomed() const { return homed_; }
    const arm::tx::Config& transmission() const { return tx_; }

    arm::JointAngles joints() const { return cont_.value(); }
    arm::Point       tip() const;

    // いまの姿勢が機構として成立しているか。**校正前は当てにならない。**
    // 原点オフセットが 0 のあいだ肘角の符号は事実上任意で、逆側と出ることがある。
    // 逆側はジョグでは直せない (どちらへ回っても死点を通る) ので、
    // 校正 (`h`) の問題として人へ見せるための口。
    arm::kin::Status poseStatus() const;

    // --- 過トルクで脱力する ---------------------------------------------------
    //
    // **0 以下なら無効。** 閾値は実機で通常運転のトルクを測ってから決める値で、
    // 知らないまま置くと「常時トリップする」か「まったく効かない」のどちらかに
    // なる。一次資料も既定値も無いので、測るまでは無効のまま出す。
    //
    // **無効は fail-open** であることを承知しておくこと。起動時に有効か無効かを
    // 人へ見せる (main.cpp) のはそのため。
    void  setTorqueLimit(float nm) { torque_limit_nm_ = nm; }
    float torqueLimit() const { return torque_limit_nm_; }
    bool  isTorqueLimitArmed() const { return torque_limit_nm_ > 0.0f; }

    // 直近に見た両軸のトルクの大きさの最大 [N*m]。**閾値を測るための窓。**
    // 一度も読めていなければ負を返す。
    float peakTorque() const { return torque_peak_nm_; }
    void  clearPeakTorque() { torque_peak_nm_ = -1.0f; }

    // --- 追従誤差 (指令と実測のずれ) -----------------------------------------
    //
    // **計測だけ。何も止めず、何も変えない。** 定義と符号と更新の時点は
    // TrackingError (この宣言の上) に書いてある。
    TrackingError trackingError() const { return tracking_; }

    // 最大だけを落とす。**口を必ず置く。** latch したまま下げる手段が無いと、
    // 立ち上げ時の 1 回の跳ねが窓を埋め、それ以降の計測が読めなくなる。
    // いまの誤差は消さない — 次に指令を出す周期が上書きする値なので、
    // ここで 0 にすると「測っていない」と「ちょうど 0」が区別できなくなる。
    void clearTrackingErrorPeak() {
        tracking_.peak1 = 0.0f;
        tracking_.peak2 = 0.0f;
    }

    // --- 指令 (main のみ) ----------------------------------------------------

    // **動く前に経路を通しで検査する。** 始点と終点だけ見ていると、
    // その間が禁止領域や到達可能円環を横切っていても動き出してしまい、
    // 中途半端な姿勢で止まることになる。
    Fault checkPath(arm::Point from, arm::Point to, int samples = 64) const;

    // 経路が通らなければ動かず、その理由を返す。
    Fault startMove(arm::Point to, const arm::motion::TrapezoidProfile& profile);

    // **断るなら理由を返す。** void で黙って戻ると、押した人には
    // 「何も起きない」しか残らない。Fault::None が「受け付けた」。
    //
    // 手先ジョグは運動学を通るので姿勢が逆側だと断る (BadPose)。
    // 関節ジョグは**姿勢では断らない** — そこから戻る唯一の手段なので。
    Fault startToolJog();
    Fault startJointJog();
    void hold();

    // --- ジョグの指令 (割り込みから呼んでよい) -------------------------------

    void setToolVelocity(arm::Velocity v);
    void setJointVelocity(arm::JointVels dq);

    // --- 制御周期 ------------------------------------------------------------

    // 1 周期ぶん。異常なら脱力し、**成功するまで毎周期やり直す。**
    Fault step(float dt);

    // --- 非常停止と復帰 ------------------------------------------------------

    // 操作者による脱力。異常ではないが、**成立するまで step() がやり直す。**
    // 送信が落ちれば isReleased() は false のままなので、届いていないことが分かる。
    // 保持の指令は止める — 送り続けるとモータ側の CAN タイムアウトを撫でてしまい、
    // 脱力が届かなかったときの最後の砦を自分で無効にする。
    //
    // **起動時の既定状態を作る口でもある。** 制御タイマを始める前に 1 度呼ぶと、
    // 最初の周期から脱力の分岐に入る。要る振る舞い (止まっている・成立するまで
    // やり直す・成立後は巡回して表示を生かす・出口は clearFault) は非常停止と
    // 同じものなので、別の入口を足しても観測では区別できない。
    void emergencyStop();

    // 異常 / 非常停止からの復帰。**脱力が成立していなければ拒む。**
    // トルクが出たままの軸を抱えたまま復帰させない。
    // 成功すると Idle に戻るので、呼ぶ側は seed と hold をやり直す。
    //
    // **原点が無ければ拒む。** ここが「原点合わせを済ませるまでトルクを出さない」
    // を保証する唯一の場所。呼ぶ側の if で守ると、その if を消してもホストの
    // ゲートからは見えない。
    //
    // **止まっていなければ何もせず true。** 断ると、腕は保持しているのに
    // 操作者は「まだ脱力している」と読む — 食い違いが安全でない向きに出る。
    bool clearFault();

    // --- 状態 ----------------------------------------------------------------

    Mode  mode() const { return mode_; }
    Fault fault() const { return fault_; }

    // 異常か非常停止で止まっている。**どちらも新しい指令を受け付けない。**
    // 脱力が成立したかは問わない — 成立していないほうが危ないので、
    // 成立を待ってから「止まっている」と言うと危険な側が隠れる。
    bool isHalted() const { return fault_ != Fault::None || estop_; }

    // 停止の最中に、脱力の送信が両軸に受理されたか。**復帰すると偽に戻る。**
    bool  isReleased() const { return released_; }
    bool  isMoveDone() const { return move_.is_done(); }
    float progress() const { return move_.progress(); }

private:
    Fault trip(Fault f);
    void  releaseNow();
    Fault commandJoints(arm::JointAngles q);  // 伝達比 → ドライブ
    Fault stepCartesian(arm::Point target);   // 直線移動。外れたら異常
    Fault stepToolJog(float dt);              // ジョグ。外れたら止めるだけ
    Fault stepJointJog(float dt);

    // ジョグの行き先を許すか。**外に居るときは戻る向きだけ通す。**
    bool acceptToolPoint(arm::Point next) const;
    bool acceptJointStep(arm::JointAngles next) const;

    JointDrive&     shoulder_;
    JointDrive&     elbow_;
    arm::kin::Config kin_;
    arm::tx::Config  tx_;
    const Space&     space_;
    JogLimits        jog_;

    arm::JointContinuity    cont_;
    arm::motion::LinearMove move_;
    arm::motion::Jog        tool_jog_;
    arm::Point              tool_last_{};  // ジョグで最後に通った点。戻す先

    // 関節ジョグ。**運動学を通さないので、目標も関節角で持つ。**
    arm::JointAngles joint_target_{};
    arm::JointVels   joint_cmd_{};  // main が書き、割り込みが読む
    arm::JointVels   joint_vel_{};  // 加速度制限をかけた実際の速度

    Mode  mode_     = Mode::Idle;
    Fault fault_    = Fault::None;
    bool  released_ = false;
    // 原点合わせ済み。**書き手は main だけ、読み手は main と割り込み。**
    // 1 バイトなので割られても値が壊れない。偽へ戻るのは再起動だけ。
    bool  homed_    = false;
    // 通電していない周期が何回続いたか。**猶予を超えたら異常。**
    int energized_misses_ = 0;

    void  pollWhileReleased(float dt);
    Fault checkTorque();

    // 指令と実測を突き合わせて控える。**読むだけ。送信も判断も増やさない。**
    void noteTrackingError(arm::JointAngles commanded);

    float idle_poll_s_      = 0.0f;
    float torque_limit_nm_  = 0.0f;   // 0 以下 = 無効
    int   over_torque_hits_ = 0;
    float torque_peak_nm_   = -1.0f;  // 負 = 一度も読めていない

    TrackingError tracking_{};

    // 操作者が止めた。**異常ではない**ので fault_ とは分けるが、
    // 「脱力が成立するまで毎周期やり直す」振る舞いは同じ。
    bool estop_ = false;
};

const char* faultText(Fault f);
const char* modeText(Mode m);

}  // namespace armctl

