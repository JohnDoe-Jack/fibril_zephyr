// **使能の 1 本道を、段の順序ごと検査する。**
//
// M3 とコンソールの `c` は同じ手順を踏まなければならない。main.cpp に 2 写し
// 置くと、どちらかだけ直した変更がゲートを通る。順序そのものも検査の対象で、
// **通信途絶保護は位置モードへ入れるより前**でなければ、保護の無い状態で
// トルクが出ている窓ができる。
//
// 偽のドライブを挿すので CAN も pico-sdk も要らない。

#include "test_util.hpp"

#include "arm_control/enable.hpp"

#include <cstring>

namespace {

using arm::JointAngles;
namespace kin = arm::kin;
namespace tx  = arm::tx;
namespace ws  = arm::ws;

using armctl::EnableResult;

constexpr float kDt = 0.002f;

uint64_t code(EnableResult r) { return static_cast<uint64_t>(r); }
uint64_t code(armctl::Mode m) { return static_cast<uint64_t>(m); }
uint64_t count(int n) { return static_cast<uint64_t>(n); }

// **main を止める時間そのものを数える。** 回数だけでは、読み返しを積み直しの
// ループへ入れた実装 (1 軸あたり 20 x 51 ms) を「粘っただけ」と読んでしまう。
// 実機ではそれが「押すたびに基板が再起動する」という形で出る欠陥で、
// 時間を模型が請求しない限りホストでは表せない。
uint64_t sim_stall_us = 0;

// **軸をまたぐ段の前後は、片軸ずつのログでは表せない。** 「両軸へ書き終えて
// から両軸を読み返す」は 2 本のログを突き合わせないと見えないので、
// 大域に 1 本の印を置く。'S'/'E' が軸、小文字が段。
char g_seq[96] = {};
int  g_seq_len = 0;
void stampSeq(char who, char what) {
    if (g_seq_len + 2 < static_cast<int>(sizeof(g_seq))) {
        g_seq[g_seq_len++] = who;
        g_seq[g_seq_len++] = what;
        g_seq[g_seq_len]   = '\0';
    }
}

// --- 偽のドライブ -----------------------------------------------------------
// **呼ばれた順を覚える。** 何回呼ばれたかだけでは段の前後を検査できない。

class FakeDrive final : public armctl::JointDrive {
public:
    static constexpr int max_log = 32;

    bool init() override { return true; }

    bool enterPositionMode() override {
        log("mode");
        stampSeq(tag, 'm');
        ++mode_calls;
        if (mode_busy > 0) {
            --mode_busy;
            return false;  // 送信バッファが詰まっているだけ。次は通る
        }
        if (mode_ok) energized = true;  // 使能するとトルクが出る
        return mode_ok;
    }
    bool commandPosition(float rad) override {
        ++command_calls;
        last_command = rad;
        return command_ok;
    }
    bool readPosition(float& out) const override {
        if (!has_position) return false;
        out = position;
        return true;
    }
    bool readTorque(float& out) const override {
        out = 0.0f;
        return true;
    }
    bool release() override {
        log("release");
        ++release_calls;
        if (!release_ok) return false;
        released  = true;
        energized = false;
        return true;
    }
    bool isAlive() const override { return true; }
    bool isEnergized() const override { return energized; }
    bool clearFaults() override {
        log("clear");
        stampSeq(tag, 'c');
        ++clear_calls;
        if (clear_busy > 0) {
            --clear_busy;
            return false;  // 送信バッファが詰まっているだけ。次は通る
        }
        return clear_ok;
    }

    // --- RobStride 固有 (JointDrive の口には無い) ---------------------------
    // 本物は robstride_joint.hpp にあり、指令が途絶えたときモータが自分で
    // 脱力するまでの時間を書く。**制御基板が死んだときの最後の砦。**
    bool setCommandTimeoutMs(uint32_t ms) {
        log("timeout");
        stampSeq(tag, 't');
        ++timeout_calls;
        last_timeout_ms = ms;
        if (timeout_busy > 0) {
            --timeout_busy;
            return false;  // 送信バッファが詰まっているだけ。次は通る
        }
        return timeout_ok;
    }

    // **書けたかを確かめる唯一の手段。** 書き込みの戻り値は「送信バッファに
    // 積めた」までしか意味しない。積み直してよいのは NotSent (詰まっているだけ
    // かもしれない) だけで、NoReply / Mismatch は積み直しても同じ答えが返る。
    armctl::VerifyResult verifyCommandTimeoutMs(uint32_t ms, uint32_t reply_timeout_ms) {
        log("verify");
        stampSeq(tag, 'v');
        ++verify_calls;
        last_verify_ms               = ms;
        last_verify_reply_timeout_ms = reply_timeout_ms;
        if (verify_busy > 0) {
            --verify_busy;
            return armctl::VerifyResult::NotSent;  // 積めない = **1 本も待たない**
        }
        // **待つ操作は必ず時間を請求する。** 応答が返る場合の遅れは実測が無い
        // ので最悪 (期限いっぱい) で数える。can/platform.hpp の
        // 「実際の待ちは公称より最大 1 ms 長い」ぶんも乗せる。
        sim_stall_us +=
            static_cast<uint64_t>(reply_timeout_ms + armctl::wait_overshoot_ms) * 1000u;
        return verify_result;
    }

    void log(const char* what) {
        if (log_len < max_log) order[log_len++] = what;
    }
    // 順序ログから release を除いたもの。**割り込みの巡回と段を混ぜない。**
    bool orderIs(std::initializer_list<const char*> want) const {
        int i = 0;
        for (const char* w : want) {
            while (i < log_len && std::strcmp(order[i], "release") == 0) ++i;
            if (i >= log_len || std::strcmp(order[i], w) != 0) return false;
            ++i;
        }
        while (i < log_len && std::strcmp(order[i], "release") == 0) ++i;
        return i == log_len;
    }

    bool mode_ok    = true;
    bool command_ok = true;
    bool release_ok = true;
    bool clear_ok   = true;
    bool timeout_ok = true;
    bool energized  = false;

    // **背圧。** 「最初の N 回は断り、そのあと通る」。恒久的な *_ok とは別もので、
    // 送信バッファが一瞬詰まっているだけの状態を写す。MCP2515 は 3 本しか
    // 持たず、errata #1 の回避策で 3 本使い切ると全部空くまで積めない。
    int clear_busy   = 0;
    int mode_busy    = 0;
    int timeout_busy = 0;
    int verify_busy  = 0;

    // 積めたあとの終端。**背圧 (verify_busy) とは別もの。**
    armctl::VerifyResult verify_result = armctl::VerifyResult::Verified;

    char tag = 'S';  // 大域の順序印で軸を見分ける

    bool  has_position = true;
    float position     = 0.0f;

    int      mode_calls    = 0;
    int      command_calls = 0;
    int      release_calls = 0;
    int      clear_calls   = 0;
    int      timeout_calls = 0;
    int      verify_calls  = 0;
    uint32_t last_timeout_ms = 0;
    uint32_t last_verify_ms  = 0;
    uint32_t last_verify_reply_timeout_ms = 0;
    bool     released      = false;
    float    last_command  = 0.0f;

    const char* order[max_log] = {};
    int         log_len        = 0;
};

constexpr kin::Config kKin{600.0f,           600.0f, kin::ElbowSide::Positive,
                           10.0f * arm::DEG, 170.0f * arm::DEG, 1.0e-3f};
constexpr tx::Config kTx{arm::tx::JointTransmission{3.0f, -1, 0.0f},
                         arm::tx::JointTransmission{1.0f, +1, 0.0f}};
constexpr armctl::JogLimits kJog{};
constexpr uint32_t kTimeoutMs = 100;

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

// **待ちは呼ぶ側から渡す。** enable.hpp は pico-sdk も CAN も知らないので、
// 背圧が抜けるのを待つ手段を自分では持てない。テストでは実際に眠らず、
// 呼ばれた回数と積算だけ数える — 「粘ったか」を検査できるようにするため。
int      wait_calls    = 0;
uint32_t wait_total_us = 0;
void     testWait(uint32_t us) {
    ++wait_calls;
    wait_total_us += us;
    sim_stall_us += us;  // 積み直しの待ちも main を止めている
}
void resetWait() {
    wait_calls    = 0;
    wait_total_us = 0;
    sim_stall_us  = 0;
    g_seq_len     = 0;
    g_seq[0]      = '\0';
}

// 脱力で立ち上がり、姿勢を取り、原点を入れたところまで。**M3 の直前。**
struct Rig {
    FakeDrive          sh, el;
    armctl::Space      space = makeSpace();
    armctl::Controller c{sh, el, kKin, kTx, space, kJog};

    Rig() {
        sh.tag = 'S';
        el.tag = 'E';
    }

    void boot(bool home = true) {
        placeAt(sh, el, 0.0f, 90.0f * arm::DEG);
        c.emergencyStop();
        for (int i = 0; i < 30; ++i) c.step(kDt);  // 巡回が姿勢を取る
        if (home) c.setTransmission(kTx);
        sh.log_len = el.log_len = 0;
        sh.clear_calls = el.clear_calls = 0;
        sh.mode_calls = el.mode_calls = 0;
        sh.timeout_calls = el.timeout_calls = 0;
        sh.release_calls = el.release_calls = 0;
        sh.command_calls = el.command_calls = 0;
        sh.verify_calls = el.verify_calls = 0;
        // **ここを消しておかないと、以後の「脱力が届いた」が素通りする。**
        sh.released = el.released = false;
        // **事例ごとに消す。** 積算が持ち越されると、検査が並び順に依存する。
        resetWait();
    }
};

// 読み返しの失敗が、操作者の次の一手ごとにどの結果へ写るか。
// **「押し直せば直る」と「押し直しても直らない」を混ぜない。**
EnableResult fromVerify(armctl::VerifyResult v) {
    return v == armctl::VerifyResult::Mismatch ? EnableResult::TimeoutMismatch
                                               : EnableResult::TimeoutUnverified;
}

// 肩 → 肘 の順に短絡する。最初に Verified でなかった軸が結果を決める。
EnableResult expectFor(armctl::VerifyResult sh, armctl::VerifyResult el) {
    if (sh != armctl::VerifyResult::Verified) return fromVerify(sh);
    if (el != armctl::VerifyResult::Verified) return fromVerify(el);
    return EnableResult::Ok;
}

// **同名の NG が並ぶと、どの不変条件が破れたか読めない。** 事例名で割る。
const char* why(const char* case_name, const char* what) {
    static char buf[192];
    std::snprintf(buf, sizeof(buf), "%s — %s", case_name, what);
    return buf;
}

}  // namespace

int main() {
    test::section("N1: 通る道 — 段の順序まで見る");
    {
        Rig r;
        r.boot();
        resetWait();
        const EnableResult res = armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait);

        test::checkEq(code(res), code(EnableResult::Ok), "N1: 使能できる");
        // **詰まっていなければ 1 度も待たない。** 積み直しは背圧のためのもので、
        // 通る道で毎回止まる理由がない。
        test::checkEq(count(wait_calls), count(0), "N1: **詰まっていなければ待たない**");
        // **通信途絶保護が位置モードより前。** 逆にすると、保護の無い状態で
        // トルクが出ている窓ができる。
        test::check(r.sh.orderIs({"clear", "timeout", "verify", "mode"}),
                    "N1: 肩は 故障解除 → 通信途絶保護 → **読み返し** → 位置モード の順");
        test::check(r.el.orderIs({"clear", "timeout", "verify", "mode"}), "N1: 肘も同じ順");
        // **両軸とも見る。** 肩だけ見ていたころ、肘へ 0 を書く変異が全検査を
        // すり抜けた — 片軸だけ保護が無い状態は、保護が無いのと変わらない。
        test::checkEq(count(static_cast<int>(r.sh.last_timeout_ms)), count(100),
                      "N1: 肩へ書いた値は profile の ms そのもの");
        test::checkEq(count(static_cast<int>(r.el.last_timeout_ms)), count(100),
                      "N1: **肘へも profile の ms そのもの**");
        test::checkEq(code(r.c.mode()), code(armctl::Mode::Hold), "N1: 保持へ入る");
        test::check(!r.c.isHalted(), "N1: 止まっていない");
        test::check(r.c.isHomed(), "N1: 原点は残っている");
    }

    test::section("N2: 原点が無ければ何も送らずに断る");
    {
        Rig r;
        r.boot(false);
        const EnableResult res = armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait);

        test::checkEq(code(res), code(EnableResult::NotHomed), "N2: **NotHomed**");
        test::checkEq(count(r.sh.clear_calls + r.el.clear_calls), count(0),
                      "N2: 故障解除を送らない");
        test::checkEq(count(r.sh.timeout_calls + r.el.timeout_calls), count(0),
                      "N2: 通信途絶保護も書かない");
        test::checkEq(count(r.sh.mode_calls + r.el.mode_calls), count(0),
                      "N2: **位置モードへ入れない = トルクが出ない**");
        test::check(!r.sh.energized && !r.el.energized, "N2: どちらの軸も通電していない");
        test::check(r.c.isHalted(), "N2: 止まったまま");
        test::checkEq(code(r.c.mode()), code(armctl::Mode::Idle), "N2: 保持へも入らない");
    }

    test::section("N3: 姿勢をまだ取れていなければ、原点ではなく待ちと言う");
    {
        // **理由の分類が操作者の次の一手を決める。** 「原点を入れてください」と
        // 出すと、押しても効かない M4 を押し続けることになる。
        Rig r;
        r.sh.has_position = false;
        r.el.has_position = false;
        r.c.emergencyStop();
        for (int i = 0; i < 30; ++i) r.c.step(kDt);
        test::check(!r.c.isSeeded(), "N3: 姿勢を取れていない");

        const EnableResult res = armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait);
        test::checkEq(code(res), code(EnableResult::WaitingDrives), "N3: **WaitingDrives**");
        test::checkEq(count(r.sh.mode_calls + r.el.mode_calls), count(0), "N3: 何も使能しない");
    }

    test::section("N4: 脱力が成立していなければ断る");
    {
        Rig r;
        placeAt(r.sh, r.el, 0.0f, 90.0f * arm::DEG);
        r.c.seedFromDrives();
        r.c.setTransmission(kTx);
        r.sh.release_ok = false;
        r.c.emergencyStop();
        r.sh.clear_calls = r.el.clear_calls = 0;
        r.sh.mode_calls = r.el.mode_calls = 0;

        const EnableResult res = armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait);
        test::checkEq(code(res), code(EnableResult::NotReleased), "N4: **NotReleased**");
        test::checkEq(count(r.sh.clear_calls + r.el.clear_calls), count(0),
                      "N4: 故障解除を送らない");
        test::checkEq(count(r.sh.mode_calls + r.el.mode_calls), count(0), "N4: 使能しない");
        test::check(r.c.isHalted(), "N4: 止まったまま");
    }

    test::section("N5: 故障解除が送れなければ、原点や脱力のせいにしない");
    {
        Rig r;
        r.boot();
        r.el.clear_ok = false;

        const EnableResult res = armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait);
        test::checkEq(code(res), code(EnableResult::ClearFailed), "N5: **ClearFailed**");
        test::checkEq(count(r.sh.mode_calls + r.el.mode_calls), count(0), "N5: 使能しない");
        test::check(r.c.isHalted(), "N5: 止まったまま");
    }

    test::section("N6: 通信途絶保護が書けなければ、使能せずに脱力へ戻す");
    {
        // **fail-closed。** 復帰だけ済ませて止まらないと、LED はモード色になり
        // 腕はトルクを出していない — 色と実体が食い違う。
        Rig r;
        r.boot();
        r.el.timeout_ok = false;

        const EnableResult res = armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait);
        test::checkEq(code(res), code(EnableResult::TimeoutWriteFailed),
                      "N6: **TimeoutWriteFailed**");
        test::checkEq(count(r.sh.mode_calls + r.el.mode_calls), count(0),
                      "N6: **位置モードへ入れない**");
        test::check(r.c.isHalted(), "N6: 止まっている");
        test::checkEq(code(r.c.mode()), code(armctl::Mode::Idle), "N6: 保持へ入っていない");
        // **両軸へ書こうとする。** 片方で諦めると、次の使能で残った軸だけ
        // 前の値のままになる。
        // **送れない軸は背圧と区別がつかないので予算ぶん積み直す。** 回数を
        // 固定しておくと、予算そのものを弄る変異がここで落ちる。
        test::checkEq(count(r.sh.timeout_calls), count(1), "N6: 肩へは書いた (1 回で通る)");
        test::checkEq(count(r.el.timeout_calls), count(armctl::busy_attempts),
                      "N6: 肘へは予算ぶん積み直した");

        for (int i = 0; i < 30; ++i) r.c.step(kDt);
        test::check(r.sh.released && r.el.released, "N6: 脱力が届いている");

        // **向きを変えて同じことを見る。** 肩で諦める実装だと、肘だけ前の値の
        // まま次の使能へ進む — 片軸だけ保護が無い状態は、保護が無いのと同じ。
        Rig r2;
        r2.boot();
        r2.sh.timeout_ok = false;
        test::checkEq(code(armctl::enableArm(r2.c, r2.sh, r2.el, kTimeoutMs, testWait)),
                      code(EnableResult::TimeoutWriteFailed), "N6: 肩が書けなくても同じ理由");
        test::checkEq(count(r2.el.timeout_calls), count(1),
                      "N6: **肩が失敗しても肘へは書く**");
        test::checkEq(count(r2.sh.mode_calls + r2.el.mode_calls), count(0),
                      "N6: それでも使能しない");
    }

    test::section("N7: 位置モードへ入れなければ脱力へ戻す");
    {
        Rig r;
        r.boot();
        r.el.mode_ok = false;

        const EnableResult res = armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait);
        test::checkEq(code(res), code(EnableResult::PositionModeFailed),
                      "N7: **PositionModeFailed**");
        test::check(r.c.isHalted(), "N7: 止まっている");
        test::checkEq(code(r.c.mode()), code(armctl::Mode::Idle), "N7: 保持へ入っていない");

        for (int i = 0; i < 30; ++i) r.c.step(kDt);
        test::check(r.sh.released && r.el.released, "N7: **入れた側も脱力させる**");
        test::check(!r.sh.energized && !r.el.energized, "N7: どちらも通電していない");
    }

    test::section("N8: 肩が入れなければ肘は使能しない");
    {
        // **fail-closed。** 戻り先が脱力なのだから、通電する軸を増やす理由がない。
        Rig r;
        r.boot();
        r.sh.mode_ok = false;

        const EnableResult res = armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait);
        test::checkEq(code(res), code(EnableResult::PositionModeFailed), "N8: 同じ理由");
        test::checkEq(count(r.el.mode_calls), count(0), "N8: **肘へは送らない**");
        test::check(!r.el.energized, "N8: 肘は通電していない");
    }

    test::section("N9: 姿勢を取り直せなければ脱力へ戻す");
    {
        Rig r;
        r.boot();
        r.sh.has_position = false;  // 使能できたのに読めなくなった

        const EnableResult res = armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait);
        test::checkEq(code(res), code(EnableResult::SeedFailed), "N9: **SeedFailed**");
        test::check(r.c.isHalted(), "N9: 止まっている");
        test::checkEq(code(r.c.mode()), code(armctl::Mode::Idle), "N9: 保持へ入っていない");
    }

    test::section("N10: 運転中の M3 は何も送らない");
    {
        // **押し間違いでトルクを落とさない。** 位置モードへ入れ直すと、その
        // 先頭の脱力で保持中のトルクが一瞬抜ける。腕は自重で落ちる。
        Rig r;
        r.boot();
        test::checkEq(code(armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait)),
                      code(EnableResult::Ok), "N10: 1 度目は使能する");
        const int mode_calls = r.sh.mode_calls + r.el.mode_calls;
        const int rel        = r.sh.release_calls + r.el.release_calls;
        const int clears     = r.sh.clear_calls + r.el.clear_calls;

        const EnableResult res = armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait);
        test::checkEq(code(res), code(EnableResult::AlreadyRunning), "N10: **AlreadyRunning**");
        test::checkEq(count(r.sh.mode_calls + r.el.mode_calls), count(mode_calls),
                      "N10: 位置モードへ入れ直さない");
        test::checkEq(count(r.sh.release_calls + r.el.release_calls), count(rel),
                      "N10: 脱力も送らない");
        test::checkEq(count(r.sh.clear_calls + r.el.clear_calls), count(clears),
                      "N10: 故障解除も送り直さない");
        test::checkEq(count(r.sh.timeout_calls + r.el.timeout_calls), count(2),
                      "N10: 通信途絶保護も書き直さない");
        test::checkEq(code(r.c.mode()), code(armctl::Mode::Hold), "N10: 保持のまま");
    }

    test::section("N11: 通信途絶保護が無効なら書かない");
    {
        // profile の can_timeout_ms が 0 = 保護なし。**書かないが、使能は通る。**
        Rig r;
        r.boot();
        const EnableResult res = armctl::enableArm(r.c, r.sh, r.el, 0, testWait);
        test::checkEq(code(res), code(EnableResult::Ok), "N11: 使能できる");
        test::checkEq(count(r.sh.timeout_calls + r.el.timeout_calls), count(0), "N11: 書かない");
        test::check(r.sh.orderIs({"clear", "mode"}), "N11: 段が 1 つ抜けるだけ");
    }

    test::section("N12: 使能の最中に割り込みが回っても脱力を撃たない");
    {
        // **実機で使能に 210 ms かかる。** そのあいだ制御タイマは回り続けており、
        // 脱力フレームが使能の後に届くとモータは使能を取り消す。保持に入った
        // 200 ms 後に「通電していない」で倒れる形で出る。
        Rig r;
        r.boot();
        test::check(r.c.clearFault(), "N12: 段 1 まで進んだ");
        const int rel = r.sh.release_calls + r.el.release_calls;
        for (int i = 0; i < 250; ++i) r.c.step(kDt);  // 0.5 秒ぶん
        test::checkEq(count(r.sh.release_calls + r.el.release_calls), count(rel),
                      "N12: **段 1 から段 5 のあいだ、脱力は 1 本も出ない**");
        test::checkEq(count(r.sh.command_calls + r.el.command_calls), count(0),
                      "N12: 指令も出ない");
    }

    test::section("N13: どの結果にも操作者への文言がある");
    {
        static constexpr EnableResult all[] = {
            EnableResult::Ok,                 EnableResult::AlreadyRunning,
            EnableResult::WaitingDrives,      EnableResult::NotHomed,
            EnableResult::NotReleased,        EnableResult::ClearFailed,
            EnableResult::TimeoutWriteFailed, EnableResult::PositionModeFailed,
            EnableResult::SeedFailed,         EnableResult::TimeoutUnverified,
            EnableResult::TimeoutMismatch,
        };
        // **互いに違うこと。** 同じ文が返ると、分類が分かれている意味が消える。
        for (EnableResult a : all) {
            for (EnableResult b : all) {
                if (code(a) == code(b)) continue;
                test::check(std::strcmp(armctl::enableText(a), armctl::enableText(b)) != 0,
                            "N13: **結果ごとに違う文言**");
            }
        }
        for (EnableResult res : all) {
            const char* t = armctl::enableText(res);
            test::check(t != nullptr && t[0] != '\0', "文言がある");
            test::check(std::strcmp(t, "不明") != 0, "**「不明」で終わらせない**");
        }
        // **原点が無いときだけは、次に押すボタンが違う。**
        test::check(std::strstr(armctl::enableText(EnableResult::NotHomed), "M4") != nullptr,
                    "NotHomed は M4 を名指しする");
    }

    // ========================================================================
    // 背圧 — **送信が一瞬詰まっているのは失敗ではない**
    // ========================================================================
    //
    // 実機で使能できなかった (2026-09-12)。段 1 の故障解除が 2 本、段 2 の
    // 通信途絶保護が 2 本を、待ちを挟まずに続けて撃つ。MCP2515 の送信バッファは
    // 3 本しかなく、errata #1 の回避策により 3 本使い切ると全部が空くまで
    // 積めない。**4 本目が必ず断られる。**
    //
    // 以前は保護の書き込みが位置モードの後ろにあり、あちらの読み返しの待ちで
    // バッファが空いていた。順序を入れ替えた理由は正しいが、その待ちに
    // 依存していたことが見えていなかった。

    test::section("B1: 保護の書き込みが詰まっても、粘って使能できる");
    {
        Rig r;
        r.boot();
        resetWait();
        r.sh.timeout_busy = 2;  // 肩の 1 回目と 2 回目が断られる
        r.el.timeout_busy = 1;
        const EnableResult res = armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait);
        test::checkEq(code(res), code(EnableResult::Ok), "**使能できる**");
        test::checkEq(count(r.sh.timeout_calls), 3u, "肩は 3 回目で通った");
        test::checkEq(count(r.el.timeout_calls), 2u, "肘は 2 回目で通った");
        test::check(wait_calls >= 3, "**待ってから積み直した**");
        test::checkEq(code(r.c.mode()), code(armctl::Mode::Hold), "保持へ入った");
        test::check(!r.c.isHalted(), "止まっていない");
    }

    test::section("B2: 位置モードは積み直さない — 断って押し直してもらう");
    {
        // **積み直しは投げっぱなしの送信にだけ使う。** 位置モードへ入れる操作は
        // 中で run_mode を読み返すので 1 軸あたり約 105 ms かかる。main ループの
        // 停止監視は 500 ms で焼き、復帰の道は既に約 320 ms を使っているので、
        // 1 回積み直すだけで残りを食い潰す。**断るほうが安い。**
        Rig r;
        r.boot();
        resetWait();
        r.sh.mode_busy = 1;
        const EnableResult res = armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait);
        test::checkEq(code(res), code(EnableResult::PositionModeFailed), "**断る**");
        test::checkEq(count(r.sh.mode_calls), 1u, "**1 回しか呼ばない**");
        test::check(r.c.isHalted(), "止まっている");
        test::check(!r.sh.energized && !r.el.energized, "トルクは出していない");
    }

    test::section("B3: 故障解除が詰まっても、粘って使能できる");
    {
        Rig r;
        r.boot();
        resetWait();
        r.sh.clear_busy = 2;
        const EnableResult res = armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait);
        test::checkEq(code(res), code(EnableResult::Ok), "**使能できる**");
        test::check(r.sh.clear_calls >= 3, "積み直した");
    }

    test::section("B4: 恒久的に送れないときは、粘り尽くして失敗し、止まっている");
    {
        Rig r;
        r.boot();
        resetWait();
        r.el.timeout_ok = false;  // 背圧ではなく本当に送れない
        const EnableResult res = armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait);
        test::checkEq(code(res), code(EnableResult::TimeoutWriteFailed), "失敗を返す");
        test::check(r.c.isHalted(), "**止まっている** (fail-closed)");
        test::checkEq(count(r.el.mode_calls), 0u, "**位置モードへ入れていない**");
        test::check(wait_calls > 0, "粘りはした");
        test::check(wait_calls < 10000, "**無限には粘らない**");
    }

    test::section("B5: 片軸だけ保護が書けた状態で使能しない");
    {
        Rig r;
        r.boot();
        resetWait();
        r.sh.timeout_busy = 1;   // 肩は粘れば通る
        r.el.timeout_ok   = false;  // 肘は通らない
        const EnableResult res = armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait);
        test::checkEq(code(res), code(EnableResult::TimeoutWriteFailed), "失敗を返す");
        test::checkEq(count(r.sh.mode_calls), 0u, "肩も通電させない");
        test::checkEq(count(r.el.mode_calls), 0u, "肘も通電させない");
        test::check(r.c.isHalted(), "止まっている");
    }

    test::section("B6: 位置モードが恒久的に落ちても止まっている");
    {
        Rig r;
        r.boot();
        resetWait();
        r.el.mode_ok = false;
        const EnableResult res = armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait);
        test::checkEq(code(res), code(EnableResult::PositionModeFailed), "失敗を返す");
        test::check(r.c.isHalted(), "**止まっている**");
        test::check(wait_calls < 10000, "無限には粘らない");
    }

    test::section("B7: 待った長さそのものを見る — 回数だけでは定数が守れない");
    {
        // **wait_total_us を読む表明がここだけにある。** 積むだけで誰も読まない
        // ころ、`busy_gap_us` を 0 にしても 250000 にしても全検査が通った。
        // 0 なら 20 回が数 us で焼き切れて元の欠陥が戻り、250000 なら 1 回の
        // 使能で 15 秒止まって基板が再起動する。**どちらも緑だった。**
        Rig r;
        r.boot();
        resetWait();
        r.sh.timeout_busy = 3;
        const EnableResult res = armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait);
        test::checkEq(code(res), code(EnableResult::Ok), "B7: 使能できる");
        test::checkEq(count(wait_calls), count(3), "B7: 3 回断られたので 3 回待った");
        test::checkEq(count(static_cast<int>(wait_total_us)),
                      count(3 * static_cast<int>(armctl::busy_gap_us)),
                      "B7: **待った長さは busy_gap_us の 3 倍ちょうど**");
    }

    test::section("B8: 予算の境界を両側から押さえる");
    {
        // **予算のすぐ内側は通る。**
        Rig r;
        r.boot();
        resetWait();
        r.sh.timeout_busy = armctl::busy_attempts - 1;
        const EnableResult res = armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait);
        test::checkEq(code(res), code(EnableResult::Ok), "B8: 予算の 1 つ内側なら通る");
        test::checkEq(count(r.sh.timeout_calls), count(armctl::busy_attempts),
                      "B8: 最後の 1 回で通った");
    }
    {
        // **予算ちょうどで断る。** off-by-one で 1 回多いと、ここが通ってしまう。
        Rig r;
        r.boot();
        resetWait();
        r.sh.timeout_busy = armctl::busy_attempts;
        const EnableResult res = armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait);
        test::checkEq(code(res), code(EnableResult::TimeoutWriteFailed),
                      "B8: 予算ちょうどなら断る");
        test::checkEq(count(r.sh.timeout_calls), count(armctl::busy_attempts),
                      "B8: **予算を 1 回も超えて撃たない**");
        test::checkEq(count(static_cast<int>(wait_total_us)),
                      count(static_cast<int>(armctl::busy_attempts) *
                            static_cast<int>(armctl::busy_gap_us)),
                      "B8: 待った長さは予算ちょうど");
        test::check(r.c.isHalted(), "B8: 止まっている");
    }

    // ========================================================================
    // 保護の読み返し — **「積めた」は「書けた」ではない**
    // ========================================================================
    //
    // 背圧を吸収する前は、通信途絶保護の書き込みが毎回 NoTxBuffer で落ちていた
    // ので、その戻り値の真を誰も信用していなかった。吸収するようになって初めて
    // 「送信バッファに積めた」という未検証の主張が荷重を受ける。
    //
    // **素朴に組むと壊れる。** 読み返しは応答を待つ操作で、期限は 50 ms。
    // これを積み直しのループへ入れると 20 回 x 51 ms = 1 秒を 1 軸で使い、
    // main の停止監視 (500 ms) を焼く — 押すたびに基板が再起動する機体になる。
    // 積み直してよいのは「送れない」ときだけ。

    test::section("V1: 通る道で、両軸を 1 度ずつ読み返す");
    {
        Rig r;
        r.boot();
        armctl::AbsorbedDrops absorbed;
        absorbed.record(0xABCDEFu);  // **必ず上書きされる**ことを見る
        const EnableResult res =
            armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait, absorbed);

        test::checkEq(code(res), code(EnableResult::Ok), "V1: 使能できる");
        test::checkEq(count(r.sh.verify_calls), count(1), "V1: 肩を 1 度だけ読み返す");
        test::checkEq(count(r.el.verify_calls), count(1), "V1: 肘も 1 度だけ");
        // **書いた値そのものと比べる。** 別の値と比べる実装は、書けていなくても
        // Verified になりうる。
        test::checkEq(count(static_cast<int>(r.sh.last_verify_ms)), count(100),
                      "V1: 肩は書いた ms で確かめる");
        test::checkEq(count(static_cast<int>(r.el.last_verify_ms)), count(100),
                      "V1: 肘も書いた ms で確かめる");
        // **期限は呼ぶ側 (予算を持つ側) が渡す。** 既定値に頼ると、50 が
        // 2 か所に書かれてずれる。
        test::checkEq(static_cast<uint64_t>(r.sh.last_verify_reply_timeout_ms),
                      static_cast<uint64_t>(armctl::verify_reply_timeout_ms),
                      "V1: **肩へ渡した期限は enable.hpp の定数**");
        test::checkEq(static_cast<uint64_t>(r.el.last_verify_reply_timeout_ms),
                      static_cast<uint64_t>(armctl::verify_reply_timeout_ms),
                      "V1: 肘へも同じ期限");
        test::checkEq(static_cast<uint64_t>(absorbed.last()), 0u, "V1: 詰まっていないので 0");
        test::checkEq(count(wait_calls), count(0), "V1: 1 度も待たない");
    }

    test::section("V2: 両軸へ書き終えてから、両軸を読み返す");
    {
        // **軸をまたぐ前後関係。** 肩を書く → 肩を読む → 肘を書く、の順だと、
        // 肘へ書く前に肩の読み返しの 50 ms が挟まる。片軸ずつのログでは表せない。
        Rig r;
        r.boot();
        armctl::AbsorbedDrops absorbed;
        test::checkEq(code(armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait, absorbed)),
                      code(EnableResult::Ok), "V2: 使能できる");
        test::checkStr(g_seq, "ScEcStEtSvEvSmEm",
                       "V2: **解除(肩肘) → 保護(肩肘) → 読み返し(肩肘) → 使能(肩肘)**");
    }

    test::section("V3: 応答が無ければ使能しない — その軸を積み直さない");
    {
        Rig r;
        r.boot();
        r.sh.verify_result = armctl::VerifyResult::NoReply;
        armctl::AbsorbedDrops absorbed;
        absorbed.record(0xABCDEFu);  // **必ず上書きされる**ことを見る
        const EnableResult res =
            armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait, absorbed);

        test::checkEq(code(res), code(EnableResult::TimeoutUnverified),
                      "V3: **TimeoutUnverified**");
        test::checkEq(count(r.sh.mode_calls + r.el.mode_calls), count(0),
                      "V3: **保護が確かめられないままトルクを出さない**");
        test::checkEq(count(r.sh.verify_calls), count(1), "V3: **無応答を積み直さない**");
        test::checkEq(count(r.el.verify_calls), count(0), "V3: 肘は読み返さない (短絡)");
        test::check(r.c.isHalted(), "V3: 止まっている");
        test::checkEq(code(r.c.mode()), code(armctl::Mode::Idle), "V3: 保持へ入っていない");
        test::checkEq(static_cast<uint64_t>(absorbed.last()), 0u, "V3: **失敗しても回数は書く**");

        for (int i = 0; i < 30; ++i) r.c.step(kDt);
        test::check(r.sh.released && r.el.released, "V3: 脱力が届いている");
    }

    test::section("V4: 値が食い違えば使能しない — 押し直しでは直らない");
    {
        Rig r;
        r.boot();
        r.el.verify_result = armctl::VerifyResult::Mismatch;
        armctl::AbsorbedDrops absorbed;
        const EnableResult res =
            armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait, absorbed);

        test::checkEq(code(res), code(EnableResult::TimeoutMismatch), "V4: **TimeoutMismatch**");
        test::checkEq(count(r.sh.verify_calls), count(1), "V4: 肩は確かめ済み");
        test::checkEq(count(r.el.verify_calls), count(1), "V4: 肘は 1 度だけ (積み直さない)");
        test::checkEq(count(r.sh.mode_calls + r.el.mode_calls), count(0), "V4: 使能しない");
        test::check(r.c.isHalted(), "V4: 止まっている");

        // **向きを変えて同じことを見る。** 肩で食い違えば肘は読み返さない。
        Rig r2;
        r2.boot();
        r2.sh.verify_result = armctl::VerifyResult::Mismatch;
        armctl::AbsorbedDrops absorbed2;
        test::checkEq(code(armctl::enableArm(r2.c, r2.sh, r2.el, kTimeoutMs, testWait, absorbed2)),
                      code(EnableResult::TimeoutMismatch), "V4: 肩でも同じ理由");
        test::checkEq(count(r2.el.verify_calls), count(0), "V4: 肘は読み返さない (短絡)");
        test::checkEq(count(r2.sh.mode_calls + r2.el.mode_calls), count(0), "V4: 使能しない");
    }

    test::section("V5: 読み要求が積めないのは背圧 — 粘って読み返す");
    {
        Rig r;
        r.boot();
        r.sh.verify_busy  = 2;
        armctl::AbsorbedDrops absorbed;
        const EnableResult res =
            armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait, absorbed);

        test::checkEq(code(res), code(EnableResult::Ok), "V5: **粘れば使能できる**");
        test::checkEq(count(r.sh.verify_calls), count(3), "V5: 3 回目で積めた");
        test::checkEq(count(wait_calls), count(2), "V5: 2 回断られたので 2 回待った");
        test::checkEq(static_cast<uint64_t>(wait_total_us),
                      static_cast<uint64_t>(2u * armctl::busy_gap_us),
                      "V5: 待った長さは busy_gap_us の 2 倍ちょうど");
        test::checkEq(static_cast<uint64_t>(absorbed.last()), 2u, "V5: **吸収したのは 2 回**");
    }

    test::section("V6: 積めたあとの無応答は終端 — そこで粘らない");
    {
        Rig r;
        r.boot();
        r.sh.verify_busy   = 1;
        r.sh.verify_result = armctl::VerifyResult::NoReply;
        armctl::AbsorbedDrops absorbed;
        const EnableResult res =
            armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait, absorbed);

        test::checkEq(code(res), code(EnableResult::TimeoutUnverified), "V6: 確かめられない");
        test::checkEq(count(r.sh.verify_calls), count(2),
                      "V6: **積めた次の無応答で止める** (背圧のぶんだけ粘る)");
        test::checkEq(static_cast<uint64_t>(absorbed.last()), 1u, "V6: 吸収は 1 回");
    }

    test::section("V7: 読み要求の積み直しは予算ちょうどで止まる");
    {
        Rig r;
        r.boot();
        r.sh.verify_busy  = armctl::busy_attempts - 1;
        armctl::AbsorbedDrops absorbed;
        test::checkEq(code(armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait, absorbed)),
                      code(EnableResult::Ok), "V7: 予算の 1 つ内側なら通る");
        test::checkEq(count(r.sh.verify_calls), count(armctl::busy_attempts),
                      "V7: 最後の 1 回で積めた");
    }
    {
        Rig r;
        r.boot();
        r.sh.verify_busy  = armctl::busy_attempts;
        armctl::AbsorbedDrops absorbed;
        const EnableResult res =
            armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait, absorbed);
        test::checkEq(code(res), code(EnableResult::TimeoutUnverified),
                      "V7: 予算ちょうどなら断る");
        test::checkEq(count(r.sh.verify_calls), count(armctl::busy_attempts),
                      "V7: **予算を 1 回も超えて撃たない**");
        test::checkEq(static_cast<uint64_t>(wait_total_us),
                      static_cast<uint64_t>(static_cast<uint32_t>(armctl::busy_attempts) *
                                            armctl::busy_gap_us),
                      "V7: 待った長さは予算ちょうど");
        test::checkEq(static_cast<uint64_t>(absorbed.last()),
                      static_cast<uint64_t>(armctl::busy_attempts), "V7: 吸収は予算ぶん");
        test::check(r.c.isHalted(), "V7: 止まっている");
    }

    test::section("V8: 書けていない軸を読み返さない");
    {
        // **分類の混線。** 書き込みが予算内に送れなかったのに読み返しへ進むと、
        // 「書けません」が「確かめられません」に化けて、操作者が見る場所が変わる。
        Rig r;
        r.boot();
        r.el.timeout_ok   = false;
        armctl::AbsorbedDrops absorbed;
        const EnableResult res =
            armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait, absorbed);

        test::checkEq(code(res), code(EnableResult::TimeoutWriteFailed),
                      "V8: **TimeoutWriteFailed のまま**");
        test::checkEq(count(r.sh.verify_calls + r.el.verify_calls), count(0),
                      "V8: **1 度も読み返さない**");
    }

    test::section("V9: 積み直しが火を噴く 5 か所が、予算の定数と揃っている");
    {
        // 故障解除・肩の書き込み・肘の書き込み・肩の読み要求・肘の読み要求。
        // **故障解除だけは門を通す 1 回目が積み直しの外**にあるので、
        // 同じ回数だけ待たせるには 1 つ多く詰まらせる。
        Rig r;
        r.boot();
        r.sh.clear_busy   = armctl::busy_attempts;
        r.sh.timeout_busy = armctl::busy_attempts - 1;
        r.el.timeout_busy = armctl::busy_attempts - 1;
        r.sh.verify_busy  = armctl::busy_attempts - 1;
        r.el.verify_busy  = armctl::busy_attempts - 1;
        armctl::AbsorbedDrops absorbed;
        const EnableResult res =
            armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait, absorbed);

        test::checkEq(code(res), code(EnableResult::Ok), "V9: 全部の山を越えて使能できる");
        test::checkEq(count(wait_calls),
                      count(armctl::retry_sites * (armctl::busy_attempts - 1)),
                      "V9: **5 か所すべてが積み直した**");
        test::checkEq(static_cast<uint64_t>(absorbed.last()), static_cast<uint64_t>(wait_calls),
                      "V9: 吸収した回数は待った回数");
        test::check(wait_total_us <= armctl::enable_retry_budget_us,
                    "V9: **予算を超えない**");
    }

    test::section("V10: main を止める時間が、予算に収まる");
    {
        // **この節が「読み返しを積み直しのループへ入れる」実装を落とす。**
        // 回数の検査は全部通ってしまう — 落ちるのは時間だけ。実機では
        // 「M3 を押すたびに基板が再起動する」という形で出る。
        struct Case {
            const char*          name;
            uint32_t             timeout_ms;
            int                  clear_busy;
            int                  sh_timeout_busy;
            int                  el_timeout_busy;
            int                  sh_verify_busy;
            int                  el_verify_busy;
            armctl::VerifyResult sh_verify;
            armctl::VerifyResult el_verify;
            bool                 el_timeout_ok;
        };
        static const Case cases[] = {
            {"通る道", kTimeoutMs, 0, 0, 0, 0, 0, armctl::VerifyResult::Verified,
             armctl::VerifyResult::Verified, true},
            {"肩が無応答", kTimeoutMs, 0, 0, 0, 0, 0, armctl::VerifyResult::NoReply,
             armctl::VerifyResult::Verified, true},
            {"肘が無応答", kTimeoutMs, 0, 0, 0, 0, 0, armctl::VerifyResult::Verified,
             armctl::VerifyResult::NoReply, true},
            {"両軸とも無応答", kTimeoutMs, 0, 0, 0, 0, 0, armctl::VerifyResult::NoReply,
             armctl::VerifyResult::NoReply, true},
            {"肘が食い違い", kTimeoutMs, 0, 0, 0, 0, 0, armctl::VerifyResult::Verified,
             armctl::VerifyResult::Mismatch, true},
            {"読み要求が詰まってから通る", kTimeoutMs, 0, 0, 0, armctl::busy_attempts - 1,
             armctl::busy_attempts - 1, armctl::VerifyResult::Verified,
             armctl::VerifyResult::Verified, true},
            {"読み要求が焼き切れる", kTimeoutMs, 0, 0, 0, armctl::busy_attempts, 0,
             armctl::VerifyResult::Verified, armctl::VerifyResult::Verified, true},
            {"5 か所すべて詰まる", kTimeoutMs, armctl::busy_attempts, armctl::busy_attempts - 1,
             armctl::busy_attempts - 1, armctl::busy_attempts - 1, armctl::busy_attempts - 1,
             armctl::VerifyResult::Verified, armctl::VerifyResult::Verified, true},
            {"保護が書けない", kTimeoutMs, 0, 0, armctl::busy_attempts, 0, 0,
             armctl::VerifyResult::Verified, armctl::VerifyResult::Verified, false},
            {"保護が無効", 0, 0, 0, 0, 0, 0, armctl::VerifyResult::Verified,
             armctl::VerifyResult::Verified, true},
        };

        for (const Case& cs : cases) {
            Rig r;
            r.boot();
            r.sh.clear_busy    = cs.clear_busy;
            r.sh.timeout_busy  = cs.sh_timeout_busy;
            r.el.timeout_busy  = cs.el_timeout_busy;
            r.sh.verify_busy   = cs.sh_verify_busy;
            r.el.verify_busy   = cs.el_verify_busy;
            r.sh.verify_result = cs.sh_verify;
            r.el.verify_result = cs.el_verify;
            r.el.timeout_ok    = cs.el_timeout_ok;
            armctl::AbsorbedDrops absorbed;
        absorbed.record(0xABCDEFu);  // **必ず上書きされる**ことを見る
            (void)armctl::enableArm(r.c, r.sh, r.el, cs.timeout_ms, testWait, absorbed);

            test::check(sim_stall_us <= armctl::enable_stall_budget_us,
                        why(cs.name, "**main を止める時間が予算に収まる**"));
            test::checkEq(static_cast<uint64_t>(absorbed.last()), static_cast<uint64_t>(wait_calls),
                          why(cs.name, "吸収した回数は待った回数"));
            test::check(wait_total_us <= armctl::enable_retry_budget_us,
                        why(cs.name, "積み直しの予算を超えない"));
            test::check(r.sh.verify_calls <= armctl::busy_attempts &&
                            r.el.verify_calls <= armctl::busy_attempts,
                        why(cs.name, "**読み返しは予算を超えて撃たない**"));
        }
    }

    test::section("V11: 保護なしのトルク窓が無い (肩 x 肘 の決定表)");
    {
        static const armctl::VerifyResult kAll[] = {
            armctl::VerifyResult::Verified,
            armctl::VerifyResult::NoReply,
            armctl::VerifyResult::Mismatch,
        };
        for (armctl::VerifyResult sv : kAll) {
            for (armctl::VerifyResult ev : kAll) {
                Rig r;
                r.boot();
                r.sh.verify_result = sv;
                r.el.verify_result = ev;
                armctl::AbsorbedDrops absorbed;
                const EnableResult res =
                    armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait, absorbed);

                const bool both = sv == armctl::VerifyResult::Verified &&
                                  ev == armctl::VerifyResult::Verified;
                const EnableResult want = expectFor(sv, ev);
                test::checkEq(code(res), code(want),
                              "V11: 結果は最初に確かめられない軸が決める");
                test::check((r.sh.mode_calls + r.el.mode_calls == 0) != both,
                            "V11: **両軸が Verified のときだけトルクが出る**");
                test::check(r.c.isHalted() != both, "V11: そうでなければ止まっている");
            }
        }
    }

    test::section("V12: 回数を受け取らない既存の呼び方も同じ段を踏む");
    {
        Rig r;
        r.boot();
        const EnableResult res = armctl::enableArm(r.c, r.sh, r.el, kTimeoutMs, testWait);
        test::checkEq(code(res), code(EnableResult::Ok), "V12: 使能できる");
        test::checkEq(count(r.sh.verify_calls), count(1), "V12: **読み返しも通る**");
        test::checkEq(count(r.el.verify_calls), count(1), "V12: 肘も同じ");
    }

    test::section("V13: 保護が無効なら読み返しもしない");
    {
        Rig r;
        r.boot();
        armctl::AbsorbedDrops absorbed;
        absorbed.record(0xABCDEFu);  // **必ず上書きされる**ことを見る
        const EnableResult res = armctl::enableArm(r.c, r.sh, r.el, 0, testWait, absorbed);
        test::checkEq(code(res), code(EnableResult::Ok), "V13: 使能できる");
        test::checkEq(count(r.sh.verify_calls + r.el.verify_calls), count(0),
                      "V13: **書かないものは確かめない**");
        test::check(r.sh.orderIs({"clear", "mode"}), "V13: 段が 2 つ抜けるだけ");
        test::checkEq(static_cast<uint64_t>(absorbed.last()), 0u, "V13: 回数は 0 で上書きされる");
    }

    test::section("V14: 既存の結果の番号を動かしていない");
    {
        // **末尾に足す。** 途中へ挿すと、ログに残った数値の意味が変わる。
        test::checkEq(code(EnableResult::Ok), 0u, "V14: Ok は 0");
        test::checkEq(code(EnableResult::TimeoutWriteFailed), 6u, "V14: TimeoutWriteFailed は 6");
        test::checkEq(code(EnableResult::SeedFailed), 8u, "V14: **SeedFailed は 8 のまま**");
        test::checkEq(code(EnableResult::TimeoutUnverified), 9u, "V14: 新しい値は末尾");
        test::checkEq(code(EnableResult::TimeoutMismatch), 10u, "V14: その次");
    }

    test::section("V15: 予算の定数が読み返しを含んでいる");
    {
        // **一次資料。** 期限 50 ms は robstride/motor.hpp の既定 (段 3 の
        // run_mode 読み返しが同じ値を使っている)。1 ms は can/platform.hpp の
        // 「実際の待ちは公称より最大 1 ms 長い」。
        test::checkEq(static_cast<uint64_t>(armctl::verify_reply_timeout_ms), 50u,
                      "V15: 読み返しの期限は 50 ms");
        test::checkEq(static_cast<uint64_t>(armctl::wait_overshoot_ms), 1u,
                      "V15: 待ちの超過は 1 ms");
        test::checkEq(count(armctl::retry_sites), count(5), "V15: 積み直す場所は 5 か所");
        test::checkEq(static_cast<uint64_t>(armctl::enable_retry_budget_us), 25000u,
                      "V15: 積み直しの予算は 25 ms");
        test::checkEq(static_cast<uint64_t>(armctl::enable_verify_budget_us), 102000u,
                      "V15: **読み返しの予算は 2 軸 x 51 ms**");
        test::checkEq(static_cast<uint64_t>(armctl::enable_stall_budget_us), 127000u,
                      "V15: 止めうる上限は 127 ms");
        // **和であること。** どちらかの項を落とす変異は、値の検査だけでは残る。
        test::checkEq(static_cast<uint64_t>(armctl::enable_stall_budget_us),
                      static_cast<uint64_t>(armctl::enable_retry_budget_us) +
                          static_cast<uint64_t>(armctl::enable_verify_budget_us),
                      "V15: **上限は積み直しと読み返しの和**");
        // 復帰の道は既に約 320 ms を使う (main.cpp の停止監視は 500 ms)。
        test::check(armctl::enable_stall_budget_us / 1000u + 320u < 500u,
                    "V15: **復帰の道と足しても停止監視を焼かない**");
    }

    test::section("V16: 分類ごとに、操作者の次の一手が書いてある");
    {
        const char* unver = armctl::enableText(EnableResult::TimeoutUnverified);
        const char* mism  = armctl::enableText(EnableResult::TimeoutMismatch);
        const char* wrote = armctl::enableText(EnableResult::TimeoutWriteFailed);

        test::check(std::strcmp(unver, mism) != 0, "V16: 2 つの失敗は別の文言");
        test::check(std::strcmp(unver, wrote) != 0, "V16: 書けないときとも別");
        // **押し直せば直りうる。** どこを見るかも書く。
        test::check(std::strstr(unver, "M3") != nullptr,
                    "V16: **確かめられないときは押し直しを勧める**");
        // **押し直しでは直らない。** 勧めると人が押し続ける。
        test::check(std::strstr(mism, "もう一度") == nullptr,
                    "V16: **食い違いに押し直しを勧めない**");
        test::check(std::strstr(mism, "profile") != nullptr,
                    "V16: 値の出どころ (profile.hpp) を名指しする");
    }

    return test::report("arm_enable");
}

