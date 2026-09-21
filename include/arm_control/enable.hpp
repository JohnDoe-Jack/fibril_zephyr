#pragma once
// **使能の手順を 1 本にする。** ゲームパッドの M3 とコンソールの `c` が
// 同じものを呼ぶ。
//
// ============================================================================
// なぜ main.cpp に置かないか
// ============================================================================
//
// main.cpp はホスト側テストから 1 本もリンクされない。段の順序も失敗したときの
// 後始末も、あちらに置くと誰も実行で検査できない。**通信途絶保護を書く位置と
// 位置モードへ入れる位置の前後関係**は、保護の無い状態でトルクが出ている窓が
// できるかどうかを決めるのに、ここへ出さなければ反証できない。
//
// 2 写しになっていたころ、片方だけ直した変更がゲートを通っていた。
//
// ドライブを型引数で受けるのは、通信途絶保護がアクチュエータ固有の設定で
// JointDrive の口に無いため。**この 1 枚はモータの型を知らないままでいられる。**

#include "arm_control/controller.hpp"
#include "arm_control/tx_drops.hpp"

namespace armctl {

enum class EnableResult : uint8_t {
    Ok = 0,
    AlreadyRunning,      // 止まっていない。何も送らずに成功
    WaitingDrives,       // まだ実測姿勢を取れていない
    NotHomed,            // 原点合わせがまだ
    NotReleased,         // 脱力が成立していない
    ClearFailed,         // 故障解除が送れない
    TimeoutWriteFailed,  // 通信途絶保護が書けない
    PositionModeFailed,  // 位置モードへ入れない
    SeedFailed,          // 姿勢を取り直せない
    // **末尾に足す。** 途中へ挿すと、ログに残った数値の意味が変わる。
    //
    // 分けてあるのは操作者の次の一手が違うから。「確かめられない」は押し直せば
    // 直りうるが、「食い違う」は押し直しても同じ答えが返る — 混ぜると人が
    // 効かないボタンを押し続ける。
    TimeoutUnverified,  // 保護を書けたか確かめられない (要求が積めない / 応答が無い)
    TimeoutMismatch,    // 保護の読み返しが書いた値と一致しない
};

// 操作者が次に何をすればよいかまで書く。**「失敗しました」で終わらせない。**
const char* enableText(EnableResult r);

// **送信が一瞬詰まっているのは失敗ではない。**
//
// 段 1 の故障解除が 2 本、段 2 の通信途絶保護が 2 本を、待ちを挟まずに続けて
// 撃つ。MCP2515 の送信バッファは 3 本しかなく、silicon errata #1 の回避策に
// より 3 本使い切ると全部が空くまで積めないので、**素で撃つと 4 本目が必ず
// 断られる。** 以前は保護の書き込みが位置モードの後ろにあり、あちらが run_mode
// を読み返して確かめる待ちのあいだにバッファが空いていた。順序を前へ出した
// ときに、その待ちに依存していたことが見えていなかった。
//
// 1 Mbps・拡張 ID・8 バイトのフレームを数えると、スタッフの対象が
// SOF 1 + ID と制御 38 + データ 64 + CRC 15 = 118 bit、最大スタッフが 29 bit、
// スタッフされない CRC デリミタ・ACK・EOF・IFS が 13 bit で、**最悪 160 bit
// = 160 us**。3 本が掃けるのに最悪 480 us。脱力中の巡回 (20 Hz で 2 本) と
// その応答が最悪で重なっても約 1.5 ms なので、その 3 倍を取る。
inline constexpr uint32_t busy_gap_us   = 250;
inline constexpr int      busy_attempts = 20;  // 250 us x 20 = 5 ms

// **待たない積み直しは元の欠陥そのもの。** 20 回が数 us で焼き切れて、
// 3 本が掃ける 480 us を一度も待たない。
static_assert(busy_gap_us > 0, "待たない積み直しは元の欠陥そのもの");
static_assert(static_cast<uint32_t>(busy_attempts) * busy_gap_us >= 2000u,
              "3 本が掃ける 0.5 ms の 4 倍は要る");

// 保護の読み返しが応答を待つ期限 [ms]。
//
// **一次資料は robstride/motor.hpp の既定 50 ms。** 段 3 の run_mode も
// 同じ経路 (型 17 の読み) を同じ値で使っている。型 17 応答の遅れの実測は無いので
// 短くする根拠が無い。**定数をここに置いて明示的に渡す** — MotorBase の既定値に
// 頼ると、予算を組む側と実際に待つ側の 2 か所に 50 が書かれてずれる。
inline constexpr uint32_t verify_reply_timeout_ms = 50;

// 締切つきの待ちは公称より最大 1 ms 長い (can/platform.hpp)。予算はその最悪で切る。
inline constexpr uint32_t wait_overshoot_ms = 1;

// 1 回の使能で積み直しが火を噴きうる場所の数。
// 故障解除・肩の保護・肘の保護・肩の読み要求・肘の読み要求。
inline constexpr int retry_sites = 5;

// 積み直しが main を止めうる長さ。
inline constexpr uint32_t enable_retry_budget_us =
    static_cast<uint32_t>(retry_sites) * static_cast<uint32_t>(busy_attempts) * busy_gap_us;

// 読み返しが main を止めうる長さ。**2 軸ぶんが必ず乗る** (短絡しても最悪は 2 軸)。
inline constexpr uint32_t enable_verify_budget_us =
    2u * (verify_reply_timeout_ms + wait_overshoot_ms) * 1000u;

// 1 回の使能が main を止めうる上限。
// **限度は main.cpp の停止監視が決める**ので、突き合わせはそちらで行う。
// 片方の項を落とすと「予算は守られているのに実機が再起動する」形で壊れるので、
// 和そのものを名前にして main.cpp からはこれだけを見る。
inline constexpr uint32_t enable_stall_budget_us =
    enable_retry_budget_us + enable_verify_budget_us;

// 積み直しは**投げっぱなしの送信にだけ**使う。
//
// **位置モードへ入れる操作は積み直さない。** あちらは中で run_mode を読み返す
// ので 1 軸あたり約 105 ms かかる (2 軸で実測 210 ms)。予算ぶん積み直すと
// 1 軸で 2.1 s になり、main ループの停止監視 (500 ms) を確実に焼く。
// 1 回だけでも 105 ms で、復帰の道に残る余裕 180 ms の半分以上を食う。
// **断って人に押し直してもらうほうが安い。**
template <class Fn, class Wait>
bool sendUntilAccepted(Fn&& op, Wait&& wait_us) {
    for (int i = 0; i < busy_attempts; ++i) {
        if (op()) return true;
        wait_us(busy_gap_us);
    }
    return false;  // 予算を使い切った。**恒久的に送れない側と同じ扱いにする**
}

// 保護の読み返しは**「積めない」ときだけ**積み直す。
//
// **待つ操作を積み直しのループへ入れてはいけない。** 読み返しは応答を期限まで
// 待つので、予算ぶん (20 回) 回すと 1 軸で 20 x 51 ms = 1 秒を使い、main の
// 停止監視 (500 ms) を確実に焼く — 押すたびに基板が再起動する機体になる。
// NoReply と Mismatch は積み直しても同じ答えが返るので、そのまま呼ぶ側へ返す。
template <class Drive, class Wait>
VerifyResult verifyUntilSent(Drive& d, uint32_t ms, Wait&& wait_us) {
    for (int i = 0; i < busy_attempts; ++i) {
        const VerifyResult v = d.verifyCommandTimeoutMs(ms, verify_reply_timeout_ms);
        if (v != VerifyResult::NotSent) return v;
        wait_us(busy_gap_us);
    }
    return VerifyResult::NotSent;  // 予算を使い切った。**恒久的に送れない側と同じ扱い**
}

// 読み返しの失敗を、操作者の次の一手で分ける。
// **畳まない** — 「確かめられない」は押し直せば直りうるが、「食い違う」は
// 押し直しても同じ答えが返る。混ぜると人が効かないボタンを押し続ける。
constexpr EnableResult verifyFailure(VerifyResult v) {
    return v == VerifyResult::Mismatch ? EnableResult::TimeoutMismatch
                                       : EnableResult::TimeoutUnverified;
}

// 使能の 1 本道。**どの段で失敗しても、戻ったときは必ず止まっている。**
//
// 段の順に:
//   0   運転中なら何も送らない
//   1   復帰 (原点・脱力・故障解除の門はすべて Controller の中)
//   2a  通信途絶保護を**両軸へ**書く
//   2b  書いた保護を**両軸で読み返す**。トルクは両軸が確かめ終わるまで出さない
//   3   位置モードへ入れる (= 使能。ここで初めてトルクが出る)
//   4   実測姿勢を取り直す
//   5   保持へ入る
//
// 吸収した回数は `absorbed` へ**数そのものではなく累積器として**渡す。
// **結果に依らず必ず記録する。** 積み直した 1 回ごとに送信落ちのカウンタが
// 1 つ以上増えるので、呼ぶ側はこれを異常の勘定から外せる — 外さないと、
// 最初の成功した使能以降ずっと CAN の警告行が点き、本物のバスオフがその
// 常時点灯に埋もれる。
//
// **数を返して呼ぶ側に足させない。** 足し込みは main.cpp に置かれることに
// なるが、あちらはホスト側テストから 1 本もリンクされないので、係数を変えても
// 直後に 0 で潰しても 4 つのゲートが緑のまま通る (変異で確かめた)。
// 累積器を渡せば、呼ぶ側に算術が 1 つも残らない。
//
// **引数名は profile.hpp の can_timeout_ms と別にしてある。** 同じ名前だと、
// profile.hpp を先に include した翻訳単位でだけ -Wshadow が出る — 警告が
// include の順で出たり出なかったりするので、ファイルを 1 枚足しただけで
// 厳しい旗のビルドが落ちる。名前を分けておけば順序に依存しない。
template <class Drive, class Wait>
EnableResult enableArm(Controller& c, Drive& shoulder, Drive& elbow, uint32_t command_timeout_ms,
                       Wait&& wait_us, AbsorbedDrops& absorbed) {
    uint32_t count = 0;
    // **数えるのは待った回数そのもの。** 積み直しの中でしか待たないので、
    // ここを通せば「吸収した回数」の定義が 1 か所に閉じる。
    auto wait = [&wait_us, &count](uint32_t us) {
        ++count;
        wait_us(us);
    };
    // **どの出口を通っても記録する。** 途中で return する経路が 9 本あるので、
    // 書き忘れを防ぐには出口ではなく破棄のときに記録するしかない。
    struct Recorder {
        AbsorbedDrops& sink;
        const uint32_t& n;
        ~Recorder() { sink.record(n); }
    } recorder{absorbed, count};

    // **運転中に押されたら何もしない。** ここで位置モードへ入れ直すと、その
    // 手順の先頭にある脱力で保持中のトルクが一瞬抜ける。押し間違いで腕が落ちる。
    if (!c.isHalted()) return EnableResult::AlreadyRunning;

    // **門はすべて clearFault() の中。** ここで条件を書き写すと、写した側を
    // 消してもホストのゲートからは見えない。理由の分類だけを読み出す。
    if (!c.clearFault()) {
        if (!c.isSeeded()) return EnableResult::WaitingDrives;
        if (!c.isHomed()) return EnableResult::NotHomed;
        if (!c.isReleased()) return EnableResult::NotReleased;
        // **門はすべて通ったのに偽 = 故障解除が送れなかった。** 詰まっている
        // だけかもしれないので積み直す。断られても状態は変わらないので、
        // 積み直しは門をもう一度通るだけで済む。
        if (!sendUntilAccepted([&c] { return c.clearFault(); }, wait)) {
            return EnableResult::ClearFailed;
        }
    }

    // **使能より先に通信途絶保護を書く。** 逆にすると、保護の無いままトルクが
    // 出ている窓ができる。モータ側のこの設定はモータの電源断で消えるので、
    // 起動時に 1 度ではなく**使能のたびに**書く — E-stop が 24V 側に入っている
    // 運用では、基板が生きたままモータだけ電源が落ちることが普通に起きる。
    if (command_timeout_ms != 0) {
        // **短絡させない。** 片軸だけ保護が無い状態は、保護が無いのと変わらない。
        const bool sh_ok = sendUntilAccepted(
            [&shoulder, command_timeout_ms] {
                return shoulder.setCommandTimeoutMs(command_timeout_ms);
            },
            wait);
        const bool el_ok = sendUntilAccepted(
            [&elbow, command_timeout_ms] { return elbow.setCommandTimeoutMs(command_timeout_ms); },
            wait);
        if (!sh_ok || !el_ok) {
            // **fail-closed。** 復帰だけ済ませて止まらないでいると、LED は
            // モード色になるのに腕はトルクを出していない。
            c.emergencyStop();
            return EnableResult::TimeoutWriteFailed;
        }

        // **書けたことを読み返しで確かめてから使能する。** 書き込みの真は
        // 「送信バッファに積めた」までしか意味せず、制御基板が死んだときの
        // 最後の砦をその主張だけで信じると、保護の無いトルク窓がここにできる。
        //
        // **両軸へ書き終えてから読む。** 軸ごとに書く → 読むの順にすると、
        // 肘へ書く前に肩の応答待ちが挟まり、その間に保護なしの肘が残る。
        const VerifyResult sh_v = verifyUntilSent(shoulder, command_timeout_ms, wait);
        if (sh_v != VerifyResult::Verified) {
            c.emergencyStop();
            return verifyFailure(sh_v);
        }
        const VerifyResult el_v = verifyUntilSent(elbow, command_timeout_ms, wait);
        if (el_v != VerifyResult::Verified) {
            c.emergencyStop();
            return verifyFailure(el_v);
        }
    }

    // **こちらは短絡させる。** 戻り先が脱力なのだから、通電する軸を増やして
    // 得るものがない。
    if (!shoulder.enterPositionMode() || !elbow.enterPositionMode()) {
        c.emergencyStop();
        return EnableResult::PositionModeFailed;
    }

    // 使能した瞬間に跳ねないよう、取り直してから保持で入る。
    if (!c.seedFromDrives()) {
        c.emergencyStop();
        return EnableResult::SeedFailed;
    }

    c.hold();
    return EnableResult::Ok;
}

// 吸収した回数を要らない呼び手のために。**戻り値の型は動かさない** —
// `== EnableResult::Ok` で比べている既存の呼び手をすべて壊すことになる。
template <class Drive, class Wait>
EnableResult enableArm(Controller& c, Drive& shoulder, Drive& elbow, uint32_t command_timeout_ms,
                       Wait&& wait_us) {
    AbsorbedDrops absorbed;
    return enableArm(c, shoulder, elbow, command_timeout_ms, static_cast<Wait&&>(wait_us),
                     absorbed);
}

}  // namespace armctl

