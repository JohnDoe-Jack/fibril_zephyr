// **出荷される機体の値そのものを検査する。**
//
// profile.hpp はこれまで src/arm_control/main.cpp からしか include されていなかった。
// つまり値が成立しているかを見るのは実機の起動時 (`kin::validate` / `tx::validate`)
// だけで、**間違いに気づくのはアームの前に立ったとき**だった。ここへ持ってくると
// `mise run verify` が同じことを言う。
//
// 数値そのものは書かない。肘角も原点オフセットも実機計測後に変わる値で、
// ここへ写せば「変更したら 2 か所直す」が生まれ、テストは値を追認するだけの
// ものになる。**測り直しても成り立っていなければならない性質**だけを置く。
//
// リンクするのは kinematics / transmission / motion の 3 本だけ。profile.hpp が
// CAN もモータも知らないことの実演を兼ねている。

#include "test_util.hpp"

#include "arm_control/profile.hpp"
// **制御側の猶予周期を見るためだけに include する。** 噛み合わせの検査は
// 両方を見ないと書けず、片方しか見ない場所に置くと成立しない。
// リンクするものは増えない (constexpr を 1 つ読むだけ)。
#include "arm_control/controller.hpp"

namespace {

using arm::JointAngles;
using arm::Point;
namespace kin = arm::kin;
namespace tx  = arm::tx;

uint64_t kcode(kin::Status s) { return static_cast<uint64_t>(s); }
uint64_t tcode(tx::Status s) { return static_cast<uint64_t>(s); }

}  // namespace

int main() {
    const kin::Config kc = armctl::kinematicsConfig();
    const tx::Config  tc = armctl::transmissionConfig();

    test::section("1. main.cpp が起動時にやる検査を、ゲートでもやる");
    {
        test::checkEq(kcode(kin::validate(kc)), kcode(kin::Status::Ok),
                      "運動学の設定が成立している");
        test::checkEq(tcode(tx::validate(tc)), tcode(tx::Status::Ok), "伝達比の設定が成立している");
    }

    test::section("2. 肘角の範囲は 2 つの死点をどちらも除いている");
    {
        // 伸びきり (0 度) と折り畳み (180 度) の**両方**が死点。
        // 実機を測って範囲を狭めることはあっても、端に触れてはいけない。
        test::check(armctl::elbow_min_deg > 0.0f, "下限が 0 度 (伸びきり) より大きい");
        test::check(armctl::elbow_max_deg < 180.0f, "上限が 180 度 (折り畳み) より小さい");
        test::check(armctl::elbow_min_deg < armctl::elbow_max_deg, "下限 < 上限");

        // 特異点の閾値は **肘角の範囲より弱く**なければならない。逆転すると、
        // 範囲の中にいるのに特異点として弾かれる姿勢が生まれる。
        const float sin_min = sinf(armctl::elbow_min_deg * arm::DEG);
        test::check(armctl::singularity_eps < sin_min,
                    "特異点の閾値が sin(肘角の下限) より小さい (範囲内を弾かない)");
    }

    test::section("3. 到達可能な円環が潰れていない");
    {
        const float inner = kin::reach_inner(kc);
        const float outer = kin::reach_outer(kc);
        test::check(inner > 0.0f, "内半径が正");
        test::check(outer > inner, "外半径 > 内半径");
        test::check(outer <= armctl::link1_mm + armctl::link2_mm,
                    "外半径がリンク長の和を超えない");
    }

    test::section("4. 原点合わせの基準姿勢そのものが合法");
    {
        // `h` を押した瞬間に「この姿勢は作れない」と言われる設定にはできない。
        // **肘角は 2 つの角度の差で決まる**ので、片方だけ直すと壊れる。
        const JointAngles home{armctl::home_theta1_deg * arm::DEG,
                               armctl::home_theta2_deg * arm::DEG};
        test::checkEq(kcode(kin::check_joints(kc, home)), kcode(kin::Status::Ok),
                      "基準姿勢が肘角の範囲にも曲がる向きにも合っている");
    }

    // `g` の行き先は設置場所の値になったので site.hpp へ移した。
    // 到達可能かどうかと作業領域に入っているかは tests/test_arm_site.cpp が見る。

    test::section("6. 運転の上限");
    {
        const arm::motion::TrapezoidProfile mp = armctl::moveProfile();
        test::check(mp.v_max > 0.0f && mp.a_max > 0.0f, "自動移動の速度・加速度が正");
        test::check(armctl::jog_v_max > 0.0f && armctl::jog_a_max > 0.0f, "手先ジョグが正");
        test::check(armctl::joint_jog_v_max > 0.0f && armctl::joint_jog_a_max > 0.0f,
                    "関節ジョグが正");

        // **手で動かすほうが速いことはない。** ジョグは人が見ながら出す指令で、
        // 自動移動より速く動く理由が無い。逆転していたら設定の書き間違い。
        test::check(armctl::jog_v_max <= mp.v_max, "手先ジョグは自動移動より速くない");

        test::check(armctl::joint_speed_limit_rad_s > 0.0f && armctl::joint_current_limit_a > 0.0f,
                    "モータへ渡す上限が正");

        // **モータ速度の上限が、手先速度と姿勢から要る値を満たしているか。**
        //
        // 手先 1 mm/s に要る関節速度は姿勢で変わる (ヤコビアンの逆)。いちばん要るのは
        // 肘角が下限のとき (伸びきりに近い) で、肩は減速比があるので
        //
        //   必要なモータ速度 = 減速比 × v / (l1 · sin e_min)
        //
        // 足りないと境界付近でモータが先に頭打ちになり、**指令だけが先に進んで
        // 実機が遅れる** — 離した瞬間に追いつくので、動きが引っかかる。
        //
        // **この 4 つ (リンク長・肘角の下限・手先速度・モータ速度の上限) は式で
        // 結ばれていて、1 つ動かすと残りが黙って破れる。** 2026-09-18 に上腕を
        // 500 → 480 mm へ作り直したとき、必要な速度が 7.89 → 8.22 rad/s へ上がって
        // 当時の上限 8.0 を超えた。**人がコメントを読んで気づくまで、どの検査も
        // 鳴らなかった。** それを鳴らすのがこの検査。
        //
        // sin を定数式で書けないのでここ (実行時) に置く — ホストの libc++ では
        // std::sin が constexpr にならず、arm-none-eabi-g++ では通るため、
        // static_assert にするとホストのゲートだけが赤くなる。
        const float sin_e_min  = sinf(armctl::elbow_min_deg * arm::DEG);
        const float need_rad_s = armctl::shoulder_ratio * mp.v_max / (armctl::link1_mm * sin_e_min);
        char         msg[128];
        std::snprintf(msg, sizeof msg,
                      "モータ速度の上限 %.2f rad/s が、伸びきりで要る %.2f rad/s 以上",
                      static_cast<double>(armctl::joint_speed_limit_rad_s),
                      static_cast<double>(need_rad_s));
        test::check(armctl::joint_speed_limit_rad_s >= need_rad_s, msg);
    }

    test::section("7. アクチュエータの割り当て");
    {
        test::check(armctl::shoulder_can_id != armctl::elbow_can_id, "2 軸の CAN_ID が違う");
        // 0 は robstride::scan() が叩かない。128 以上は 0x200a CAN_ID の定義域外。
        test::check(armctl::shoulder_can_id >= 1 && armctl::shoulder_can_id <= 127,
                    "肩の CAN_ID が 1..127");
        test::check(armctl::elbow_can_id >= 1 && armctl::elbow_can_id <= 127,
                    "肘の CAN_ID が 1..127");
    }

    test::section("8. 伝達比");
    {
        test::check(armctl::shoulder_ratio > 0.0f && armctl::elbow_ratio > 0.0f, "減速比が正");
        test::check(armctl::shoulder_direction == 1 || armctl::shoulder_direction == -1,
                    "肩の向きが ±1");
        test::check(armctl::elbow_direction == 1 || armctl::elbow_direction == -1,
                    "肘の向きが ±1");
    }

    test::section("8b. 過トルクの閾値は実測の上下に挟まれている");
    {
        // **無効 (0 以下) でも通る検査にする。** 測る前は無効が正しい状態で、
        // そこを落とすと「測るまで出荷できない」ことになってしまう。
        if (armctl::over_torque_nm > 0.0f) {
            // 下: 通常運転で出るトルクより上でなければ常時トリップする。
            // 上: 電流上限が作る天井より下でなければ一度も効かない。
            //     RS00 のトルク定数 14 N*m / 16 A。
            const float ceiling_nm = armctl::joint_current_limit_a * (14.0f / 16.0f);
            test::check(armctl::over_torque_nm < ceiling_nm,
                        "**電流上限の天井より下** (でなければ一度も効かない)");
            // 天井の半分は超えていること。低すぎる値を置くと通常運転で落ちる。
            test::check(armctl::over_torque_nm > ceiling_nm * 0.5f,
                        "天井の半分より上 (通常運転で落ちない側)");
        }
        test::check(armctl::over_torque_nm >= 0.0f, "負の閾値は意味を持たない");
    }

    test::section("9. 制御周期と通信途絶保護の噛み合わせ");
    {
        test::check(armctl::control_hz > 0, "制御周期が正");
        test::checkEq(static_cast<uint64_t>(armctl::control_period_us), 2000u,
                      "500 Hz なら 1 周期 2000 us");

        // **0 は「保護を切る」ではなく「モータ側の状態が分からない」。**
        // main.cpp は 0 のとき書き込みを丸ごと飛ばすので、前の使用者が入れた値が
        // そのまま残る。出荷時が 0 (= 保護なし) であることは実機で確認済み。
        test::check(armctl::can_timeout_ms != 0, "**通信途絶保護を切っていない**");

        // 下限: 制御周期のゆらぎで誤発火しない。10 周期ぶんは要る。
        const uint32_t period_ms = armctl::control_period_us / 1000u;
        test::check(armctl::can_timeout_ms >= period_ms * 10u,
                    "制御周期の 10 倍以上ある (ゆらぎで脱力しない)");

        // 上限: **モータ側の保護が、制御側の検出より先に効くこと。**
        //
        // 逆転すると、実際にはまだ通電しているのに制御側が先に
        // 「脱力した」と判定する。モータ側が先なら、実体が先に変わってから
        // 制御側がそれを見るので、表示と実体が一致する。
        //
        // can_timeout が効くのは制御ループ自体が止まったときだけで
        // (1 本でも送信が落ちれば commandJoints が即 DriveRejected へ倒す)、
        // その状況では制御側の判定はもう走っていない。**それでも順序を固定するのは、
        // 片方だけ変えたときに気づくため。**
        const uint32_t grace_ms = static_cast<uint32_t>(armctl::energized_grace_cycles) * period_ms;
        test::checkEq(static_cast<uint64_t>(grace_ms), 200u, "制御側の検出は 200 ms");
        test::check(armctl::can_timeout_ms < grace_ms,
                    "**モータ側の保護のほうが短い** (保持をやめて人に返すのが先)");
    }

    return test::report("arm_profile");
}

