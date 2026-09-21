// ②' 伝達比の検証。
//
// この層の要点は 2 つ。**正規化しないこと**と、**オフセットを引いてから掛けること**。
// どちらも往復テストだけでは捕まらないので、片道の値を手計算で置いている。
//
// **リンクしているのは transmission.cpp 1 本だけ。** kinematics にも
// pico-sdk のスタブにも依存していないことの実演を兼ねている。

#include "test_util.hpp"

#include "arm/transmission.hpp"

namespace {

using arm::JointAngles;
using arm::JointVels;
namespace tx = arm::tx;

uint64_t code(tx::Status s) { return static_cast<uint64_t>(s); }

// 実機の値。肩は外部平歯車 1 段で反転、肘は平行リンクで直結。
constexpr tx::Config cfg_real{};

}  // namespace

int main() {
    test::section("9. to_motor → to_joint の往復 (誤差 < 1e-5 rad)");
    {
        tx::Config cfg = cfg_real;
        cfg.shoulder.offset = 0.35f;
        cfg.elbow.offset    = -0.20f;

        float max_err = 0.0f;
        bool  all_ok  = true;
        for (int i = -60; i <= 60; ++i) {
            const float t1 = static_cast<float>(i) * 0.1f;
            const float t2 = static_cast<float>(i) * -0.07f + 0.5f;

            tx::MotorAngles m{};
            all_ok = all_ok && (tx::to_motor(cfg, JointAngles{t1, t2}, m) == tx::Status::Ok);
            const JointAngles back = tx::to_joint(cfg, m);
            const float e1 = fabsf(back.theta1 - t1);
            const float e2 = fabsf(back.theta2 - t2);
            if (e1 > max_err) max_err = e1;
            if (e2 > max_err) max_err = e2;
        }
        test::check(all_ok, "全点で変換できる");
        test::checkNearF(max_err, 0.0f, 1e-5f, "往復誤差が 1e-5 rad 未満");
    }

    test::section("10. 減速比・方向・オフセットの適用と順序");
    {
        tx::Config cfg;
        cfg.shoulder = tx::JointTransmission{3.0f, -1, 0.2f};
        cfg.elbow    = tx::JointTransmission{1.0f, +1, 0.0f};

        tx::MotorAngles m{};
        test::checkEq(code(tx::to_motor(cfg, JointAngles{1.0f, 0.75f}, m)), code(tx::Status::Ok),
                      "変換できる");
        // (1.0 − 0.2) · 3 · (−1) = −2.4
        // 順序を間違えて 1.0·3·(−1) − 0.2 = −3.2 になっていないこと
        test::checkNearF(m.m1, -2.4f, 1e-6f, "オフセットを引いてから掛ける");
        test::checkNearF(m.m2, 0.75f, 1e-6f, "肘は直結なのでそのまま");

        // 減速比だけを変えたときの効き方
        cfg.shoulder.ratio = 5.0f;
        test::checkEq(code(tx::to_motor(cfg, JointAngles{1.0f, 0.0f}, m)), code(tx::Status::Ok),
                      "変換できる");
        test::checkNearF(m.m1, -4.0f, 1e-6f, "(1.0 − 0.2) · 5 · (−1)");

        // オフセットだけを変えたときの効き方
        cfg.shoulder.ratio  = 3.0f;
        cfg.shoulder.offset = 0.0f;
        test::checkEq(code(tx::to_motor(cfg, JointAngles{1.0f, 0.0f}, m)), code(tx::Status::Ok),
                      "変換できる");
        test::checkNearF(m.m1, -3.0f, 1e-6f, "オフセット 0 なら 1.0 · 3 · (−1)");
    }

    test::section("11. 0 → 4π を連続に動かすとモータ角は 12π まで単調増加する");
    {
        // **正規化していないことの検査。** 畳んでいればここで折り返す。
        tx::Config cfg;
        cfg.shoulder = tx::JointTransmission{3.0f, +1, 0.0f};

        float prev      = 0.0f;
        bool  monotonic = true;
        bool  all_ok    = true;
        float max_step  = 0.0f;
        const int steps = 4000;
        for (int i = 0; i <= steps; ++i) {
            const float t1 = 4.0f * arm::PI * static_cast<float>(i) / static_cast<float>(steps);
            tx::MotorAngles m{};
            all_ok = all_ok && (tx::to_motor(cfg, JointAngles{t1, 0.0f}, m) == tx::Status::Ok);
            if (i > 0) {
                if (m.m1 < prev) monotonic = false;
                const float step = fabsf(m.m1 - prev);
                if (step > max_step) max_step = step;
            }
            prev = m.m1;
        }
        test::check(all_ok, "全点で変換できる");
        test::check(monotonic, "単調増加している (折り返していない)");
        test::checkNearF(prev, 12.0f * arm::PI, 1e-3f, "終端は 12π");
        // 畳んでいれば 1 ステップで 2π 近く飛ぶ。実際の 1 ステップは 12π/4000 ≈ 0.0094。
        test::check(max_step < 0.02f, "1 ステップの跳びが小さい");
    }

    test::section("12. direction = −1 で符号が反転する");
    {
        tx::Config pos, neg;
        pos.shoulder = tx::JointTransmission{3.0f, +1, 0.0f};
        neg.shoulder = tx::JointTransmission{3.0f, -1, 0.0f};

        tx::MotorAngles mp{}, mn{};
        tx::to_motor(pos, JointAngles{0.7f, 0.0f}, mp);
        tx::to_motor(neg, JointAngles{0.7f, 0.0f}, mn);
        test::checkNearF(mp.m1, -mn.m1, 1e-6f, "角度の符号が反転する");

        const tx::MotorVels dp = tx::to_motor_vel(pos, JointVels{2.0f, 0.0f});
        const tx::MotorVels dn = tx::to_motor_vel(neg, JointVels{2.0f, 0.0f});
        test::checkNearF(dp.dm1, -dn.dm1, 1e-6f, "速度の符号も反転する");
        test::checkNearF(dp.dm1, 6.0f, 1e-6f, "2.0 rad/s · 3 = 6.0");

        // 往復すれば元に戻る
        test::checkNearF(tx::to_joint(neg, mn).theta1, 0.7f, 1e-6f, "負方向でも往復する");
    }

    test::section("13. この層は機構の可動限界を持たない");
    {
        // ケーブル巻取りのような限界はこの層の関心事ではない。肩は無限回転で、
        // 肘の制約は「死点を避ける肘角の範囲」として kinematics が持つ。
        // **何回転させても拒否されないこと**が、その設計の検査になる。
        tx::MotorAngles m{};
        bool            all_ok = true;
        for (int turns = -50; turns <= 50; ++turns) {
            const float t1 = static_cast<float>(turns) * arm::TWO_PI;
            all_ok = all_ok && (tx::to_motor(cfg_real, JointAngles{t1, 0.0f}, m) == tx::Status::Ok);
        }
        test::check(all_ok, "±50 回転でも拒否されない");
        test::checkNearF(m.m1, -50.0f * arm::TWO_PI * 3.0f, 1e-2f, "モータ角も畳まれていない");
    }

    test::section("1 関節ぶんのスカラ変換が本体");
    {
        // 腕の自由度を知らない形。関節が増えてもこの層は書き換えずに済む。
        const tx::JointTransmission t{3.0f, -1, 0.2f};

        float m = 9.0f;
        test::checkEq(code(tx::to_motor(t, 1.0f, m)), code(tx::Status::Ok), "変換できる");
        test::checkNearF(m, -2.4f, 1e-6f, "(1.0 − 0.2) · 3 · (−1)");
        test::checkNearF(tx::to_joint(t, m), 1.0f, 1e-6f, "往復する");
        test::checkNearF(tx::to_motor_vel(t, 2.0f), -6.0f, 1e-6f, "速度にオフセットは効かない");
        test::checkNearF(tx::to_joint_vel(t, -6.0f), 2.0f, 1e-6f, "速度も往復する");

        // 2 関節版はこれを 2 回呼ぶだけ
        tx::Config      cfg;
        cfg.shoulder = t;
        cfg.elbow    = t;
        tx::MotorAngles pair{};
        test::checkEq(code(tx::to_motor(cfg, JointAngles{1.0f, 1.0f}, pair)), code(tx::Status::Ok),
                      "2 関節版も同じ");
        test::checkNearF(pair.m1, -2.4f, 1e-6f, "肩");
        test::checkNearF(pair.m2, -2.4f, 1e-6f, "肘");

        // **片方だけ書き込まない。** 肘が失敗したら肩も書かない。
        tx::Config half;
        half.shoulder = t;
        half.elbow    = tx::JointTransmission{0.0f, +1, 0.0f};  // 使えない係数
        tx::MotorAngles keep{7.0f, 8.0f};
        test::checkEq(code(tx::to_motor(half, JointAngles{1.0f, 1.0f}, keep)),
                      code(tx::Status::BadConfig), "片方が壊れていれば全体が失敗");
        test::checkNearF(keep.m1, 7.0f, 1e-6f, "肩も書き換えない");
        test::checkNearF(keep.m2, 8.0f, 1e-6f, "肘も書き換えない");
    }

    test::section("14. 速度変換の往復とオフセット非依存");
    {
        tx::Config cfg = cfg_real;
        cfg.shoulder.offset = 1.234f;  // 速度には効かないはず
        cfg.elbow.offset    = -0.5f;

        float max_err = 0.0f;
        for (int i = -50; i <= 50; ++i) {
            const JointVels dq{static_cast<float>(i) * 0.3f, static_cast<float>(i) * -0.11f};
            const JointVels back = tx::to_joint_vel(cfg, tx::to_motor_vel(cfg, dq));
            const float e1 = fabsf(back.dq1 - dq.dq1);
            const float e2 = fabsf(back.dq2 - dq.dq2);
            if (e1 > max_err) max_err = e1;
            if (e2 > max_err) max_err = e2;
        }
        test::checkNearF(max_err, 0.0f, 1e-5f, "速度の往復誤差が 1e-5 rad/s 未満");

        // オフセットを変えても速度変換の結果が変わらないこと
        tx::Config zero_off = cfg_real;
        const tx::MotorVels a = tx::to_motor_vel(cfg, JointVels{1.0f, 2.0f});
        const tx::MotorVels b = tx::to_motor_vel(zero_off, JointVels{1.0f, 2.0f});
        test::checkNearF(a.dm1, b.dm1, 1e-6f, "オフセットは速度に効かない (肩)");
        test::checkNearF(a.dm2, b.dm2, 1e-6f, "オフセットは速度に効かない (肘)");
        test::checkNearF(a.dm1, -3.0f, 1e-6f, "1.0 rad/s · 3 · (−1)");

        // 速度が 0 なら 0。オフセットが漏れていればここがずれる。
        const tx::MotorVels z = tx::to_motor_vel(cfg, JointVels{0.0f, 0.0f});
        test::checkNearF(z.dm1, 0.0f, 1e-9f, "0 は 0 のまま");
    }

    test::section("ratio = 0 で NaN を撒かない");
    {
        tx::Config bad;
        bad.shoulder = tx::JointTransmission{0.0f, +1, 0.1f};

        test::checkEq(code(tx::validate(bad)), code(tx::Status::BadConfig),
                      "設定検査が落とす (起動時に気づける)");

        tx::MotorAngles m{5.0f, 6.0f};
        test::checkEq(code(tx::to_motor(bad, JointAngles{1.0f, 1.0f}, m)),
                      code(tx::Status::BadConfig), "黙って 0 を指令せず BadConfig を返す");
        test::checkNearF(m.m1, 5.0f, 1e-6f, "出力を書き換えない");

        // 読み出し方向は有限値を返す (モータ 0 に対応する関節角 = オフセット)
        const JointAngles q = tx::to_joint(bad, tx::MotorAngles{9.0f, 9.0f});
        test::check(arm::is_finite(q), "to_joint が NaN を返さない");
        test::checkNearF(q.theta1, 0.1f, 1e-6f, "モータ 0 に対応する関節角 = オフセット");

        const JointVels dq = tx::to_joint_vel(bad, tx::MotorVels{9.0f, 9.0f});
        test::check(arm::is_finite(dq), "to_joint_vel が NaN を返さない");
        test::checkNearF(dq.dq1, 0.0f, 1e-9f, "速度は 0");

        // direction = 0 も同じ扱い
        tx::Config zero_dir;
        zero_dir.shoulder = tx::JointTransmission{3.0f, 0, 0.0f};
        test::checkEq(code(tx::validate(zero_dir)), code(tx::Status::BadConfig),
                      "direction = 0 も設定ミス");
    }

    test::section("設定の検査と NaN の拒否");
    {
        test::checkEq(code(tx::validate(cfg_real)), code(tx::Status::Ok), "実機の設定は妥当");

        const float nan_v = sqrtf(-1.0f);

        tx::Config nan_cfg;
        nan_cfg.elbow.offset = nan_v;
        test::checkEq(code(tx::validate(nan_cfg)), code(tx::Status::NotFinite),
                      "NaN のオフセットを弾く");
        test::check(!tx::usable(nan_cfg.elbow), "usable が false");

        tx::MotorAngles m{1.0f, 2.0f};
        test::checkEq(code(tx::to_motor(cfg_real, JointAngles{nan_v, 0.0f}, m)),
                      code(tx::Status::NotFinite), "NaN の関節角を拒否する");
        test::checkNearF(m.m1, 1.0f, 1e-6f, "出力を書き換えない");

        // 読み出し方向は NaN を返さない (モータ 0 に対応する値へ落とす)
        test::check(arm::is_finite(tx::to_joint(cfg_real, tx::MotorAngles{nan_v, nan_v})),
                    "NaN のモータ角を読んでも NaN を返さない");
        test::check(arm::is_finite(tx::to_joint_vel(cfg_real, tx::MotorVels{nan_v, nan_v})),
                    "速度も同じ");
    }

    return test::report("arm_transmission");
}

