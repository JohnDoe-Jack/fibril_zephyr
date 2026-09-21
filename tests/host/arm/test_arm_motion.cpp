// ④ 軌道生成の検証。
//
// この層は罠が集中している。**終端で止まりきること**が一番むずかしく、
// 長い台形 / 短い三角形 / 長い S字 / 短い S字 の 4 通りで確かめる。
// 短い S字 (len = 20 mm, a_max = 2000, jerk = 20000) は、停止距離に
// 「加速度を 0 に戻すあいだに進む距離」を入れ忘れると必ず落ちる形。
//
// **リンクしているのは motion.cpp 1 本だけ。**

#include "test_util.hpp"

#include "arm/motion.hpp"

namespace {

using arm::Point;
using arm::Velocity;
namespace mo = arm::motion;

constexpr float kDt = 0.001f;  // 1 kHz

}  // namespace

int main() {
    test::section("18. 直線補間の始点・終点と、経路が直線から外れないこと");
    {
        mo::LinearMove         m;
        const Point            from{100.0f, 200.0f};
        const Point            to{700.0f, -150.0f};
        const mo::TrapezoidProfile p{300.0f, 1200.0f, 0.0f};

        test::check(m.start(from, to, p), "開始できる");
        test::checkNearF(m.position().x, from.x, 1e-4f, "始点 x");
        test::checkNearF(m.position().y, from.y, 1e-4f, "始点 y");
        test::checkNearF(m.progress(), 0.0f, 1e-6f, "進捗 0");

        const float dx = to.x - from.x, dy = to.y - from.y;
        const float len = arm::hypotf2(dx, dy);
        const float nx = -dy / len, ny = dx / len;  // 経路の法線
        float       max_off = 0.0f;

        int steps = 0;
        while (!m.is_done() && steps < 100000) {
            const Point q   = m.update(kDt);
            const float off = fabsf((q.x - from.x) * nx + (q.y - from.y) * ny);
            if (off > max_off) max_off = off;
            ++steps;
        }
        test::check(m.is_done(), "完了する");
        test::check(max_off < 1e-3f, "経路が直線から外れない");
        test::checkNearF(m.position().x, to.x, 1e-4f, "終点 x");
        test::checkNearF(m.position().y, to.y, 1e-4f, "終点 y");
        test::checkNearF(m.progress(), 1.0f, 1e-6f, "進捗 1");

        mo::LinearMove z;
        test::check(z.start(from, from, p), "長さ 0 の移動も成功扱い");
        test::check(z.is_done(), "即完了");
        test::checkNearF(z.position().x, from.x, 1e-6f, "位置は始点のまま");
    }

    test::section("19. 速度・加速度・ジャークが上限を超えない (4 プロファイル)");
    {
        struct Case {
            float       len, v_max, a_max, jerk;
            const char* name;
        };
        const Case cases[] = {
            {1000.0f, 300.0f, 1200.0f, 0.0f, "長い台形 (v_max に到達する)"},
            {10.0f, 300.0f, 1200.0f, 0.0f, "短い三角形 (v_max に届かない)"},
            {1000.0f, 300.0f, 1200.0f, 12000.0f, "長い S字"},
            {20.0f, 500.0f, 2000.0f, 20000.0f, "短い S字 (停止距離の罠が出る形)"},
        };

        for (const Case& c : cases) {
            mo::ScalarPlanner pl;
            test::check(pl.start(c.len, mo::TrapezoidProfile{c.v_max, c.a_max, c.jerk}), c.name);

            float prev_v = 0.0f, prev_a = 0.0f, prev_s = -1.0f;
            float max_v = 0.0f, max_a = 0.0f, max_j = 0.0f;
            float residual_v = 0.0f;
            bool  monotonic  = true;
            int   steps      = 0;

            while (steps < 200000) {
                const float v_before = pl.v();
                pl.update(kDt);
                if (pl.is_done()) {
                    // 完了ステップは残速度を 0 へ切り捨てるので上限の計測から除く。
                    // 残速度そのものが小さいことは下で別に確かめる。
                    residual_v = v_before;
                    break;
                }
                const float v = pl.v();
                const float a = (v - prev_v) / kDt;
                const float j = (a - prev_a) / kDt;

                if (v > max_v) max_v = v;
                if (fabsf(a) > max_a) max_a = fabsf(a);
                // 整定クリープ域 (v が v_max の 1 % 未満) はエンベロープ追従なので
                // ジャークの計測から除く。速度自体が無視できる大きさ。
                if (steps > 0 && v > 0.01f * c.v_max && fabsf(j) > max_j) max_j = fabsf(j);
                if (pl.s() < prev_s) monotonic = false;

                prev_v = v;
                prev_a = a;
                prev_s = pl.s();
                ++steps;
            }

            test::check(pl.is_done(), "完了する (終端で止まりきる)");
            test::check(steps < 200000, "現実的な周期数で終わる");
            test::check(monotonic, "進捗が後戻りしない");
            test::checkNearF(pl.s(), c.len, 1e-3f, "終端の経路長が一致する");
            test::check(max_v <= c.v_max * 1.001f + 1e-3f, "速度が上限以下");
            test::check(max_a <= c.a_max * 1.001f + 1e-3f, "加速度が上限以下");
            if (c.jerk > 0.0f) test::check(max_j <= c.jerk * 1.05f + 1.0f, "ジャークが上限以下");
            // 完了時に残る速度は減速 2 周期ぶん = 2·a_max·dt 以下。
            // 位置に直すと 2·a_max·dt² (1 kHz / 1200 mm/s² で 2.4 µm)。
            test::check(residual_v <= 2.5f * c.a_max * kDt + 1e-3f, "終端の残速度が十分小さい");
        }

        // 長い移動では実際に v_max まで出ること (減速しすぎていない)
        mo::ScalarPlanner pl;
        pl.start(1000.0f, mo::TrapezoidProfile{300.0f, 1200.0f, 0.0f});
        float peak = 0.0f;
        for (int i = 0; i < 200000 && !pl.is_done(); ++i) {
            pl.update(kDt);
            if (pl.v() > peak) peak = pl.v();
        }
        test::checkNearF(peak, 300.0f, 1.0f, "巡航速度に達している");
    }

    test::section("20. ジョグの速度クランプ (向きが保たれること)");
    {
        mo::Jog j;
        j.set_limits(150.0f, 600.0f);
        j.set_position(Point{0.0f, 0.0f});
        j.set_velocity(Velocity{3000.0f, 4000.0f});  // 大きさ 5000、3:4 の向き

        float    max_v = 0.0f, max_a = 0.0f;
        Velocity prev{0.0f, 0.0f};
        for (int i = 0; i < 2000; ++i) {
            j.update(kDt);
            const Velocity v   = j.velocity();
            const float    sp  = arm::norm(v);
            const float    acc = arm::hypotf2(v.vx - prev.vx, v.vy - prev.vy) / kDt;
            if (sp > max_v) max_v = sp;
            if (acc > max_a) max_a = acc;
            prev = v;
        }
        test::check(max_v <= 150.0f * 1.001f, "速度が上限以下");
        test::check(max_a <= 600.0f * 1.001f, "加速度が上限以下");

        const Velocity v = j.velocity();
        test::checkNearF(arm::norm(v), 150.0f, 0.1f, "上限まで出る");
        test::checkNearF(v.vx / arm::norm(v), 0.6f, 1e-3f, "向きが保たれている (x)");
        test::checkNearF(v.vy / arm::norm(v), 0.8f, 1e-3f, "向きが保たれている (y)");

        // 指令を切っても加速度制限で急停止しない
        j.set_velocity(Velocity{0.0f, 0.0f});
        j.update(kDt);
        test::check(arm::norm(j.velocity()) > 100.0f, "1 周期では止まらない");

        // 上限を後から下げても掛け直される
        mo::Jog k;
        k.set_limits(200.0f, 100000.0f);
        k.set_velocity(Velocity{200.0f, 0.0f});
        k.set_limits(50.0f, 100000.0f);
        for (int i = 0; i < 100; ++i) k.update(kDt);
        test::checkNearF(arm::norm(k.velocity()), 50.0f, 0.5f, "新しい上限まで落ちる");

        k.set_position(Point{123.0f, -45.0f});
        test::checkNearF(k.position().x, 123.0f, 1e-6f, "set_position で同期できる");
        test::checkNearF(arm::norm(k.velocity()), 0.0f, 1e-6f, "速度もクリアされる");

        // 壊れた指令は無視して前の指令を残す
        mo::Jog n;
        n.set_limits(100.0f, 100000.0f);
        n.set_velocity(Velocity{100.0f, 0.0f});
        for (int i = 0; i < 100; ++i) n.update(kDt);
        n.set_velocity(Velocity{sqrtf(-1.0f), 0.0f});
        for (int i = 0; i < 10; ++i) n.update(kDt);
        test::check(arm::is_finite(n.position()), "NaN の指令で位置が壊れない");
        test::checkNearF(arm::norm(n.velocity()), 100.0f, 0.5f, "前の指令が残る");
    }

    test::section("21. 一時停止・再開で位置が飛ばない / abort は減速して止まる");
    {
        mo::LinearMove m;
        const Point    from{0.0f, 0.0f}, to{1000.0f, 0.0f};
        test::check(m.start(from, to, mo::TrapezoidProfile{300.0f, 1200.0f, 0.0f}), "開始");

        Point prev     = m.position();
        float max_jump = 0.0f;
        int   steps    = 0;
        bool  paused_stopped = false;

        while (!m.is_done() && steps < 100000) {
            if (steps == 500) m.pause();
            if (steps == 1500) m.resume();

            const Point q    = m.update(kDt);
            const float jump = arm::dist(q, prev);
            if (jump > max_jump) max_jump = jump;
            prev = q;
            ++steps;

            if (steps == 1000) {
                paused_stopped =
                    (m.state() == mo::State::Paused) && (fabsf(m.velocity().vx) < 1e-3f);
            }
        }
        test::check(m.is_done(), "再開後に完了する");
        test::check(paused_stopped, "一時停止中は完全に止まっている");
        // 1 周期の移動は v_max·dt = 0.3 mm を超えない = 位置が飛んでいない
        test::check(max_jump <= 0.3f * 1.01f, "位置が飛ばない");
        test::checkNearF(m.position().x, to.x, 1e-4f, "終点は正確");

        // S字でも同じこと
        mo::LinearMove sm;
        sm.start(from, to, mo::TrapezoidProfile{300.0f, 1200.0f, 12000.0f});
        Point sp_prev  = sm.position();
        float s_jump   = 0.0f;
        int   s_steps  = 0;
        while (!sm.is_done() && s_steps < 100000) {
            if (s_steps == 700) sm.pause();
            if (s_steps == 1400) sm.resume();
            const Point q = sm.update(kDt);
            const float d = arm::dist(q, sp_prev);
            if (d > s_jump) s_jump = d;
            sp_prev = q;
            ++s_steps;
        }
        test::check(sm.is_done(), "S字でも再開後に完了する");
        test::check(s_jump <= 0.3f * 1.01f, "S字でも位置が飛ばない");

        // abort: 減速して止まり、そのあとは動かない
        mo::LinearMove b;
        b.start(from, to, mo::TrapezoidProfile{300.0f, 1200.0f, 0.0f});
        for (int i = 0; i < 500; ++i) b.update(kDt);
        b.abort();
        Point p_prev  = b.position();
        float max_j2  = 0.0f;
        for (int i = 0; i < 2000; ++i) {
            const Point q = b.update(kDt);
            const float d = arm::dist(q, p_prev);
            if (d > max_j2) max_j2 = d;
            p_prev = q;
        }
        test::check(b.state() == mo::State::Aborted, "Aborted のまま");
        test::check(!b.is_done(), "完了扱いにはならない");
        test::check(max_j2 <= 0.3f * 1.01f, "急停止しない (位置が飛ばない)");
        test::checkNearF(b.velocity().vx, 0.0f, 1e-3f, "止まっている");
        test::check(b.position().x < to.x, "終点まで行かずに止まる");

        // 止まったあとは動かない
        const Point rest = b.position();
        for (int i = 0; i < 500; ++i) b.update(kDt);
        test::checkNearF(b.position().x, rest.x, 1e-6f, "止まったあとは動かない");
    }

    test::section("円弧補間");
    {
        mo::ArcMove a;
        const Point c{0.0f, 0.0f};
        const float r = 500.0f;
        test::check(a.start(c, r, 0.0f, arm::PI, mo::TrapezoidProfile{300.0f, 1200.0f, 0.0f}),
                    "開始できる");
        test::checkNearF(a.position().x, 500.0f, 1e-3f, "始点 x");
        test::checkNearF(a.position().y, 0.0f, 1e-3f, "始点 y");

        float max_r_err = 0.0f;
        int   steps     = 0;
        while (!a.is_done() && steps < 100000) {
            const Point q   = a.update(kDt);
            const float err = fabsf(arm::hypotf2(q.x - c.x, q.y - c.y) - r);
            if (err > max_r_err) max_r_err = err;
            ++steps;
        }
        test::check(a.is_done(), "完了する");
        test::check(max_r_err < 1e-2f, "半径から外れない");
        test::checkNearF(a.position().x, -500.0f, 1e-3f, "終点 x (吸着している)");
        test::checkNearF(a.position().y, 0.0f, 1e-3f, "終点 y");

        mo::ArcMove b;
        b.start(c, r, 0.0f, -0.5f * arm::PI, mo::TrapezoidProfile{300.0f, 1200.0f, 0.0f});
        for (int i = 0; i < 100000 && !b.is_done(); ++i) b.update(kDt);
        test::checkNearF(b.position().x, 0.0f, 1e-3f, "逆回りの終点 x");
        test::checkNearF(b.position().y, -500.0f, 1e-3f, "逆回りの終点 y");

        mo::ArcMove z;
        // **未開始でも NaN を返さない。** r_ = 0 のまま s()/r_ を計算していた頃は
        // position() も velocity() も (nan, nan) を返していた。
        test::check(arm::is_finite(z.position()), "未開始の位置が有限");
        test::check(arm::is_finite(z.velocity()), "未開始の速度が有限");

        test::check(!z.start(c, 0.0f, 0.0f, arm::PI, mo::TrapezoidProfile{300.0f, 1200.0f, 0.0f}),
                    "半径 0 は拒否");
        test::check(arm::is_finite(z.position()), "拒否したあとも位置が有限");
        test::check(!z.start(c, sqrtf(-1.0f), 0.0f, arm::PI,
                             mo::TrapezoidProfile{300.0f, 1200.0f, 0.0f}),
                    "NaN の半径も拒否");
        test::check(arm::is_finite(z.position()), "拒否したあとも位置が有限");
    }

    test::section("可変 dt でも終点に到達する");
    {
        mo::LinearMove m;
        m.start(Point{0.0f, 0.0f}, Point{500.0f, 0.0f},
                mo::TrapezoidProfile{300.0f, 1200.0f, 0.0f});

        unsigned seed  = 12345u;
        int      steps = 0;
        while (!m.is_done() && steps < 100000) {
            seed = seed * 1103515245u + 12345u;
            const float dt =
                0.0005f + 0.001f * static_cast<float>((seed >> 16) & 0xFFu) / 255.0f;
            m.update(dt);
            ++steps;
        }
        test::check(m.is_done(), "完了する");
        test::checkNearF(m.position().x, 500.0f, 1e-4f, "終点に一致する");

        // dt が 0 や負なら何もしない
        mo::LinearMove n;
        n.start(Point{0.0f, 0.0f}, Point{100.0f, 0.0f},
                mo::TrapezoidProfile{300.0f, 1200.0f, 0.0f});
        n.update(0.0f);
        n.update(-1.0f);
        n.update(sqrtf(-1.0f));
        test::checkNearF(n.position().x, 0.0f, 1e-6f, "不正な dt では動かない");
    }

    test::section("成立していないプロファイルは拒否する");
    {
        mo::ScalarPlanner pl;
        test::check(!pl.start(100.0f, mo::TrapezoidProfile{0.0f, 1000.0f, 0.0f}), "v_max = 0");
        test::check(!pl.start(100.0f, mo::TrapezoidProfile{100.0f, 0.0f, 0.0f}), "a_max = 0");
        test::check(!pl.start(100.0f, mo::TrapezoidProfile{sqrtf(-1.0f), 1000.0f, 0.0f}),
                    "NaN の v_max");
        test::check(!pl.start(-5.0f, mo::TrapezoidProfile{100.0f, 1000.0f, 0.0f}), "負の長さ");
        test::check(!pl.start(sqrtf(-1.0f), mo::TrapezoidProfile{100.0f, 1000.0f, 0.0f}),
                    "NaN の長さ");
        test::check(pl.start(0.0f, mo::TrapezoidProfile{100.0f, 1000.0f, 0.0f}), "長さ 0 は成功");
        test::check(pl.is_done(), "長さ 0 は即完了");
        test::checkNearF(pl.progress(), 1.0f, 1e-6f, "進捗は 1");
    }

    return test::report("arm_motion");
}

