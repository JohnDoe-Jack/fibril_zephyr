#include "arm/motion.hpp"

namespace arm {
namespace motion {
namespace {

// 終端の切り捨て閾値。IK の要求精度 0.01 mm より十分小さく取る。
constexpr float kPosEps = 1.0e-6f;  // mm    完全到達とみなす残距離
constexpr float kVelEps = 1.0e-4f;  // mm/s  停止とみなす速度

// 整定判定。停止距離を保守的に取っているぶん、終点の手前でいったん止まりきり、
// 低速で残りを詰めるクリープ区間が出る。速度が十分小さく残距離も 1 µm 以下に
// なった時点で到達とみなす。切り捨て量は最大 1 µm。
constexpr float kSettleDist = 1.0e-3f;  // mm
constexpr float kSettleVel  = 1.0e-1f;  // mm/s

bool profile_ok(const TrapezoidProfile& p) {
    return is_finite(p.v_max) && is_finite(p.a_max) && is_finite(p.jerk) && p.v_max > 0.0f &&
           p.a_max > 0.0f;
}

}  // namespace

// ---------------------------------------------------------------------------
// ScalarPlanner
// ---------------------------------------------------------------------------

bool ScalarPlanner::start(float length, const TrapezoidProfile& p) {
    reset();
    if (!profile_ok(p)) return false;
    pr_ = p;

    // NaN もここで弾かれる (NaN > x はいつでも偽)。
    if (!(length > kPosEps)) {
        len_ = (length > 0.0f) ? length : 0.0f;
        rem_ = 0.0f;
        st_  = State::Done;
        return length >= 0.0f;  // 長さ 0 は正常、負や NaN は失敗
    }

    len_ = length;
    rem_ = length;
    st_  = State::Running;
    return true;
}

void ScalarPlanner::pause() {
    if (st_ == State::Running) st_ = State::Paused;
}

void ScalarPlanner::resume() {
    if (st_ != State::Paused) return;
    st_    = State::Running;
    decel_ = false;  // 残距離しだいで再加速できるようラッチを解除する
}

void ScalarPlanner::abort() {
    if (st_ == State::Running || st_ == State::Paused) st_ = State::Aborted;
}

void ScalarPlanner::reset() {
    pr_    = TrapezoidProfile{};
    len_   = 0.0f;
    rem_   = 0.0f;
    v_     = 0.0f;
    a_     = 0.0f;
    decel_ = false;
    st_    = State::Idle;
}

// いまの (v, a) から止まるのに要る距離。
//
//   ve = v + 0.5·a·(a/j)         … a を 0 に戻すあいだに増える速度 (a > 0 のとき)
//   d  = ve²/(2·a_max)           … 減速本体
//      + ve·a_max/(2·jerk)       … 加速度を 0→−a、−a→0 に動かすぶん (S字のみ)
//      + ve·(a/j)                … a を 0 に戻すあいだに進む距離 (a > 0 のとき)
//      + ve·dt/2                 … 離散化ぶんの余裕 (1 周期)
//
// **加速度を反転させるあいだに進む距離**を忘れると、短い移動 (len = 20 mm,
// a_max = 2000, jerk = 20000 など) で減速がまったく間に合わない。
// **dt の項も要る。** 連続時間の式のままだと離散積分の誤差が終端付近で増幅し、
// 止まりきれずに終点を通過する。
float ScalarPlanner::stop_distance(float v, float a, float dt) const {
    if (v <= 0.0f) return 0.0f;

    float ve     = v;
    float d_ramp = 0.0f;
    if (pr_.jerk > 0.0f && a > 0.0f) {
        const float t_r = a / pr_.jerk;  // 加速度を 0 に戻すまでの時間
        ve += 0.5f * a * t_r;            // そのあいだに増える速度
        d_ramp = ve * t_r;               // そのあいだに進む距離 (上界)
    }

    float d = (ve * ve) / (2.0f * pr_.a_max) + 0.5f * ve * dt + d_ramp;
    if (pr_.jerk > 0.0f) d += (ve * pr_.a_max) / (2.0f * pr_.jerk);
    return d;
}

float ScalarPlanner::update(float dt) {
    if (!(dt > 0.0f)) return s();  // NaN もここで弾かれる
    if (st_ == State::Idle || st_ == State::Done) return s();

    // --- 目標速度 -----------------------------------------------------------
    float v_target;
    if (st_ == State::Paused || st_ == State::Aborted) {
        v_target = 0.0f;  // 減速して止まる。位置は連続のまま
    } else {
        // 保守的な停止距離のせいで手前で止まりきったら、ラッチを外して
        // 低速で残りを詰める。
        if (decel_ && v_ <= kVelEps && rem_ > kSettleDist) decel_ = false;
        if (rem_ <= stop_distance(v_, a_, dt)) decel_ = true;
        v_target = decel_ ? 0.0f : pr_.v_max;
    }

    // --- 加速度 -------------------------------------------------------------
    float a_des = 0.0f;
    if (pr_.jerk > 0.0f) {
        // ジャーク制限があると、加速度を 0 に戻すあいだにも速度は変わる。
        // その分 (a·|a| / 2j) を見越して切り替えないと目標速度を行き過ぎ、
        // 角が立って (= ジャークが無限大になって) S字にならない。
        const float dv    = v_target - v_;
        const float a_dec = (a_ * fabsf(a_)) / (2.0f * pr_.jerk);
        const float sw    = dv - a_dec;
        if (sw > 0.0f) {
            a_des = pr_.a_max;
        } else if (sw < 0.0f) {
            a_des = -pr_.a_max;
        }
        const float da_max = pr_.jerk * dt;
        a_ += clampf(a_des - a_, -da_max, da_max);
    } else {
        if (v_ < v_target) {
            a_des = pr_.a_max;
        } else if (v_ > v_target) {
            a_des = -pr_.a_max;
        }
        a_ = a_des;
    }

    // --- 積分 ---------------------------------------------------------------
    v_ += a_ * dt;
    if (pr_.jerk <= 0.0f) {  // 台形では目標速度を行き過ぎない
        if (a_ > 0.0f && v_ > v_target) v_ = v_target;
        if (a_ < 0.0f && v_ < v_target) v_ = v_target;
    }
    v_ = clampf(v_, 0.0f, pr_.v_max);

    rem_ -= v_ * dt;

    // --- 終端 ---------------------------------------------------------------
    if (st_ == State::Aborted) {
        if (v_ <= kVelEps) {
            v_ = 0.0f;
            a_ = 0.0f;
        }
        if (rem_ < 0.0f) rem_ = 0.0f;  // 経路の端より先へは進まない
        return s();
    }

    // ここに来る時点で v_ はほぼ 0。残差を切り捨てて始点・終点を厳密に一致させる。
    if (rem_ <= kPosEps || (rem_ <= kSettleDist && v_ <= kSettleVel)) {
        rem_ = 0.0f;
        v_   = 0.0f;
        a_   = 0.0f;
        st_  = State::Done;
    }
    return s();
}

// ---------------------------------------------------------------------------
// Jog
// ---------------------------------------------------------------------------

void Jog::set_limits(float v_max, float a_max) {
    if (is_finite(v_max) && v_max > 0.0f) v_max_ = v_max;
    if (is_finite(a_max) && a_max > 0.0f) a_max_ = a_max;
    set_velocity(cmd_);  // 既存の指令を新しい上限で掛け直す
}

void Jog::set_velocity(Velocity v) {
    if (!is_finite(v)) return;  // 壊れた指令は無視する。前の指令が残るほうが安全
    const float n = norm(v);
    if (n > v_max_ && n > 0.0f) {
        const float k = v_max_ / n;  // **向きを保ったまま**縮める
        v.vx *= k;
        v.vy *= k;
    }
    cmd_ = v;
}

Point Jog::update(float dt) {
    if (!(dt > 0.0f)) return pos_;

    // 指令速度への追従を加速度で制限する (急な指令変化を鈍らせる)。
    float       dvx  = cmd_.vx - vel_.vx;
    float       dvy  = cmd_.vy - vel_.vy;
    const float dn   = hypotf2(dvx, dvy);
    const float dmax = a_max_ * dt;
    if (dn > dmax && dn > 0.0f) {
        const float k = dmax / dn;
        dvx *= k;
        dvy *= k;
    }
    vel_.vx += dvx;
    vel_.vy += dvy;

    pos_.x += vel_.vx * dt;
    pos_.y += vel_.vy * dt;
    return pos_;
}

void Jog::set_position(Point p) {
    if (!is_finite(p)) return;
    pos_ = p;
    vel_ = Velocity{0.0f, 0.0f};
    cmd_ = Velocity{0.0f, 0.0f};
}

bool Jog::is_moving() const { return norm(vel_) > kVelEps || norm(cmd_) > kVelEps; }

// ---------------------------------------------------------------------------
// LinearMove
// ---------------------------------------------------------------------------

bool LinearMove::start(Point from, Point to, const TrapezoidProfile& p) {
    if (!is_finite(from) || !is_finite(to)) return false;

    from_ = from;
    to_   = to;

    const float dx  = to.x - from.x;
    const float dy  = to.y - from.y;
    const float len = hypotf2(dx, dy);

    if (len > 0.0f) {
        ux_ = dx / len;
        uy_ = dy / len;
    } else {
        ux_ = 1.0f;
        uy_ = 0.0f;
    }
    return pl_.start(len, p);
}

Point LinearMove::update(float dt) {
    pl_.update(dt);
    return position();
}

Point LinearMove::position() const {
    // 完了時は丸め誤差なく終点に一致させる。
    if (pl_.is_done()) return to_;
    const float s = pl_.s();
    return Point{from_.x + ux_ * s, from_.y + uy_ * s};
}

Velocity LinearMove::velocity() const {
    const float v = pl_.v();
    return Velocity{ux_ * v, uy_ * v};
}

// ---------------------------------------------------------------------------
// ArcMove
// ---------------------------------------------------------------------------

bool ArcMove::start(Point center, float radius, float ang_start, float ang_end,
                    const TrapezoidProfile& p) {
    if (!is_finite(center) || !is_finite(radius) || !is_finite(ang_start) || !is_finite(ang_end)) {
        return false;
    }
    if (!(radius > 0.0f)) return false;

    center_ = center;
    r_      = radius;
    a0_     = ang_start;
    a1_     = ang_end;

    const float sweep = ang_end - ang_start;
    dir_              = (sweep >= 0.0f) ? 1.0f : -1.0f;

    return pl_.start(fabsf(sweep) * radius, p);
}

Point ArcMove::update(float dt) {
    pl_.update(dt);
    return position();
}

float ArcMove::angle() const {
    // **未開始 / start 失敗のときは r_ = 0。** そのまま s()/r_ を計算すると
    // 0/0 = NaN になり、position() も velocity() も NaN を返す。
    // 下流の is_finite ガードが受け止めるので実害は「動かない」だが、
    // 戻り値を返す関数が NaN を作る理由はない。
    if (!(r_ > 0.0f)) return a0_;
    // 完了時は終端角へ吸着させる。s/r の割り算を経由すると丸めが残る。
    if (pl_.is_done()) return a1_;
    return a0_ + dir_ * (pl_.s() / r_);
}

Point ArcMove::position() const {
    const float a = angle();
    return Point{center_.x + r_ * cosf(a), center_.y + r_ * sinf(a)};
}

Velocity ArcMove::velocity() const {
    // 接線方向。r は経路長 → 角速度 → 速度で往復して約分されるので掛けない。
    const float a = angle();
    const float v = pl_.v() * dir_;
    return Velocity{-sinf(a) * v, cosf(a) * v};
}

}  // namespace motion
}  // namespace arm

