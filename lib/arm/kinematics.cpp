#include "arm/kinematics.hpp"

namespace arm {
namespace kin {
namespace {

// 目標点から |e| を求める。
//
// **acosf のクランプが到達判定を兼ねる。** 引数は丸め誤差だけでなく
// 「到達不能な目標点」でも ±1 を超えるので、クランプした結果:
//
//   外側限界より遠い → c > 1  → |e| = 0  → TooExtended
//   内側限界より近い → c < −1 → |e| = π  → TooFolded
//
// と、到達不能ケースが自然に正しい側の Status へ落ちる。原点 (d = 0) も
// この経路を通るので、ゼロ除算にも atan2f(0,0) にも触れない。分岐が要らない。
//
// 座標が巨大で d2 が ∞ になっても c = ∞ → クランプ → |e| = 0 で正しく落ちる。
// NaN だけはクランプを素通りするので、呼び出し側が先に弾いている。
inline float elbow_abs_from_point(const Config& cfg, Point p) {
    const float d2 = p.x * p.x + p.y * p.y;
    float       c  = (d2 - cfg.l1 * cfg.l1 - cfg.l2 * cfg.l2) / (2.0f * cfg.l1 * cfg.l2);
    c              = clampf(c, -1.0f, 1.0f);
    return acosf(c);  // 0 .. π
}

inline Status classify(const Config& cfg, float e_abs) {
    if (e_abs < cfg.elbow_min) return Status::TooExtended;
    if (e_abs > cfg.elbow_max) return Status::TooFolded;
    return Status::Ok;
}

// 肘角から肩-手先距離。d² = l1² + l2² + 2·l1·l2·cos e
inline float reach_at(const Config& cfg, float e) {
    const float d2 = cfg.l1 * cfg.l1 + cfg.l2 * cfg.l2 + 2.0f * cfg.l1 * cfg.l2 * cosf(e);
    return (d2 > 0.0f) ? sqrtf(d2) : 0.0f;
}

// 制御周期ごとに払っても惜しくない最小の設定検査。
// これを通れば 2·l1·l2 > 0 なので、有限な入力から NaN は生まれない。
inline bool links_positive(const Config& cfg) { return cfg.l1 > 0.0f && cfg.l2 > 0.0f; }

}  // namespace

Status validate(const Config& cfg) {
    if (!is_finite(cfg.l1) || !is_finite(cfg.l2)) return Status::NotFinite;
    if (!is_finite(cfg.elbow_min) || !is_finite(cfg.elbow_max)) return Status::NotFinite;
    if (!is_finite(cfg.det_eps)) return Status::NotFinite;
    if (!links_positive(cfg)) return Status::BadConfig;
    if (cfg.elbow_min < 0.0f || cfg.elbow_max > PI) return Status::BadConfig;
    if (cfg.elbow_min > cfg.elbow_max) return Status::BadConfig;
    if (cfg.det_eps < 0.0f) return Status::BadConfig;
    return Status::Ok;
}

Point forward(const Config& cfg, JointAngles q) {
    Point p;
    p.x = cfg.l1 * cosf(q.theta1) + cfg.l2 * cosf(q.theta2);
    p.y = cfg.l1 * sinf(q.theta1) + cfg.l2 * sinf(q.theta2);
    return p;
}

Status check_reachable(const Config& cfg, Point target) {
    if (!is_finite(target)) return Status::NotFinite;
    if (!links_positive(cfg)) return Status::BadConfig;
    return classify(cfg, elbow_abs_from_point(cfg, target));
}

Status inverse(const Config& cfg, Point target, JointAngles& out) {
    if (!is_finite(target)) return Status::NotFinite;
    if (!links_positive(cfg)) return Status::BadConfig;

    const float  e_abs = elbow_abs_from_point(cfg, target);
    const Status st    = classify(cfg, e_abs);
    if (st != Status::Ok) return st;

    const float e = e_abs * static_cast<float>(static_cast<int8_t>(cfg.elbow_side));

    // ここまで来れば d は [reach_inner, reach_outer] の内側にある。
    // 仮に elbow_max = π まで許して原点が入っても atan2f(0,0) は 0 を返す
    // (未定義動作ではない) ので、専用の分岐は置かない。
    const float phi = atan2f(target.y, target.x);
    const float psi = atan2f(cfg.l2 * sinf(e), cfg.l1 + cfg.l2 * cosf(e));

    out.theta1 = normalize_angle(phi - psi);
    out.theta2 = normalize_angle(out.theta1 + e);  // **θ2 は絶対角**
    return Status::Ok;
}

Status check_joints(const Config& cfg, JointAngles q) {
    if (!is_finite(q)) return Status::NotFinite;
    if (!links_positive(cfg)) return Status::BadConfig;

    const float e = normalize_angle(q.theta2 - q.theta1);

    // **向きが先。** 範囲だけ見ると、逆側に折れた鏡像の姿勢を通してしまう。
    // 組み立てで片側に固定されているので、そこへは物理的に行けない。
    const bool want_positive = (cfg.elbow_side == ElbowSide::Positive);
    if ((e > 0.0f) != want_positive) return Status::WrongElbowSide;

    const float e_abs = fabsf(e);
    if (e_abs < cfg.elbow_min) return Status::TooExtended;
    if (e_abs > cfg.elbow_max) return Status::TooFolded;
    return Status::Ok;
}

float reach_outer(const Config& cfg) { return reach_at(cfg, cfg.elbow_min); }
float reach_inner(const Config& cfg) { return reach_at(cfg, cfg.elbow_max); }

void jacobian(const Config& cfg, JointAngles q, float J[2][2]) {
    const float s1 = sinf(q.theta1), c1 = cosf(q.theta1);
    const float s2 = sinf(q.theta2), c2 = cosf(q.theta2);
    J[0][0] = -cfg.l1 * s1;
    J[0][1] = -cfg.l2 * s2;
    J[1][0] = cfg.l1 * c1;
    J[1][1] = cfg.l2 * c2;
}

float det_jacobian(const Config& cfg, JointAngles q) {
    // det = (−l1·s1)(l2·c2) − (−l2·s2)(l1·c1) = l1·l2·sin(θ2 − θ1)
    return cfg.l1 * cfg.l2 * sinf(q.theta2 - q.theta1);
}

float manipulability(const Config& cfg, JointAngles q) { return fabsf(det_jacobian(cfg, q)); }

float normalized_manipulability(const Config& cfg, JointAngles q) {
    (void)cfg;  // |det J|/(l1·l2) = |sin e|。リンク長は約分されて消える
    return fabsf(sinf(q.theta2 - q.theta1));
}

Status inverse_velocity(const Config& cfg, JointAngles q, Velocity v, JointVels& out) {
    if (!is_finite(q) || !is_finite(v)) return Status::NotFinite;
    if (!links_positive(cfg)) return Status::BadConfig;

    // **normalized_manipulability() と同じ量。** |det J|/(l1·l2) = |sin e| で、
    // リンク長に依らない無次元量。ここで関数を呼び直さないのは sinf を
    // 2 度払わないため — 下でも同じ s_e を使う。
    const float s_e = sinf(q.theta2 - q.theta1);
    if (fabsf(s_e) < cfg.det_eps) return Status::Singular;

    const float s1 = sinf(q.theta1), c1 = cosf(q.theta1);
    const float s2 = sinf(q.theta2), c2 = cosf(q.theta2);

    // J⁻¹ = 1/det · [[ l2·c2,  l2·s2],
    //                [−l1·c1, −l1·s1]]     det = l1·l2·sin e
    // → l1, l2 が約分されるので、除算は 2 回で済む。
    const float inv1 = 1.0f / (cfg.l1 * s_e);
    const float inv2 = 1.0f / (cfg.l2 * s_e);

    out.dq1 = (c2 * v.vx + s2 * v.vy) * inv1;
    out.dq2 = -(c1 * v.vx + s1 * v.vy) * inv2;
    return Status::Ok;
}

Velocity forward_velocity(const Config& cfg, JointAngles q, JointVels dq) {
    const float s1 = sinf(q.theta1), c1 = cosf(q.theta1);
    const float s2 = sinf(q.theta2), c2 = cosf(q.theta2);
    Velocity    v;
    v.vx = -cfg.l1 * s1 * dq.dq1 - cfg.l2 * s2 * dq.dq2;
    v.vy = cfg.l1 * c1 * dq.dq1 + cfg.l2 * c2 * dq.dq2;
    return v;
}

}  // namespace kin
}  // namespace arm

