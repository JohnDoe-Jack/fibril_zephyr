#include "arm/transmission.hpp"

namespace arm {
namespace tx {
namespace {

// 関節角からモータ角への一次係数。**この 1 つに減速比と向きが畳まれている。**
inline float gain(const JointTransmission& t) {
    return t.ratio * static_cast<float>(t.direction);
}

}  // namespace

bool usable(const JointTransmission& t) {
    const float g = gain(t);
    return is_finite(g) && g != 0.0f && is_finite(t.offset);
}

Status validate(const Config& cfg) {
    const JointTransmission* joints[2] = {&cfg.shoulder, &cfg.elbow};
    for (const JointTransmission* t : joints) {
        if (!is_finite(t->ratio) || !is_finite(t->offset)) return Status::NotFinite;
        if (!usable(*t)) return Status::BadConfig;
    }
    return Status::Ok;
}

// --- 1 関節ぶん -------------------------------------------------------------

Status to_motor(const JointTransmission& t, float joint_rad, float& out) {
    if (!is_finite(joint_rad)) return Status::NotFinite;
    if (!usable(t)) return Status::BadConfig;
    // **正規化しないこと。** ここで −π〜+π へ畳むとモータ角の連続性が失われる。
    out = (joint_rad - t.offset) * gain(t);
    return Status::Ok;
}

float to_joint(const JointTransmission& t, float motor_rad) {
    if (!usable(t) || !is_finite(motor_rad)) return t.offset;
    return motor_rad / gain(t) + t.offset;
}

float to_motor_vel(const JointTransmission& t, float joint_vel) {
    // **オフセットは効かない。** 定数の微分は 0。
    if (!usable(t) || !is_finite(joint_vel)) return 0.0f;
    return joint_vel * gain(t);
}

float to_joint_vel(const JointTransmission& t, float motor_vel) {
    if (!usable(t) || !is_finite(motor_vel)) return 0.0f;
    return motor_vel / gain(t);
}

// --- 2 関節ぶん -------------------------------------------------------------

Status to_motor(const Config& cfg, JointAngles q, MotorAngles& out) {
    float m1 = 0.0f;
    float m2 = 0.0f;
    // 片方だけ書き込まないよう、両方そろってから out を触る。
    const Status s1 = to_motor(cfg.shoulder, q.theta1, m1);
    if (s1 != Status::Ok) return s1;
    const Status s2 = to_motor(cfg.elbow, q.theta2, m2);
    if (s2 != Status::Ok) return s2;

    out.m1 = m1;
    out.m2 = m2;
    return Status::Ok;
}

JointAngles to_joint(const Config& cfg, MotorAngles m) {
    return JointAngles{to_joint(cfg.shoulder, m.m1), to_joint(cfg.elbow, m.m2)};
}

MotorVels to_motor_vel(const Config& cfg, JointVels dq) {
    return MotorVels{to_motor_vel(cfg.shoulder, dq.dq1), to_motor_vel(cfg.elbow, dq.dq2)};
}

JointVels to_joint_vel(const Config& cfg, MotorVels dm) {
    return JointVels{to_joint_vel(cfg.shoulder, dm.dm1), to_joint_vel(cfg.elbow, dm.dm2)};
}

}  // namespace tx
}  // namespace arm

