#pragma once
// 関節角の連続化。**逆運動学と伝達比のあいだに必ず挟むもの。**
//
// ============================================================================
// なぜ要るか
// ============================================================================
//
// `kin::inverse()` は θ を −π〜+π に正規化して返す。その戻り値をそのまま
// `tx::to_motor()` へ渡すと、θ1 が ±π を跨いだ瞬間にモータ角が 2π×3 = 6π 飛ぶ。
//
//     kin::inverse(cfg, target, q);   // q は −π〜+π
//     tx::to_motor(txcfg, q, m);      // ← ここで飛ぶ。**これはバグ**
//
// 伝達比の層が正規化しないだけでは足りない。**入力側も連続でなければならない。**
// 手先の軌道が滑らかでも、その表現が不連続なら指令は飛ぶ。
//
// ============================================================================
// θ2 を θ1 から導く
// ============================================================================
//
// θ1 と θ2 を別々に `nearest_angle` へ通しても普通は動くが、両者の 2π の巻き数が
// 独立に決まるため、肘角が 2π ずれた状態を検出できない。
//
// ここでは **肘角 e = normalize(θ2 − θ1) を取り出し、連続化した θ1 に足し戻す。**
// e は機構上 ±10°〜±170° に収まる有界な量なので巻き数の曖昧さがなく、
// θ2 = θ1 + e という機構の関係が構造的に保たれる。
//
// 肩が N 回転すれば前腕も一緒に N 回転する (平行リンクで担がれている) ので、
// これが物理的にも正しい。
//
// 状態を持つのは軌道生成層だけ、という規則からは外れる小さな入れ物だが、
// これは層ではなく**層のあいだの継ぎ目**で、アプリケーションが持つ状態にあたる。

#include "arm/arm_types.hpp"

namespace arm {

class JointContinuity {
public:
    // 起動時に実測の関節角で初期化する。**アームは電源投入時どこかに居る。**
    // ここを省くと、最初の track() が −π〜+π のどこかを基準にしてしまい、
    // 実際のモータ角と食い違ったまま指令を出すことになる。
    void seed(JointAngles measured) {
        cont_ = measured;
        have_ = true;
    }

    void reset() {
        cont_ = JointAngles{};
        have_ = false;
    }

    bool has_reference() const { return have_; }

    JointAngles value() const { return cont_; }

    // 正規化された関節角を受け取り、連続な等価角を返す。
    JointAngles track(JointAngles normalized) {
        const float e = normalize_angle(normalized.theta2 - normalized.theta1);
        const float t1 =
            have_ ? nearest_angle(cont_.theta1, normalized.theta1) : normalized.theta1;
        cont_ = JointAngles{t1, t1 + e};
        have_ = true;
        return cont_;
    }

private:
    JointAngles cont_{};
    bool        have_ = false;
};

}  // namespace arm

