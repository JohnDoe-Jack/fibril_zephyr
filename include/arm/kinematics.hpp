#pragma once
// ② 運動学。1 点の座標変換と、機構固有の制約チェックだけ。
//
// **状態を持たない (すべて純粋関数)。** 作業領域も速度制限も減速比も扱わない。
// ここに置く制約は「この機体なら設置場所によらず常に成り立つもの」だけ。
//
// ============================================================================
// 機構: 水平面内の平行リンク 2 リンク。**θ2 は前腕の絶対角**
// ============================================================================
//
//         肩          肘              手先
//         ●━━━ L1 ━━━●━━━ L2 ━━━━━━━●
//        (θ1)       (θ2)
//
//   x = L1·cos θ1 + L2·cos θ2
//   y = L1·sin θ1 + L2·sin θ2
//   e = normalize(θ2 − θ1)                       … 肘角。−π〜+π
//   d = √(L1² + L2² + 2·L1·L2·cos e)             … 肩から手先までの距離
//
// θ2 を絶対角で定式化するとヤコビが分離する:
//
//   J     = [[−L1·sin θ1, −L2·sin θ2],
//            [ L1·cos θ1,  L2·cos θ2]]
//   det J = L1·L2·sin e
//
// J⁻¹ を展開すると L1・L2 が約分され、逆速度変換の除算は 2 回で済む。
//
// ============================================================================
// **|e| が小さいほど「伸びきり」、大きいほど「折り畳み」**
// ============================================================================
//
// ここが直感と逆になりやすい。L = 600 のとき:
//
//   |e| → 0°    d → 1200 mm   伸びきり。**死点** (det J = 0)
//   |e| = 10°   d = 1195 mm   到達可能な **外側**の限界
//   |e| = 170°  d =  104 mm   到達可能な **内側**の限界
//   |e| → 180°  d → 0 mm      完全に折り畳み。**死点**、手先が肩に干渉
//
// θ1 に可動範囲の制約が無いので、到達可能領域は内半径 104 / 外半径 1195 の**円環**。
//
// 元仕様の Status 名 (`ElbowTooClosed` / `TooCloseToStraight`) はこの対応と
// 逆になっていた。数値と IK の骨子は互いに整合していたので**判定の振る舞いは
// そのまま**にし、名前だけ幾何に合わせて `TooExtended` / `TooFolded` にしてある。
// 詳細は docs/arm_control.md。

#include "arm/arm_types.hpp"

namespace arm {
namespace kin {

// 組み立てで肘の曲がる向きが片側に固定される。死点 (0° と 180°) を跨ぐ動作は
// 機構を壊すので、片側だけを使う。
//
// **template ではなく Config のメンバにしてある。** 左右対称機、組立後の修正、
// キャリブレーション中の一時的な反転で実行時に切り替えられる必要がある。
// `constexpr Config` として使えばコンパイル時に畳み込まれる。
enum class ElbowSide : int8_t { Positive = +1, Negative = -1 };

struct Config {
    float l1 = 600.0f;  // mm
    float l2 = 600.0f;  // mm

    ElbowSide elbow_side = ElbowSide::Positive;

    // 肘角 |e| の許容範囲。**小さい側が伸びきり、大きい側が折り畳み。**
    float elbow_min = 10.0f * DEG;   // これより小さい = 伸びきりすぎ (死点 0° が近い)
    float elbow_max = 170.0f * DEG;  // これより大きい = 折り畳みすぎ (死点 180°、肩に干渉)

    // 特異点判定の閾値。**`normalized_manipulability()` と比べる無次元量。**
    // 生の `det_jacobian()` は l1·l2 のスケール (L=600 で最大 360000) を持つので、
    // 1e-3 と直接比べてもほとんど意味を成さない。正規化すればリンク長を
    // 変えても閾値の意味が変わらない。
    float det_eps = 1.0e-3f;
};

enum class Status : uint8_t {
    Ok = 0,
    TooExtended,     // |e| < elbow_min。伸びきり側。目標が外側限界より遠い
    TooFolded,       // |e| > elbow_max。折り畳み側。目標が内側限界より近い
    WrongElbowSide,  // 肘の曲がる向きが Config と逆。その姿勢はこの機体では作れない
    Singular,        // normalized_manipulability() < det_eps。速度変換ができない
    NotFinite,       // 入力に NaN / ∞ が混じっている
    BadConfig,       // リンク長が正でない
};

// 設定が成立しているか。**起動時に 1 度呼ぶ。** 制御周期で呼ぶものではない。
Status validate(const Config& cfg);

// --- 位置 -------------------------------------------------------------------

// 入力は有限であること (呼び出し側が保証する)。NaN を渡せば NaN が返る。
Point forward(const Config& cfg, JointAngles q);

Status inverse(const Config& cfg, Point target, JointAngles& out);

// inverse と同じ判定だけを行う。関節角は要らないが可否は知りたい場面用。
Status check_reachable(const Config& cfg, Point target);

// **点ではなく姿勢を検査する。** 関節ジョグや校正のように、運動学を通さずに
// 関節角を直接作る経路で要る。肘角の範囲に加えて**曲がる向き**も見る —
// 組み立てで片側に固定されているので、逆側の姿勢はこの機体では作れない。
Status check_joints(const Config& cfg, JointAngles q);

// 到達可能な半径。**|e| が小さいほど遠い**ので、外側が elbow_min 側になる。
float reach_outer(const Config& cfg);
float reach_inner(const Config& cfg);

// --- ヤコビ -----------------------------------------------------------------

void  jacobian(const Config& cfg, JointAngles q, float J[2][2]);
float det_jacobian(const Config& cfg, JointAngles q);    // = l1·l2·sin e。単位 mm²
float manipulability(const Config& cfg, JointAngles q);  // |det J|。単位 mm²

// |det J| / (l1·l2) = |sin e|。**det_eps と比べるのはこちら。**
// 生の可操作度はリンク長の 2 乗のスケールを持つので、閾値と直接比べられない。
// 2 つある理由: manipulability() は物理量 (どれだけ手先を動かせるか)、
// こちらは姿勢の良さだけを取り出した無次元量。
float normalized_manipulability(const Config& cfg, JointAngles q);

// --- 速度 -------------------------------------------------------------------

Status   inverse_velocity(const Config& cfg, JointAngles q, Velocity v, JointVels& out);
Velocity forward_velocity(const Config& cfg, JointAngles q, JointVels dq);

// --- 肘角 -------------------------------------------------------------------

inline float elbow_angle(JointAngles q) { return normalize_angle(q.theta2 - q.theta1); }

}  // namespace kin
}  // namespace arm

