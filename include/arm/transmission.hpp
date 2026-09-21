#pragma once
// ②' 伝達比。関節角 ⇄ モータ角の単位変換だけ。
//
// **状態を持たない (すべて純粋関数)。** 機構的な可否判定は kinematics の責務で、
// ここではやらない。運動学と伝達比は別物なので、この層は kinematics を
// include しないし、されもしない。
//
//   motor_angle = (joint_angle − offset) · ratio · direction
//   joint_angle =  motor_angle / (ratio · direction) + offset
//   motor_vel   =  joint_vel · ratio · direction        (オフセットは効かない)
//   joint_vel   =  motor_vel / (ratio · direction)
//
// ============================================================================
// **機構の可動限界はここに持たない**
// ============================================================================
//
// ケーブルの巻取り限界のようなものはこの層の関心事ではない。この機体は
// 肩が無限回転で、肘の制約は「死点を避ける肘角の範囲」として kinematics が持つ。
// アクチュエータ自身が表現できる範囲 (RS00 なら位置指令が ±12.57 rad) も
// アクチュエータの性質なので、そちらのライブラリの話。
//
// ============================================================================
// **正規化してはいけない**
// ============================================================================
//
// θ1 は無限回転でき、しかも肩の減速比は 3 なのでモータは 3 倍回る。
// 出力を −π〜+π へ畳むとモータ角の連続性が失われ、指令が飛んで機構を壊す。
// 出力は連続値のまま返す。多回転の追跡は上位が累積角で行う前提。
//
// **この層が正規化しないだけでは足りない。** `kin::inverse()` は −π〜+π に
// 正規化された関節角を返すので、その戻り値を直接ここへ渡すと、θ1 が ±π を
// 跨いだ瞬間にモータ角が 2π×3 = 6π 飛ぶ。`arm::JointContinuity` を挟むこと。
//
// ============================================================================
// 失敗しうるのは位置の書き込みだけ
// ============================================================================
//
//   関節 → モータ (位置)  Status を返す。**間違った位置は危険**なので失敗しうる
//   関節 → モータ (速度)  値を返す。係数が使えない設定なら 0 = 動かない
//   モータ → 関節 (両方)  値を返す。読み出しで NaN を撒くほうが害が大きい
//
// バックラッシュ (肩の外部平歯車、先端換算 3〜6 mm 相当) はこの層では補正しない。
// 補正には運動の向きの履歴が要り、この層は状態を持たないため。

#include "arm/arm_types.hpp"

namespace arm {
namespace tx {

// 方向とオフセットは組立ごとに実機で確認する。**ハードコードしない。**
// モータのエンコーダ原点とアームの θ = 0 は一致しない。
//
// ratio / direction は歯車列の性質で、機械設計を変えない限り動かない。
// offset はエンコーダの原点で、**再原点合わせ・再組立・モータ交換のたびに変わる**。
// 寿命が違うので、offset は起動時の校正手順が決めるものとして扱うこと。
struct JointTransmission {
    float  ratio     = 1.0f;
    int8_t direction = +1;    // +1 か −1
    float  offset    = 0.0f;  // rad。モータ角 0 に対応する関節角
};

struct Config {
    // 肩: 外部平歯車 m2 z24 → z72 の 1 段。1 段なので回転方向が反転する。
    JointTransmission shoulder{3.0f, -1, 0.0f};
    // 肘: 平行リンクのため直結。
    JointTransmission elbow{1.0f, +1, 0.0f};
};

struct MotorAngles { float m1 = 0.0f, m2 = 0.0f; };    // rad
struct MotorVels   { float dm1 = 0.0f, dm2 = 0.0f; };  // rad/s

enum class Status : uint8_t {
    Ok = 0,
    BadConfig,  // ratio·direction が 0 か、有限でない
    NotFinite,  // 入力に NaN / ∞ が混じっている
};

// 設定が成立しているか。**起動時に 1 度呼ぶ。**
// ratio = 0 のような設定ミスは、静かに 0 を指令するのではなくここで落とす。
Status validate(const Config& cfg);

// 係数 (= ratio · direction) が使えるか。0 だとモータ角が関節角の情報を持たない。
bool usable(const JointTransmission& t);

// --- 1 関節ぶんの変換 -------------------------------------------------------
// **こちらが本体。** 腕の自由度を知らないので、関節が増えても書き換えずに済む。

// **out は Ok のときだけ書く。** 変な値を書き込むと、Status を見ない
// 呼び出し側がそれをそのままモータへ送る。書かなければ前回値が残るだけで、
// 動かない側へ倒れる。
Status to_motor(const JointTransmission& t, float joint_rad, float& out);

// 読み出し方向と速度は total。係数が使えなければモータ 0 に対応する値
// (関節はオフセット、速度は 0) を返す。NaN を撒くよりは有限で無害な値を返し、
// 設定の誤りは validate() で捕まえる。
float to_joint(const JointTransmission& t, float motor_rad);
float to_motor_vel(const JointTransmission& t, float joint_vel);
float to_joint_vel(const JointTransmission& t, float motor_vel);

// --- 2 関節ぶんの便宜版 -----------------------------------------------------

Status      to_motor(const Config& cfg, JointAngles q, MotorAngles& out);
JointAngles to_joint(const Config& cfg, MotorAngles m);
MotorVels   to_motor_vel(const Config& cfg, JointVels dq);
JointVels   to_joint_vel(const Config& cfg, MotorVels dm);

}  // namespace tx
}  // namespace arm

