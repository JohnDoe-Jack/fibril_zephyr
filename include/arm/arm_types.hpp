#pragma once
// 層をまたぐ共通型とスカラ関数。
//
// **pico-sdk に依存しない。** ホストでそのままコンパイルでき、ロジックは実機なしで
// 検証できる。依存は <cmath> <cstdint> <cstddef> だけ。
//
// 単位は mm / mm/s / rad / rad/s。角度は **θ1 も θ2 も絶対角**で、
// θ2 は「上腕に対する相対角」ではない (平行リンク機構のため)。
// ここを取り違えると全部狂うので、型名ではなくコメントで繰り返し書いてある。

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace arm {

constexpr float PI     = 3.14159265358979f;
constexpr float TWO_PI = 6.28318530717959f;
constexpr float DEG    = PI / 180.0f;

struct Point       { float x  = 0.0f, y  = 0.0f; };          // mm
struct Velocity    { float vx = 0.0f, vy = 0.0f; };          // mm/s
struct JointAngles { float theta1 = 0.0f, theta2 = 0.0f; };  // rad (ともに絶対角)
struct JointVels   { float dq1 = 0.0f, dq2 = 0.0f; };        // rad/s

// --- スカラ -----------------------------------------------------------------

constexpr float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

// 角度を [-PI, PI) へ畳む。
// すでに範囲内なら分岐 1 回で返る — 制御周期のほとんどはこの経路を通る。
inline float normalize_angle(float rad) {
    if (rad >= -PI && rad < PI) return rad;
    rad = fmodf(rad + PI, TWO_PI);
    if (rad < 0.0f) rad += TWO_PI;
    return rad - PI;
}

// prev に最も近い target の等価角 (target + 2πk) を返す。
//
// θ1 は無限回転できるので、正規化された関節角をそのままアクチュエータへ渡すと
// ±π の折り返しでモータ角が跳ぶ。肩は減速比 3 なので跳び幅は 6π になる。
// **連続化した値を状態として持ち、それを伝達比へ渡すこと。**
inline float nearest_angle(float prev, float target) {
    return prev + normalize_angle(target - prev);
}

inline float hypotf2(float x, float y) { return sqrtf(x * x + y * y); }

inline float dist(Point a, Point b) { return hypotf2(a.x - b.x, a.y - b.y); }

inline float norm(Velocity v) { return hypotf2(v.vx, v.vy); }

// --- 有限性 -----------------------------------------------------------------
//
// **クランプは NaN を素通しする。** `clampf(NaN, -1, 1)` は比較がすべて偽になるので
// NaN を返し、`acosf(NaN)` も NaN、そのあとの範囲判定もすべて偽。結果として
// 「Ok と NaN の関節角」が返り、それがモータ角になって実機へ届く。
// NaN はどちらの端に寄せても安全でないので、**入口で受け取らない**のが唯一の答え。
inline bool is_finite(float v) { return std::isfinite(v); }
inline bool is_finite(Point p) { return is_finite(p.x) && is_finite(p.y); }
inline bool is_finite(Velocity v) { return is_finite(v.vx) && is_finite(v.vy); }
inline bool is_finite(JointAngles q) { return is_finite(q.theta1) && is_finite(q.theta2); }
inline bool is_finite(JointVels d) { return is_finite(d.dq1) && is_finite(d.dq2); }

}  // namespace arm

