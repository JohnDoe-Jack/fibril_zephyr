#pragma once
// ③ 作業領域。設置環境・運用に依存する制約だけを持つ。
//
// **機構の到達限界はここに入れない。** それは kinematics の責務で、設置場所を
// 変えても変わらない。ここに入れるのは「この設置ではここへ行かせない」という、
// 現場ごとに変わる制約 (作業台の縁、柱、他の機械)。
//
// 動的確保はしない。領域は静的配列で持ち、実行時に足したり書き換えたりできる。
//
// ============================================================================
// 判定はすべて符号付き距離 (内側が正) で組み立てる
// ============================================================================
//
//   contains(p)  ⇔  distance_to_boundary(p) >= 0
//
// これを **定義から**成り立たせてある。2 つを別々に実装すると、境界上の点の
// 扱いがいつの間にかずれる — 「入っているのに距離が負」という状態は、
// 減速判断と可否判定が食い違うという形で現場に出る。
//
// 合成は CSG の標準形。
//   許可領域の和集合 = max(sd)
//   禁止領域の補集合 = min(−sd)
//   全体は両者の積集合 = min(上の 2 つ)
//
// 境界の扱いは「**許可領域は境界を含む / 禁止領域は厳密内部のみを禁止**」。
// どちらも符号付き距離 0 を「許可」に倒す、という 1 つの規則から出る。

#include "arm/arm_types.hpp"

namespace arm {
namespace ws {

enum class ShapeType : uint8_t { Rect, Circle, AnnulusSector };

struct Shape {
    ShapeType type  = ShapeType::Rect;
    bool      allow = true;  // true = 許可領域、false = 禁止領域

    // Rect
    float x_min = 0.0f, x_max = 0.0f, y_min = 0.0f, y_max = 0.0f;

    // Circle / AnnulusSector。r_min = 0 なら円板、r_min > 0 なら円環。
    float cx = 0.0f, cy = 0.0f, r_min = 0.0f, r_max = 0.0f;

    // AnnulusSector のみ。ang_min <= ang_max。幅が 2π 以上なら角度制約なし。
    float ang_min = 0.0f, ang_max = 0.0f;
};

// --- ファクトリ -------------------------------------------------------------

inline Shape rect(float x_min, float x_max, float y_min, float y_max, bool allow) {
    Shape s;
    s.type  = ShapeType::Rect;
    s.allow = allow;
    s.x_min = x_min;
    s.x_max = x_max;
    s.y_min = y_min;
    s.y_max = y_max;
    return s;
}

inline Shape circle(float cx, float cy, float r_min, float r_max, bool allow) {
    Shape s;
    s.type  = ShapeType::Circle;
    s.allow = allow;
    s.cx    = cx;
    s.cy    = cy;
    s.r_min = r_min;
    s.r_max = r_max;
    return s;
}

inline Shape annulus_sector(float cx, float cy, float r_min, float r_max, float ang_min,
                            float ang_max, bool allow) {
    Shape s;
    s.type    = ShapeType::AnnulusSector;
    s.allow   = allow;
    s.cx      = cx;
    s.cy      = cy;
    s.r_min   = r_min;
    s.r_max   = r_max;
    s.ang_min = ang_min;
    s.ang_max = ang_max;
    return s;
}

// 形状として成立しているか。逆転した矩形や負の半径は黙って受けない。
bool is_valid(const Shape& s);

// 単一形状の符号付き距離。内側が正、外側が負、境界で 0。
//
// 矩形 (角を含む) と円環は厳密なユークリッド距離。扇形の**角度方向だけ**は
// r·sin(Δ) の近似で、**符号は常に正しい**が絶対値は境界付近以外で誤差を持つ。
// 減速判断には十分で、`contains` の正しさには影響しない。
float signed_distance(const Shape& s, Point p);

// 「無限遠」の代わりに使う有限の大きな値。∞ の演算を避ける。
constexpr float kFar = 1.0e30f;

template <std::size_t N>
class Workspace {
public:
    static constexpr std::size_t capacity = N;

    // 満杯か、形状が成立していなければ false。
    bool add(const Shape& s) {
        if (n_ >= N) return false;
        if (!is_valid(s)) return false;
        shapes_[n_++] = s;
        return true;
    }

    void        clear() { n_ = 0; }
    std::size_t count() const { return n_; }

    const Shape* at(std::size_t i) const { return (i < n_) ? &shapes_[i] : nullptr; }

    // 実行時に差し替える用 (可動範囲を運転中に絞る、校正中だけ広げるなど)。
    // **add() と同じ検査を通す。** 生のポインタを返して外から書き換えさせると、
    // 入口で弾いたはずの「逆転した矩形」を後から作れてしまい、
    // もっともらしい符号付き距離が返る (実測で確認した)。
    bool set(std::size_t i, const Shape& s) {
        if (i >= n_) return false;
        if (!is_valid(s)) return false;
        shapes_[i] = s;
        return true;
    }

    bool remove(std::size_t i) {
        if (i >= n_) return false;
        for (std::size_t k = i + 1; k < n_; ++k) shapes_[k - 1] = shapes_[k];
        --n_;
        return true;
    }

    // 合成領域の符号付き距離。内側なら正、外側なら負。
    // 領域を 1 つも持たないときは「全面許可」として kFar を返す。
    float distance_to_boundary(Point p) const {
        float d_allow    = -kFar;
        bool  has_allow  = false;
        float d_deny     = kFar;

        for (std::size_t i = 0; i < n_; ++i) {
            const float sd = signed_distance(shapes_[i], p);
            if (shapes_[i].allow) {
                has_allow = true;
                if (sd > d_allow) d_allow = sd;  // 和集合
            } else {
                const float outside = -sd;       // 補集合
                if (outside < d_deny) d_deny = outside;
            }
        }

        if (!has_allow) d_allow = kFar;  // 許可領域の指定なし = 全面許可
        return (d_allow < d_deny) ? d_allow : d_deny;
    }

    // 境界 (距離 0) は「内側」として扱う。
    bool contains(Point p) const { return distance_to_boundary(p) >= 0.0f; }

private:
    Shape       shapes_[N];
    std::size_t n_ = 0;
};

}  // namespace ws
}  // namespace arm

