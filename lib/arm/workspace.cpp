#include "arm/workspace.hpp"

namespace arm {
namespace ws {
namespace {

// 軸平行矩形の符号付き距離 (内側が正)。**外側も角を含めて厳密なユークリッド距離。**
// min(x − x_min, ...) だけで済ませると角の外側で値が過大になり、減速判断が甘くなる。
float sd_rect(const Shape& s, Point p) {
    const float hx = 0.5f * (s.x_max - s.x_min);
    const float hy = 0.5f * (s.y_max - s.y_min);
    const float cx = 0.5f * (s.x_max + s.x_min);
    const float cy = 0.5f * (s.y_max + s.y_min);

    const float qx = fabsf(p.x - cx) - hx;
    const float qy = fabsf(p.y - cy) - hy;

    const float ox      = (qx > 0.0f) ? qx : 0.0f;
    const float oy      = (qy > 0.0f) ? qy : 0.0f;
    const float outside = hypotf2(ox, oy);         // 外側なら > 0
    const float inside  = (qx > qy) ? qx : qy;     // 内側なら < 0

    // 外側が正の標準形が outside + min(inside, 0)。内側を正にしたいので符号を反転。
    return -(outside + ((inside < 0.0f) ? inside : 0.0f));
}

// 円板 / 円環の半径方向。厳密。
float sd_annulus(float r, float r_min, float r_max) {
    const float d_out = r_max - r;
    if (r_min <= 0.0f) return d_out;  // 円板には内側の壁がない
    const float d_in = r - r_min;
    return (d_in < d_out) ? d_in : d_out;
}

// 扇形。半径方向は厳密、角度方向は「角度境界の直線までの距離」r·sin(Δ) の近似。
//
// Δ を ±π/2 にクランプするので、扇の真後ろでも符号が反転せず単調に振る舞う。
// **符号は常に正しい**ので contains の判定は厳密。絶対値だけが境界から離れた
// ところで誤差を持つが、その距離は減速判断にしか使わない。
float sd_sector(const Shape& s, Point p) {
    const float dx = p.x - s.cx;
    const float dy = p.y - s.cy;
    const float r  = hypotf2(dx, dy);

    const float d_r = sd_annulus(r, s.r_min, s.r_max);

    const float half   = 0.5f * (s.ang_max - s.ang_min);
    const float center = 0.5f * (s.ang_max + s.ang_min);

    if (half >= PI) return d_r;      // 全周ぶんあるので角度の制約にならない
    if (r < 1.0e-6f) return d_r;     // 中心の上では角度が定義できない

    const float a  = atan2f(dy, dx);
    const float da = fabsf(normalize_angle(a - center));

    float       slack = clampf(half - da, -0.5f * PI, 0.5f * PI);  // 正なら扇の内側
    const float d_a   = r * sinf(slack);

    return (d_a < d_r) ? d_a : d_r;
}

bool finite_rect(const Shape& s) {
    return is_finite(s.x_min) && is_finite(s.x_max) && is_finite(s.y_min) && is_finite(s.y_max);
}

bool finite_radial(const Shape& s) {
    return is_finite(s.cx) && is_finite(s.cy) && is_finite(s.r_min) && is_finite(s.r_max);
}

}  // namespace

bool is_valid(const Shape& s) {
    switch (s.type) {
        case ShapeType::Rect:
            return finite_rect(s) && s.x_min <= s.x_max && s.y_min <= s.y_max;
        case ShapeType::Circle:
            return finite_radial(s) && s.r_min >= 0.0f && s.r_min <= s.r_max;
        case ShapeType::AnnulusSector:
            return finite_radial(s) && s.r_min >= 0.0f && s.r_min <= s.r_max &&
                   is_finite(s.ang_min) && is_finite(s.ang_max) && s.ang_min <= s.ang_max;
    }
    return false;
}

float signed_distance(const Shape& s, Point p) {
    switch (s.type) {
        case ShapeType::Rect:
            return sd_rect(s, p);
        case ShapeType::Circle: {
            const float r = hypotf2(p.x - s.cx, p.y - s.cy);
            return sd_annulus(r, s.r_min, s.r_max);
        }
        case ShapeType::AnnulusSector:
            return sd_sector(s, p);
    }
    // 未知の種別は「どこにも属さない」として扱う。許可なら届かず、禁止なら妨げない。
    return -kFar;
}

}  // namespace ws
}  // namespace arm

