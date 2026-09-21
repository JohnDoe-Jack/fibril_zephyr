// ③ 作業領域の検証。
//
// 期待値は幾何から手で立てる。**特に矩形の外側は角を含めた厳密なユークリッド距離**で、
// min(x−x_min, ...) で済ませた実装とはここで差が出る。
//
// **リンクしているのは workspace.cpp 1 本だけ。**

#include "test_util.hpp"

#include "arm/workspace.hpp"

namespace {

using arm::Point;
namespace ws = arm::ws;

}  // namespace

int main() {
    test::section("17. 単一形状の符号付き距離 (内側が正)");
    {
        const ws::Shape r = ws::rect(-100.0f, 100.0f, -50.0f, 50.0f, true);
        test::checkNearF(ws::signed_distance(r, Point{0.0f, 0.0f}), 50.0f, 1e-4f,
                         "矩形の内側は最も近い辺までの距離");
        test::checkNearF(ws::signed_distance(r, Point{100.0f, 0.0f}), 0.0f, 1e-4f, "辺の上は 0");
        test::checkNearF(ws::signed_distance(r, Point{0.0f, 60.0f}), -10.0f, 1e-4f,
                         "辺の外側は負");
        // **角の外側。** 最近点は角 (100, 50) で、距離は hypot(50, 40)。
        // min(x−x_min, …) だけの実装だと −50 になってしまう。
        test::checkNearF(ws::signed_distance(r, Point{150.0f, 90.0f}), -hypotf(50.0f, 40.0f), 1e-3f,
                         "角の外側は角までの厳密なユークリッド距離");

        const ws::Shape c = ws::circle(0.0f, 0.0f, 0.0f, 200.0f, true);
        test::checkNearF(ws::signed_distance(c, Point{0.0f, 0.0f}), 200.0f, 1e-4f, "円板の中心");
        test::checkNearF(ws::signed_distance(c, Point{200.0f, 0.0f}), 0.0f, 1e-3f, "円周上は 0");
        test::checkNearF(ws::signed_distance(c, Point{300.0f, 0.0f}), -100.0f, 1e-3f, "外側は負");

        const ws::Shape a = ws::circle(0.0f, 0.0f, 100.0f, 200.0f, true);
        test::checkNearF(ws::signed_distance(a, Point{150.0f, 0.0f}), 50.0f, 1e-3f,
                         "円環は内外どちらの壁も見る");
        test::checkNearF(ws::signed_distance(a, Point{0.0f, 0.0f}), -100.0f, 1e-3f,
                         "円環の穴の中は負");
    }

    test::section("扇形は角度方向の符号が正しい");
    {
        const ws::Shape s =
            ws::annulus_sector(0.0f, 0.0f, 100.0f, 500.0f, -0.25f * arm::PI, 0.25f * arm::PI, true);

        test::check(ws::signed_distance(s, Point{300.0f, 0.0f}) > 0.0f, "扇の内側は正");
        test::check(ws::signed_distance(s, Point{0.0f, 300.0f}) < 0.0f, "扇の外 (90 deg) は負");
        test::check(ws::signed_distance(s, Point{-300.0f, 0.0f}) < 0.0f, "扇の真後ろも負");
        test::check(ws::signed_distance(s, Point{50.0f, 0.0f}) < 0.0f, "内半径より内側は負");
        test::check(ws::signed_distance(s, Point{600.0f, 0.0f}) < 0.0f, "外半径より外側は負");
        // 角度境界のちょうど上 (45 deg) は 0
        const float rr = 300.0f;
        test::checkNearF(ws::signed_distance(s, Point{rr * cosf(0.25f * arm::PI),
                                                      rr * sinf(0.25f * arm::PI)}),
                         0.0f, 1e-3f, "角度境界の上は 0");
        // 半径方向は厳密
        test::checkNearF(ws::signed_distance(s, Point{110.0f, 0.0f}), 10.0f, 1e-3f,
                         "半径方向は厳密 (角度方向より近いとき)");
    }

    test::section("15. 許可領域と禁止領域の合成");
    {
        ws::Workspace<8> space;
        test::check(space.add(ws::rect(-800.0f, 800.0f, 100.0f, 900.0f, true)), "作業台を足す");
        test::check(space.add(ws::circle(0.0f, 0.0f, 0.0f, 300.0f, false)), "自機の周りを禁止");
        test::checkEq(space.count(), 2u, "2 つ入っている");

        test::check(space.contains(Point{0.0f, 500.0f}), "許可の中で禁止の外なら入れる");
        test::check(!space.contains(Point{0.0f, 200.0f}), "禁止の中は入れない");
        test::check(!space.contains(Point{0.0f, 50.0f}), "許可の外は入れない");
        test::check(!space.contains(Point{0.0f, 1000.0f}), "許可の外 (上) も入れない");

        // 距離の値も確かめる。許可 400 と 禁止まで 200 の小さいほう。
        test::checkNearF(space.distance_to_boundary(Point{0.0f, 500.0f}), 200.0f, 1e-3f,
                         "禁止円までの距離が効く");
        // 許可の縁のほうが近い点
        test::checkNearF(space.distance_to_boundary(Point{0.0f, 850.0f}), 50.0f, 1e-3f,
                         "許可の縁までの距離が効く");

        // 許可領域を 2 つにすると和集合になる
        ws::Workspace<8> two;
        two.add(ws::rect(0.0f, 100.0f, 0.0f, 100.0f, true));
        two.add(ws::rect(200.0f, 300.0f, 0.0f, 100.0f, true));
        test::check(two.contains(Point{50.0f, 50.0f}), "片方に入っていればよい");
        test::check(two.contains(Point{250.0f, 50.0f}), "もう片方でもよい");
        test::check(!two.contains(Point{150.0f, 50.0f}), "どちらにも入っていなければ不可");

        // 禁止領域を 2 つにすると、どれにも入っていないこと
        ws::Workspace<8> deny2;
        deny2.add(ws::circle(0.0f, 0.0f, 0.0f, 100.0f, false));
        deny2.add(ws::circle(500.0f, 0.0f, 0.0f, 100.0f, false));
        test::check(deny2.contains(Point{250.0f, 0.0f}), "どちらの禁止領域にも入っていない");
        test::check(!deny2.contains(Point{50.0f, 0.0f}), "1 つ目の禁止領域の中");
        test::check(!deny2.contains(Point{480.0f, 0.0f}), "2 つ目の禁止領域の中");

        // 領域が空なら全面許可
        ws::Workspace<4> empty;
        test::check(empty.contains(Point{99999.0f, -99999.0f}), "制約なしなら全面許可");
    }

    test::section("16. 境界上の点の扱いが判定関数どうしで一貫している");
    {
        ws::Workspace<8> space;
        space.add(ws::rect(-500.0f, 500.0f, -300.0f, 300.0f, true));
        space.add(ws::circle(100.0f, 0.0f, 0.0f, 150.0f, false));
        space.add(ws::annulus_sector(0.0f, 0.0f, 50.0f, 400.0f, 0.0f, 0.5f * arm::PI, true));

        // 格子で全点確認する。**contains は距離から定義されている**ので、
        // ここが破れたらどちらかを別実装に差し替えたということ。
        int  checked    = 0;
        bool consistent = true;
        for (int ix = -60; ix <= 60; ++ix) {
            for (int iy = -40; iy <= 40; ++iy) {
                const Point p{static_cast<float>(ix) * 10.0f, static_cast<float>(iy) * 10.0f};
                const bool  in = space.contains(p);
                const bool  ge = space.distance_to_boundary(p) >= 0.0f;
                if (in != ge) consistent = false;
                ++checked;
            }
        }
        test::check(checked > 9000, "十分な点数を検査した");
        test::check(consistent, "contains(p) == (distance_to_boundary(p) >= 0) が全点で成立");

        // 許可領域は境界を含む / 禁止領域は厳密内部のみを禁止
        ws::Workspace<4> edge;
        edge.add(ws::rect(0.0f, 100.0f, 0.0f, 100.0f, true));
        test::check(edge.contains(Point{100.0f, 50.0f}), "許可領域の境界は入れる");
        test::checkNearF(edge.distance_to_boundary(Point{100.0f, 50.0f}), 0.0f, 1e-4f,
                         "境界の距離は 0");

        ws::Workspace<4> pillar;
        pillar.add(ws::circle(0.0f, 0.0f, 0.0f, 100.0f, false));
        test::check(pillar.contains(Point{100.0f, 0.0f}), "禁止領域の境界は入れる (厳密内部のみ禁止)");
        test::check(!pillar.contains(Point{99.0f, 0.0f}), "厳密内部は入れない");
        test::checkNearF(pillar.distance_to_boundary(Point{100.0f, 0.0f}), 0.0f, 1e-3f,
                         "禁止領域の境界も距離 0");
    }

    test::section("実行時の追加・書き換え・削除");
    {
        ws::Workspace<3> small;
        test::check(small.add(ws::rect(0.0f, 1.0f, 0.0f, 1.0f, true)), "1 つ目");
        test::check(small.add(ws::rect(0.0f, 2.0f, 0.0f, 2.0f, true)), "2 つ目");
        test::check(small.add(ws::rect(0.0f, 3.0f, 0.0f, 3.0f, true)), "3 つ目");
        test::check(!small.add(ws::rect(0.0f, 4.0f, 0.0f, 4.0f, true)), "満杯なら false");
        test::checkEq(small.count(), 3u, "3 つのまま");

        // 運転中に可動範囲を絞る
        test::check(small.set(0, ws::rect(0.0f, 10.0f, 0.0f, 1.0f, true)), "set で差し替えられる");
        test::checkNearF(small.at(0)->x_max, 10.0f, 1e-6f, "書き換わっている");
        test::check(small.at(3) == nullptr, "範囲外の読み出しは nullptr");
        test::check(!small.set(3, ws::rect(0.0f, 1.0f, 0.0f, 1.0f, true)), "範囲外の set は false");

        // **set は add と同じ検査を通す。** 生ポインタを返していた頃は、
        // 入口で弾いたはずの逆転した矩形を後から作れてしまい、
        // もっともらしい符号付き距離が返っていた。
        test::check(!small.set(0, ws::rect(100.0f, -100.0f, 0.0f, 1.0f, true)),
                    "成立しない形状への差し替えを拒む");
        test::checkNearF(small.at(0)->x_max, 10.0f, 1e-6f, "拒んだときは元のまま");

        test::check(small.remove(1), "削除できる");
        test::checkEq(small.count(), 2u, "2 つになった");
        test::checkNearF(small.at(1)->x_max, 3.0f, 1e-6f, "後ろが詰められている");
        test::check(!small.remove(5), "範囲外の削除は false");

        small.clear();
        test::checkEq(small.count(), 0u, "clear で空になる");
    }

    test::section("成立していない形状は受け付けない");
    {
        ws::Workspace<8> space;
        test::check(!space.add(ws::rect(100.0f, -100.0f, 0.0f, 10.0f, true)), "逆転した矩形");
        test::check(!space.add(ws::circle(0.0f, 0.0f, -5.0f, 100.0f, true)), "負の内半径");
        test::check(!space.add(ws::circle(0.0f, 0.0f, 200.0f, 100.0f, true)), "内半径 > 外半径");
        test::check(!space.add(ws::annulus_sector(0.0f, 0.0f, 0.0f, 100.0f, 1.0f, 0.0f, true)),
                    "逆転した角度");

        const float nan_v = sqrtf(-1.0f);
        test::check(!space.add(ws::rect(nan_v, 1.0f, 0.0f, 1.0f, true)), "NaN を含む矩形");
        test::checkEq(space.count(), 0u, "1 つも入っていない");

        test::check(space.add(ws::rect(0.0f, 0.0f, 0.0f, 0.0f, true)), "退化した矩形 (点) は許す");
        test::check(ws::is_valid(ws::circle(0.0f, 0.0f, 0.0f, 0.0f, true)), "半径 0 の円も許す");
    }

    return test::report("arm_workspace");
}

