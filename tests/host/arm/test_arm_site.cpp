// **この設置場所の値そのものを検査する。**
//
// 作業領域と行き先は main.cpp の中にあり、ホストから見えなかった。だから
// 「行き先が作業領域に入っているか」を誰も確かめられず、監査の積み残しに
// なっていた。site.hpp へ出したのでここで確かめられる。
//
// 数値そのものは書かない。台を測り直せば変わる値で、写せば「変えたら 2 か所
// 直す」が生まれる。**測り直しても成り立っていなければならない性質**だけを置く。

#include "test_util.hpp"

#include "arm_control/profile.hpp"
#include "arm_control/site.hpp"

namespace {

using arm::Point;
namespace kin = arm::kin;

uint64_t kcode(kin::Status s) { return static_cast<uint64_t>(s); }

}  // namespace

int main() {
    const kin::Config kc = armctl::kinematicsConfig();

    armctl::Space space;
    test::check(armctl::buildWorkspace(space), "作業領域を組み立てられる");
    // **形が増えて容量を超えると add() が黙って false を返す。** 上の 1 行が
    // それを捕まえるが、境界も見ておく。
    test::check(armctl::max_shapes >= 2, "形を 2 つ置ける容量がある");

    test::section("1. 行き先が作業領域の中にある");
    {
        // **監査の積み残しがここ。** 到達できても作業領域の外なら
        // `g` は Fault::OutOfWorkspace で断られ、その定数は用を成さない。
        const Point target{armctl::target_x_mm, armctl::target_y_mm};
        test::check(space.contains(target), "`g` の行き先が許可域に入っている");
        test::checkEq(kcode(kin::check_reachable(kc, target)), kcode(kin::Status::Ok),
                      "行き先が到達可能な円環の中にある");
    }

    test::section("2. 原点合わせの基準姿勢の手先も、作業領域の中にある");
    {
        // **ここが外だと `h` の直後から動かせない。** 基準姿勢は人が手で
        // 持っていく姿勢なので、機構として作れるだけでは足りず、
        // その場所が許可域でなければ次の一手が無い。
        const arm::JointAngles home{armctl::home_theta1_deg * arm::DEG,
                                    armctl::home_theta2_deg * arm::DEG};
        const Point            tip = kin::forward(kc, home);
        test::check(space.contains(tip), "基準姿勢の手先が許可域に入っている");
    }

    test::section("3. 肩の根元は禁止されている");
    {
        // 支柱まわりの禁止円が効いていること。**許可矩形だけだと原点は通る。**
        test::check(!space.contains(Point{0.0f, 0.0f}), "原点は入れない");
        test::check(!space.contains(Point{100.0f, 0.0f}), "根元のすぐ横も入れない");
    }

    test::section("4. 作業領域の外は外と判定される");
    {
        // 矩形の外。**どちらの軸でも効くこと** — 片方だけ符号を間違えると、
        // もう片方は通ってしまう。
        test::check(!space.contains(Point{2000.0f, 0.0f}), "+x の遠方は外");
        test::check(!space.contains(Point{-2000.0f, 0.0f}), "-x の遠方は外");
        test::check(!space.contains(Point{0.0f, 2000.0f}), "+y の遠方は外");
        test::check(!space.contains(Point{0.0f, -2000.0f}), "-y の遠方は外");
    }

    test::section("5. 作業領域は機構の到達限界を写していない");
    {
        // **二重管理をしていないことの検査。** 到達限界はリンク長と肘角から
        // 出る値で、site.hpp が持ってはいけない。作業領域の中に「届かないが
        // 許可されている」点が在ることが、写していない証拠になる。
        // (届かない点は kinematics が別に断るので、二重に守られている)
        // **斜め 45 度を取る。** 軸上だと矩形の辺 (±900) が先に効いて、
        // 到達限界より手前で外になってしまう。角の方向なら矩形の中のまま
        // 到達限界を越えられる。
        const float outer = kin::reach_outer(kc);
        const float d     = (outer + 50.0f) * 0.70710678f;
        const Point beyond{d, d};
        test::check(space.contains(beyond), "許可域は到達限界を知らない");
        test::checkEq(kcode(kin::check_reachable(kc, beyond)),
                      kcode(kin::Status::TooExtended), "**届かないほうは運動学が断る**");
    }

    return test::report("arm_site");
}

