#pragma once
// **この設置場所の値。台を変えたら書き換えるのはここだけ。**
//
// profile.hpp との線引き:
//
//   profile.hpp  **この機体そのもの** (リンク長・肘角・減速比・CAN_ID・運転の上限)
//   site.hpp     **この設置場所**     (作業領域の形状・行き先)
//
// 同じアームを別の台へ据え直したら profile.hpp は 1 行も変わらず、
// こちらだけが変わる。逆にリンクを交換したら profile.hpp だけが変わる。
//
// **機構の到達限界をここへ書き写さないこと。** 内側・外側の限界はリンク長と
// 肘角から出るので kinematics が持っている。写すと二重管理になり、
// リンク長を直したときに片方だけ古くなる。
//
// **main.cpp から出したのはゲートで検査するため。** 中にあったころは作業領域も
// 行き先もホストから見えず、「行き先が作業領域に入っているか」を誰も
// 確かめられなかった (監査の積み残し)。

#include "arm_control/controller.hpp"  // Space = arm::ws::Workspace<max_shapes>

#include "arm/workspace.hpp"

namespace armctl {

// この設置場所の作業領域を組み立てる。**形が増えたら max_shapes を見直す** —
// 容量を超えると add() が false を返し、ここが false になる。
inline bool buildWorkspace(Space& space) {
    namespace ws = arm::ws;

    // 手先が動いてよい矩形。**肩の回転軸が原点。実測 (2026-09-11)。**
    if (!space.add(ws::rect(-900.0f, 900.0f, -900.0f, 900.0f, true))) return false;

    // 支柱まわり。肩の根元へ手先を入れさせない。
    if (!space.add(ws::circle(0.0f, 0.0f, 0.0f, 250.0f, false))) return false;

    return true;
}

// `g` で向かう行き先 [mm]。**どこへ行きたいかは設置場所の話**なので、
// 機体の値ではなくこちらに置く。自動操縦の目標点の格子も、足すならここ。
inline constexpr float target_x_mm = 500.0f;
inline constexpr float target_y_mm = 350.0f;

}  // namespace armctl

