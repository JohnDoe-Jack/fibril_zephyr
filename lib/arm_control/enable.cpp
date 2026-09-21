#include "arm_control/enable.hpp"

namespace armctl {

const char* enableText(EnableResult r) {
    switch (r) {
        case EnableResult::Ok:
            return "使能しました (保持)";
        case EnableResult::AlreadyRunning:
            return "すでに運転中です";
        case EnableResult::WaitingDrives:
            return "モータの応答を待っています (24V と E-stop、CAN の配線を確認)";
        case EnableResult::NotHomed:
            return "原点合わせがまだです (基準姿勢へ手で動かしてから M4 / `h`)";
        case EnableResult::NotReleased:
            return "脱力が成立していません (送り直しています。数百 ms 待ってもう一度 M3 / `c`)";
        case EnableResult::ClearFailed:
            return "故障解除が送れません (配線とバスを確認)";
        case EnableResult::TimeoutWriteFailed:
            return "通信途絶保護が書けません "
                   "(**この状態で動かすと基板が固まってもモータは止まりません**)";
        case EnableResult::PositionModeFailed:
            return "位置モードへ入れません (応答が無い / モードが変わらない。もう一度 M3 / `c`)";
        case EnableResult::SeedFailed:
            return "姿勢を取り直せません (もう一度 M3 / `c`)";
        // **押し直せば直りうる。** 見る場所も書く — 「確かめられません」だけ
        // だと、人は保護が入っているのかどうかを判断できない。
        case EnableResult::TimeoutUnverified:
            return "通信途絶保護を書けたか確かめられません "
                   "(モータの応答か CAN の行を見て、もう一度 M3 / `c`)";
        // **押し直しを勧めない。** 積み直しても同じ答えが返る失敗なので、
        // 勧めると人が効かないボタンを押し続ける。見る場所を名指しする。
        case EnableResult::TimeoutMismatch:
            return "通信途絶保護が書いた値と違います "
                   "(profile.hpp の can_timeout_ms と換算、モータのファームを確認。"
                   "生値は rs00-check で読めます)";
    }
    return "不明";
}

}  // namespace armctl

