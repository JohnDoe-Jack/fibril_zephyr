#pragma once
// ④ 軌道生成。時間軸の管理。目標点や速度指令を制御周期ごとの座標へ展開する。
//
// **状態を持つのはこの層だけ。** dt は引数で受け取り、内部でタイマに触らない。
// 生成した点が作業領域の外かどうかは判定しない (それは上位が workspace に訊く)。
//
// ============================================================================
// オンライン方式を採る理由
// ============================================================================
//
// 事前に台形区間を計算しておくオフライン方式ではなく、毎周期
// 「いまの (v, a) から止まるのに要る距離」と「残距離」を比べて加減速を決める。
//
//   ・可変 dt でも破綻しない
//   ・一時停止 / 再開 / 中断を再計画なしで扱える (位置が飛ばない)
//   ・速度・加速度・ジャークの上限が構造的に守られる
//
// 直線も円弧も、経路長 s ∈ [0, length] を扱う 1 自由度プランナ (ScalarPlanner) を
// 共有し、s から座標への写像だけを差し替える。

#include "arm/arm_types.hpp"

namespace arm {
namespace motion {

struct TrapezoidProfile {
    float v_max = 200.0f;   // mm/s
    float a_max = 1000.0f;  // mm/s²
    float jerk  = 0.0f;     // mm/s³。0 以下なら台形 (ジャーク制限なし)
};

enum class State : uint8_t { Idle, Running, Paused, Done, Aborted };

// ---------------------------------------------------------------------------
// 経路長に沿った 1 自由度の台形 / S字プランナ。
// LinearMove / ArcMove が内部で共有する。単体でもテストできる。
// ---------------------------------------------------------------------------
class ScalarPlanner {
public:
    // length <= 0 なら即 Done。プロファイルが成立していなければ false。
    bool start(float length, const TrapezoidProfile& p);

    void pause();   // 加速度制限を守って止まる (位置は連続)
    void resume();
    void abort();   // 減速して止まり、以降 Aborted のまま。**即座には止めない**
    void reset();

    float update(float dt);  // 更新後の s を返す

    float s() const { return len_ - rem_; }
    float v() const { return v_; }
    float a() const { return a_; }
    float length() const { return len_; }
    float remaining() const { return rem_; }
    float progress() const {
        return (len_ > 0.0f) ? clampf((len_ - rem_) / len_, 0.0f, 1.0f) : 1.0f;
    }

    State state() const { return st_; }
    bool  is_done() const { return st_ == State::Done; }
    bool  is_active() const { return st_ == State::Running || st_ == State::Paused; }

private:
    // いまの (v, a) から、加速度・ジャーク制限を守って止まるのに要る距離。
    float stop_distance(float v, float a, float dt) const;

    TrapezoidProfile pr_{};
    float            len_ = 0.0f;

    // **進捗は s ではなく「残距離」で持つ。**
    // s を積算すると、len = 1000 mm のあたりで float の分解能が約 6e-5 mm になり、
    // 終端の低速域 (v < 0.06 mm/s) では増分が丸めに埋もれて位置が動かなくなる。
    // 結果 is_done() が永久に立たない。残距離を減らしていけば、小さくなるほど
    // 相対精度が上がるのでこの問題が起きない。
    float rem_ = 0.0f;
    float v_   = 0.0f;
    float a_   = 0.0f;

    // **減速フェーズに入ったことのラッチ。**
    // 「残距離 <= 停止距離」を毎周期そのまま評価すると、1 周期減速した直後に
    // 停止距離が縮んで条件が外れ、加速に戻る。これを繰り返すと減速が間に合わず、
    // 終点を数十〜百数十 mm/s の速度を持ったまま通過する。
    // 停止距離を保守的に取ったせいで手前で止まりきったときだけ解除し、
    // 低速で残りを詰める (クリープ)。resume() でも解除する。
    bool  decel_ = false;
    State st_    = State::Idle;
};

// ---------------------------------------------------------------------------
// 1. ジョグ (手動)。速度指令を加速度制限つきで追従させ、積分して目標点を更新する。
// ---------------------------------------------------------------------------
class Jog {
public:
    void set_limits(float v_max, float a_max);  // mm/s, mm/s²

    // 大きさが v_max を超える指令は **向きを保ったまま**クランプする。
    // 成分ごとにクランプすると斜め方向の指令で向きが変わる。
    void set_velocity(Velocity v);
    void stop() { set_velocity(Velocity{0.0f, 0.0f}); }

    Point update(float dt);

    void  set_position(Point p);  // 初期化・同期用。速度もクリアする
    Point position() const { return pos_; }
    Velocity velocity() const { return vel_; }  // 実際に出ている速度 (指令ではない)
    bool     is_moving() const;

private:
    Point    pos_{};
    Velocity vel_{};  // ランプ後の現在速度
    Velocity cmd_{};  // クランプ後の指令速度
    float    v_max_ = 200.0f;
    float    a_max_ = 1000.0f;
};

// ---------------------------------------------------------------------------
// 2. 直線補間
// ---------------------------------------------------------------------------
class LinearMove {
public:
    bool  start(Point from, Point to, const TrapezoidProfile& p);
    Point update(float dt);

    void pause() { pl_.pause(); }
    void resume() { pl_.resume(); }
    void abort() { pl_.abort(); }

    bool     is_done() const { return pl_.is_done(); }
    float    progress() const { return pl_.progress(); }
    State    state() const { return pl_.state(); }
    Point    position() const;
    Velocity velocity() const;

private:
    ScalarPlanner pl_;
    Point         from_{}, to_{};
    float         ux_ = 1.0f, uy_ = 0.0f;  // 単位方向ベクトル
};

// ---------------------------------------------------------------------------
// 3. 円弧補間。ang_end > ang_start なら反時計回り。
// ---------------------------------------------------------------------------
class ArcMove {
public:
    bool  start(Point center, float radius, float ang_start, float ang_end,
                const TrapezoidProfile& p);
    Point update(float dt);

    void pause() { pl_.pause(); }
    void resume() { pl_.resume(); }
    void abort() { pl_.abort(); }

    bool     is_done() const { return pl_.is_done(); }
    float    progress() const { return pl_.progress(); }
    State    state() const { return pl_.state(); }
    Point    position() const;
    Velocity velocity() const;
    float    angle() const;  // 現在の中心角

private:
    ScalarPlanner pl_;
    Point         center_{};
    float         r_    = 0.0f;
    float         a0_   = 0.0f;
    float         a1_   = 0.0f;  // 終端の角度。完了時にここへ吸着させる
    float         dir_  = 1.0f;  // +1 = 反時計回り、−1 = 時計回り
};

}  // namespace motion
}  // namespace arm

