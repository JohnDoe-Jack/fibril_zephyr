#pragma once
// テスト用の can::Controller。
//
// SPI を通さず、送信された can::Frame をそのまま記録する。バイト詰めの正しさは
// test_xl2515 が実レジスタで見ているので、上位層のテストでは「どんなフレームを
// 出したか」だけを見ればよい。コントローラを差し替えられる設計であることの
// 実演も兼ねている。

#include "stubs.hpp"

#include "can/controller.hpp"

#include <cstddef>
#include <initializer_list>

namespace testing {

class FakeController final : public can::Controller {
public:
    static constexpr std::size_t max_log     = 64;
    static constexpr std::size_t max_queue   = 64;
    static constexpr std::size_t max_filters = 32;

    explicit FakeController(int irq_pin = 8) : irq_pin_(irq_pin) { updatePin(); }

    // --- can::Controller ----------------------------------------------------
    can::Status send(const can::Frame& f) override {
        // 送信バッファが一時的に埋まる状況を模す。succeed_after 本を通したあと
        // fail_next_sends 本を断り、そのあとまた通る。**恒久的な fail_send とは別。**
        if (fail_next_sends != 0) {
            if (succeed_after != 0) {
                --succeed_after;
            } else {
                --fail_next_sends;
                ++send_failures;
                return can::Status::NoTxBuffer;
            }
        }
        if (fail_send) {
            ++send_failures;
            return can::Status::NoTxBuffer;
        }
        if (sent_count < max_log) sent[sent_count] = f;
        ++sent_count;
        // 要求に対して機器が返してくる、という往復を模すためのフック。
        // 応答を先に積んでおくと、要求より前に消費されてしまって模擬にならない。
        if (on_send != nullptr) on_send(*this, f);
        return can::Status::Ok;
    }

    bool receive(can::Frame& out) override {
        if (read_ >= write_) return false;
        out = queue_[read_ % max_queue];
        ++read_;
        updatePin();
        return true;
    }

    bool isRxPending() const override { return stuck_rx_pending_ || read_ < write_; }

    bool setFilters(const can::RxFilter* filters, std::size_t count) override {
        ++filter_calls;
        // 運転モードへ戻せずに自分を止めるコントローラ (XL2515 の H4 経路) を模す。
        if (offline_on_filters) {
            initialized  = false;
            filter_count = 0;
            return false;
        }
        if (count > filter_capacity) {
            filter_count = 0;
            return false;
        }
        filter_count = count;
        for (std::size_t i = 0; i < count && i < max_filters; ++i) filters_[i] = filters[i];
        return true;
    }

    int  getRxIrqPin() const override { return irq_pin_; }
    bool isRxIrqActiveLow() const override { return true; }

    const can::Health& updateHealth() override { return health_; }
    const can::Health& getHealth() const override { return health_; }
    bool               isBusOff() const override { return false; }
    bool               abortAllTx() override {
        ++abort_calls;
        return !fail_abort;
    }
    bool isInitialized() const override { return initialized; }

    // --- テスト補助 ---------------------------------------------------------
    void pushFrame(const can::Frame& f) {
        if (write_ - read_ >= max_queue) return;
        queue_[write_ % max_queue] = f;
        ++write_;
        updatePin();
    }

    void push(uint32_t id, bool extended, std::initializer_list<uint8_t> data) {
        can::Frame f{};
        f.id       = id;
        f.extended = extended;
        f.dlc      = static_cast<uint8_t>(data.size() < 8 ? data.size() : 8);
        std::size_t i = 0;
        for (uint8_t b : data) {
            if (i >= 8) break;
            f.data[i++] = b;
        }
        pushFrame(f);
    }

    const can::Frame& lastSent() const { return sent[(sent_count == 0 ? 0 : sent_count - 1)]; }
    const can::Frame& sentAt(std::size_t i) const { return sent[i]; }
    void              clearSent() { sent_count = 0; }

    const can::RxFilter& filterAt(std::size_t i) const { return filters_[i]; }

    // Health に値を差し込む。**「読んだ」と「既定値のまま」を見分けるため。**
    // 既定値しか返らない偽物だと、updateHealth() を呼ばない実装も検査を通ってしまう。
    void setHealth(const can::Health& h) { health_ = h; }

    // 受信要求が落ちない故障を模す。**INT ピンは Low のまま、receive() は 1 本も返さない。**
    // GP8 の配線短絡、コントローラの INT 出力の故障、消せない割り込み要因が該当する。
    void setStuckRxPending(bool on) {
        stuck_rx_pending_ = on;
        updatePin();
    }

    // 送信のたびに呼ばれる。キャプチャなしラムダをそのまま代入できる。
    using SendHook = void (*)(FakeController& self, const can::Frame& sent);
    SendHook on_send = nullptr;

    can::Frame  sent[max_log] = {};
    std::size_t sent_count    = 0;
    std::size_t send_failures = 0;
    bool        fail_send     = false;  // 恒久的に送れない
    std::size_t succeed_after = 0;      // 何本通してから詰まらせるか
    std::size_t fail_next_sends = 0;    // 詰まらせる本数。0 になれば復帰する
    bool        initialized   = true;

    std::size_t abort_calls = 0;
    bool        fail_abort  = false;

    std::size_t filter_calls    = 0;
    std::size_t filter_count    = 0;
    std::size_t filter_capacity = 32;  // MCP2518FD 相当。6 にすれば MCP2515 相当
    // setFilters() で自分を未初期化に落とすコントローラ。
    bool offline_on_filters = false;

private:
    void updatePin() {
        if (irq_pin_ >= 0 && irq_pin_ < static_cast<int>(stub::num_gpio)) {
            stub::gpio_state[irq_pin_].level = !isRxPending();  // INT は Low アクティブ
        }
    }

    int           irq_pin_;
    bool          stuck_rx_pending_    = false;
    can::Frame    queue_[max_queue]    = {};
    can::RxFilter filters_[max_filters] = {};
    std::size_t   write_               = 0;
    std::size_t   read_                = 0;
    can::Health   health_{};
};

}  // namespace testing

