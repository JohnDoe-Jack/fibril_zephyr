# ESP-NOW gamepad UART bridge

XIAO ESP32-C3 の `esp-now-gamepad-bridge` が送る UART フレームを受信し、
アプリケーションに `gamepad_state` と統計を提供する通信ライブラリ。
`lib/esp_now_gamepad_bridge/` に置き、UART device はアプリケーションから渡す。
受信専用で、ESP-NOW 無線処理、テレメトリ送信、CAN 転送、モータ・サーボ制御は行わない。

## 有効化

```text
CONFIG_SERIAL=y
CONFIG_COBS=y
CONFIG_ESP_NOW_GAMEPAD_BRIDGE=y
CONFIG_ESP_NOW_GAMEPAD_BRIDGE_TIMEOUT_MS=100
```

`CONFIG_COBS=y` を明示する。Zephyr v4.4.1 では SERIAL に依存するこのライブラリから
COBS を select すると、NET_BUF の logging と UART shell の依存を介した循環になる。
CRC と UART runtime configuration はライブラリが有効化する。

受信方式は次のいずれかを選ぶ。両者でプロトコルと公開 API は共通。

| 設定 | 対象 |
| --- | --- |
| `CONFIG_ESP_NOW_GAMEPAD_BRIDGE_ASYNC=y` | Zephyr async UART 対応 device。利用可能なら既定で選択 |
| `CONFIG_ESP_NOW_GAMEPAD_BRIDGE_INTERRUPT=y` | 割り込み UART 対応 device。RP2350 PL011 はこれを使用 |

対応する `CONFIG_UART_ASYNC_API` / `CONFIG_UART_INTERRUPT_DRIVEN` は自動で有効になる。
SoC 内に複数種類の UART がある場合、Kconfig の capability は渡す device 自体の対応を保証しない。
選択した API に対応する device を渡す必要がある。

## 配線と UART の選択

UART は 115200 baud / 8 data bits / parity none / 1 stop bit / flow control none。
`gamepad_bridge_init()` が `uart_configure()` で設定する。
信号は 3.3 V で、GND を共通化する。

| XIAO ESP32-C3 | RP2350-CAN の例 |
| --- | --- |
| D6 / GPIO21 TX | GP1 / UART0 RX |
| GND | GND |
| D7 / GPIO20 RX | 今回は接続不要 |

application overlay:

```dts
/ {
    aliases {
        gamepad-uart = &uart0;
    };
};

&uart0 {
    /* ESP32 側の UART フレーム仕様に合わせる。 */
    current-speed = <115200>;
    /delete-property/ hw-flow-control;
    status = "okay";
};
```

別の UART を使う場合は alias とその UART の pinctrl を変更する。
ライブラリ自身は alias や GPIO を参照しない。

UART はこの受信機専用とし、console / shell / 他の UART client と共有しない。
有効な UART console / serial shell の chosen device を渡すと `-EBUSY` を返す。
その他の利用者との所有権調整はアプリケーションの責務。
デバッグは RTT を使う。RP2350-CAN サンプルには RTT logging の設定を含めている。

## アプリケーションから使う

公開ヘッダは {download}`gamepad_bridge.h <../include/esp_now_gamepad_bridge/gamepad_bridge.h>`。

```c
#include <esp_now_gamepad_bridge/gamepad_bridge.h>

const struct device *uart = DEVICE_DT_GET(DT_ALIAS(gamepad_uart));
int ret = gamepad_bridge_init(uart);
if (ret < 0) {
    return ret;
}
ret = gamepad_bridge_start();
if (ret < 0) {
    return ret;
}

struct gamepad_state state;
ret = gamepad_bridge_get_state(&state);
if (ret < 0) {
    return ret;
}
if (!state.connected) {
    /* アプリケーション側で機構固有の安全状態を選ぶ。 */
}
```

| API | 契約 |
| --- | --- |
| `gamepad_bridge_init(uart)` | singleton に UART を割り当てて設定。thread context で一度だけ成功させる |
| `gamepad_bridge_start()` | RX 開始。失敗時は再試行可能。成功後の再呼び出しは `-EALREADY` |
| `gamepad_bridge_get_state(&state)` | coherent snapshot をコピーし、返す前に期限切れを反映 |
| `gamepad_bridge_is_connected()` | 未初期化または timeout 後は false |
| `gamepad_bridge_get_stats(&stats)` | coherent snapshot をコピー。カウンタをクリアしない |

init / start は mutex で直列化する。getters と受信・タイマは spinlock で状態を保護する。
getters は通常の ISR からも利用可能。userspace syscall は提供しない。
成功した init の後に UART を差し替える API や stop / deinit は今回提供しない。
init 前の start / snapshot 取得は `-EACCES`、NULL の出力先は `-EINVAL`。
未 ready の UART は `-ENODEV`。設定・callback 登録・RX 開始の失敗は UART の errno を返す。

## Wire format と状態

`COBS(payload || CRC8(payload)) || 0x00` を受信する。
標準 `cobs_decoder_init()` / `cobs_decoder_write()` を使い、
`COBS_FLAG_TRAILING_DELIMITER` で境界を検出する。独自 COBS / CRC 実装は持たない。

decoded length が 11、CRC が一致、kind が `0xA1` のときのみ有効。
CRC は `crc8(payload, 10, 0x07, 0x00, false)` で計算し、最後の 1 byte と比較する。

| byte | フィールド |
| --- | --- |
| 0 | kind = `0xA1` |
| 1–2 | buttons、16 bit little-endian |
| 3–6 | lx / ly / rx / ry、符号付き 8 bit |
| 7–8 | lt / rt、符号なし 8 bit |
| 9 | seq |
| 10 | CRC-8 |

buttons の bit 0–14 は D-pad Left / Up / Right / Down、X、Y、B、A、LB、RB、L3、R3、
View、Menu、Home。bit 15 は予約。LT / RT は buttons に含まれない。
公開 enum の `GAMEPAD_BUTTON_*` で判定する。

軸の送信側の範囲は -127～127、+X は右、+Y は下。trigger は 0～255。
raw byte をそのまま保持し、正規化や機構への変換は行わない。
送信側が予約範囲の軸値 `0x80` を送った場合も int8_t の -128 として保持する。

## Sequence と failsafe

最初の有効フレームは seq にかかわらず採用する。
次からは `(uint8_t)(new_seq - previous_seq)` を計算する。

- 差が 0: duplicate / stall。状態と受信時刻を更新しない。
- 差が 1: 連続した snapshot として採用。255 → 0 もこれに含む。
- 差が 2～255: `差 - 1` を dropped_frames に加え、新しい snapshot を採用する。

逆順や送信機再起動と、大きな欠落は 8 bit seq だけでは区別できない。
仕様どおり modulo 差として計上する。正常な snapshot の利用は継続できる。

最後に採用した frame から設定時間以上が経過すると、buttons / axes / triggers を 0、
connected を false にする。seq は最後の採用値を保持する。
タイマが独立して期限を管理し、アプリケーションの polling は不要。
getter でも期限切れを確認するため、タイマ処理が遅れた場合に古い connected 状態を返さない。

CRC / COBS エラー、長さ・kind の不一致、duplicate、単なる UART bytes では期限を延ばさない。
timeout 後も同じ seq の frame は duplicate とし、異なる seq の正常 frame で復帰する。
タイムアウトは `CONFIG_ESP_NOW_GAMEPAD_BRIDGE_TIMEOUT_MS`（既定 100、20～5000 ms）。
50 Hz 専用の受信周期は持たないので、低い送信頻度を使う場合は timeout も調整する。

## RX バッファと異常からの復帰

async は 64 byte × 2 個の静的バッファを使い、`UART_RX_BUF_REQUEST` に応答する。
`UART_RX_RDY` の offset / len を処理し、`UART_RX_BUF_RELEASED` までは再利用しない。
1 ms の UART idle timeout は chunk 通知用で、link timeout とは別。

`UART_RX_STOPPED` 後の残留データは破棄する。
`UART_RX_DISABLED` で全バッファの解放を確認した後、system workqueue から RX を再開する。
再開できなければ 10 ms 間隔で再試行する。エラー時も watchdog は延長しない。

割り込み方式は FIFO を drain して同じデコーダへ渡し、UART error を検出したら再同期する。
デコーダの格納領域は 11 byte に固定。過長・不正 COBS・UART のデータ欠損時は
次の delimiter まで捨て、それ以降の完全なフレームから再開する。
UART callback 内でプロトコル処理を完了するので、別の無制限 RX queue は持たない。

## 統計

カウンタは uint32_t（オーバーフロー時は modulo 2^32）。

| カウンタ | 意味 |
| --- | --- |
| frames_received | 受信 delimiter 数。空・破棄フレームも含む |
| frames_valid | 長さ・CRC・kind が正常。duplicate も含む |
| cobs_errors / crc_errors / length_errors / kind_errors | それぞれの理由で破棄した数。過長 frame は最初の overflow で一度だけ計上 |
| duplicate_frames | 同一 seq の正常 frame |
| dropped_frames | modulo seq 差から推定した欠落数 |
| timeouts | connected → disconnected の期限切れ遷移数 |
| uart_errors | RX hardware error / 受信 API の失敗数 |
| rx_restarts | 初回 start を除いた async RX 再開成功数 |

## 検証

```sh
west twister -T fibril_zephyr/tests/lib/esp_now_gamepad_bridge --integration
west twister -T fibril_zephyr/samples/lib/esp_now_gamepad_bridge --integration
```

protocol テストは時刻を注入して境界を決定的に検証する。
UART テストは Zephyr `uart_emul` で async / 割り込みを実行し、
fault テストは受信 API の失敗、バッファの貸し出し・解放、停止後の残留データを模擬する。

実機の配線・ビルド手順は {download}`サンプル README <../samples/lib/esp_now_gamepad_bridge/README.md>`。
RP2350-CAN と XIAO ESP32-C3 の接続、RTT 表示、無通信時の neutral 化と再接続は実機で確認する。
