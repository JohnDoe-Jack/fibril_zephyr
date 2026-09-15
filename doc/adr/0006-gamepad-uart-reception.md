# 0006: Gamepad UART の共通プロトコルと受信方式

- ステータス: Accepted

## 背景

ESP32 の COBS / CRC-8 gamepad snapshot を、アプリケーションが coherent な状態として
利用する必要がある。仕様は UART async を想定しているが、Zephyr v4.4.1 の RP2350
PL011 は async を実装していない。

## 決定

- `lib/esp_now_gamepad_bridge/` を通信層として追加し、singleton の公開 API にする。
- async と割り込みの 2 バックエンドを Kconfig choice で選ぶ。
  RP2350-CAN は割り込み、対応 UART では async を使う。
- Zephyr streaming COBS / CRC API を使う共通 state machine に、時刻を引数で渡す。
  単体テストは sleep なしで timeout の境界を検証できる。
- デコード領域は 11 byte に固定し、UART 欠損や overflow 後は delimiter まで破棄する。
- 新しい正常 seq だけが watchdog を更新し、timeout 後も最後の seq を記憶する。
- snapshot と統計は spinlock で保護する。受信 callback 内で小さいフレームを処理し、
  watchdog は独立した kernel timer と getter の両方で期限を確認する。
- `COBS` は application が明示的に有効化する。
  Zephyr v4.4.1 で SERIAL 依存のライブラリから select すると、NET_BUF / logging /
  serial shell 経由の Kconfig 循環が生じるため、`depends on COBS` とする。

## 却下した案と帰結

PL011 に独自 async 実装や DMA 対応を加える案は、今回の通信層の範囲を超える。
アプリケーションはどちらの RX バックエンドでも同じ API を使える。
タイマの更新を単なる UART 受信や duplicate に結び付けると、送信側の stall を検出できない。
timeout 時に seq を忘れると同じ frame の繰り返しで復帰するため、その案も採用しない。
受信停止後は workqueue で RX を再開し、ドライバ所有中のバッファを再利用しない。
モータ・サーボの停止や安全角はアプリケーションに残す。
