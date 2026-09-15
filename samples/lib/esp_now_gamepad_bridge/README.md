# ESP-NOW gamepad UART bridge

XIAO ESP32-C3 の `esp-now-gamepad-bridge` から届く状態を受信し、
200 ms ごとに接続状態、buttons、sticks、LT / RT、seq と統計を RTT に表示する。
UART へは送信しない。

## RP2350-CAN の配線

| XIAO ESP32-C3 | RP2350-CAN |
| --- | --- |
| D6 / GPIO21（TX、3.3 V） | GP1（UART0 RX） |
| GND | GND |

D7 / GPIO20（ESP32 RX）への接続は不要。
UART0 は 115200 baud / 8N1 / no flow control で使い、console / shell と共有しない。
RTT ログの確認には SWD デバッグプローブを使用する。

## ビルド

通常の west workspace のルートで実行する。

```sh
west build -b waveshare_rp2350_can/rp2350a/m33 \
  fibril_zephyr/samples/lib/esp_now_gamepad_bridge -d build-gamepad
```

`build-gamepad/zephyr/zephyr.uf2` が生成される。
このボードの Zephyr v4.4.1 PL011 は async API 未対応なので、
サンプルの board conf で割り込み受信を選択する。
async 対応ボードでは `CONFIG_ESP_NOW_GAMEPAD_BRIDGE_ASYNC=y` とし、
`gamepad-uart` alias、UART pinctrl と RTT 出力をボードに合わせて設定する。

## 動作確認

1. ESP32 側でゲームパッドの bridge を起動し、UART を接続する。
2. RTT の `connected=1`、button bit、4 軸、LT / RT、増加する seq を確認する。
3. ESP32 TX を外すと、既定 100 ms で内部状態が neutral / disconnected になる。
   サンプルの表示更新は 200 ms 周期なのでログには次の周期で現れる。
4. 再接続し、前回と異なる seq の正常フレームで connected に復帰することを確認する。

ライブラリはモータ・サーボ・CAN を操作しない。
詳細は [ライブラリ文書](../../../doc/gamepad_bridge.md) を参照。
