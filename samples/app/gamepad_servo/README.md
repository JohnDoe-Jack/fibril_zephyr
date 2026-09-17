# Gamepad-controlled PWM servo

Waveshare RP2350-CANで`esp-now-gamepad-bridge`からUART受信したD-pad入力を使い、
RCサーボをゆっくり動かす統合サンプル。

## 配線

ここでいうピン番号は、RP2350-CANのPico互換40ピンヘッダの**物理ピン番号**。

| 接続先 | RP2350-CAN | MCU機能 |
| --- | --- | --- |
| サーボ信号 | 22番ピン | GP17 / PWM0B |
| ESP32のUART TX | 27番ピン | GP21 / UART1 RX |
| ESP32 GND | GND | GND |
| サーボ GND | GND | GND |

UARTは115200 baud / 8N1。ESP32側のRXは接続しない。
サーボは定格に合う外部電源から給電し、RP2350-CAN、ESP32、サーボのGNDを共通化する。
サーボ電源をRP2350-CANの3.3 V端子から取らない。

## 動作

- 起動時は1500 us（7.50%）を出力する。
- D-pad UPを押している間、10 msごとに5 usずつパルス幅を増やす。
- D-pad DOWNを押している間、同じ速度でパルス幅を減らす。
- UP/DOWNの同時押し、またはどちらも押していないときは現在位置を保持する。
- 出力は500--2500 us（2.50--12.50%）に制限する。
- 有効なUART frameが100 ms届かなければ1500 usへ戻す。
- 250 msごとに接続状態、ボタン、パルス幅、dutyをUSB CDC ACMへ表示する。

全範囲の移動には約4秒かかる。速度を変更する場合は`src/main.c`の
`PULSE_STEP_US`または`CONTROL_PERIOD_MS`を調整する。

## ビルドと書き込み

workspaceルートから実行する。

```shell
west build -p always -b waveshare_rp2350_can/rp2350a/m33 \
  fibril_zephyr/.claude/worktrees/gamepad-servo/samples/app/gamepad_servo \
  -d build/gamepad-servo -- \
  -DZEPHYR_EXTRA_MODULES=/home/iwasakim/zephyrproject/fibril_zephyr/.claude/worktrees/gamepad-servo
west flash -d build/gamepad-servo --runner uf2
```

または、BOOTボタンを押したままUSB接続し、
`build/gamepad-servo/zephyr/zephyr.uf2`をROMのUSBドライブへコピーする。

書き込み後にBOOTボタンを離して再起動すると、Linux上にUSBシリアル
`/dev/ttyACM0`などが現れる。たとえばworkspaceのPython環境から次のように開く。

```shell
/home/iwasakim/zephyrproject/.venv/bin/python -m serial.tools.miniterm \
  /dev/ttyACM0 115200
```

USB CDCでは115200の設定値は実際のUSB通信速度を変えない。複数の`ttyACM`がある場合は
`ls -l /dev/serial/by-id/`で対象を確認する。サーボ制御は端末の接続を待たずに開始し、
端末を後から開いても250 ms周期の現在値を確認できる。

ログ例:

```text
<inf> gamepad_servo: connected=1 seq=42 buttons=0x0002 pulse=1530 us duty=7.65%
```
