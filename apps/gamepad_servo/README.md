# UART gamepad + CAN controlled PWM servo

Waveshare RP2350-CANで、`esp-now-gamepad-bridge`からUART受信したD-pad入力、
またはClassic CANの`can_servo`互換コマンドを使ってRCサーボを操作する統合サンプル。
CAN FDおよび`fibril_can`は使用しない。

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

CANはオンボードXL2515を1 MbpsのClassic CAN 2.0Bとして使用する。
CAN_H/CAN_Lを対向ノードへ接続し、バス両端を適切に終端する。
CANの起動に失敗した場合はエラーを記録し、UART/PWM制御は継続する。

## UARTゲームパッド制御

- 起動時は1210 usを出力する。
- D-pad UPを押している間、10 msごとに5 usずつパルス幅を増やす。
- D-pad DOWNを押している間、同じ速度でパルス幅を減らす。
- UP/DOWNの同時押し、またはどちらも押していないときは現在位置を保持する。
- 出力は1000--1400 usに制限する。
- UARTが制御権を持つ状態で有効なframeが100 ms届かなければ1210 usへ戻す。
- CANが制御権を持つ間は、UART timeoutでcenterへ戻さない。

## CANサーボ制御

サーボIDは1固定で、Standard CAN frameだけを受理する。

| 用途 | CAN ID | DLC / payload |
| --- | --- | --- |
| 個別command | `0x301` | 下記参照 |
| broadcast detach | `0x300` | `02` |
| status | `0x381` | 5 byte、20 Hzおよび要求時 |

Command payload:

- SetPosition: `00 AA AA SS SS`。角度はsigned little-endian、0.1 deg/LSB。速度はunsigned little-endian、0.1 deg/s/LSB。速度0は制限なし。
- Hold: `01`。現在のPWM指令角を保持する。
- Detach: `02`。PWM出力を停止する。
- RequestStatus: `03`。次のmain loopでstatusを送る。

SetPositionはdetach後のPWMを再開し、指定速度で現在指令角からtargetへslewする。
ローカル範囲外の角度は0--180 degへclampされる。CAN commandにwatchdogはなく、
新しいcommandがない場合は最後の指令を保持する。

Statusのbyte 0はstate（0=Holding、1=Moving、2=Detached）、bit 4がclamped、
bit 5がrejected。byte 1--2は受理target、byte 3--4は現在のPWM指令角で、
いずれもsigned little-endian、0.1 deg/LSB。RCサーボの実軸角フィードバックではない。

UARTとCANを同時操作した場合はlast-command-winsとする。各10 ms loopではCAN queueを先に処理し、
そのloopでD-padが実際にpulseを変更した場合はUARTが最後のcommandになる。

例:

```shell
cansend can0 301#008403A00F  # 90.0 deg, 400.0 deg/s
cansend can0 301#0008072C01  # 180.0 deg, 30.0 deg/s
cansend can0 301#01          # Hold
cansend can0 301#02          # Detach
cansend can0 300#02          # Broadcast detach
cansend can0 301#03          # RequestStatus
```

## LEDとログ

ゲームパッド接続中はLEDが500 msごとに反転し、切断中は点灯する。
250 msごとにcontrol source、ゲームパッド状態、pulse、CAN target、現在指令角、
Holding/Moving/Detached、およびCAN送信失敗回数をUSB CDC ACMへ表示する。

## ビルドと書き込み

workspaceルートから実行する。

```shell
west build -p always -b waveshare_rp2350_can/rp2350a/m33 \
  fibril_zephyr/apps/gamepad_servo \
  -d build/gamepad-servo -- \
  -DZEPHYR_EXTRA_MODULES=$PWD/fibril_zephyr
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
`ls -l /dev/serial/by-id/`で対象を確認する。
