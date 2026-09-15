# Generic PWM servo

起動から 3 秒後に、Devicetree の最小・中央・最大パルスを各 2 秒出力し、
角度範囲の中点を 2 秒出力してから停止する。一巡で終了する。
PWM エラー時も出力停止を試み、失敗をログに残す。

## RP2350-CAN

通常の west workspace で実行する。

```sh
west build -b waveshare_rp2350_can/rp2350a/m33 fibril_zephyr/samples/drivers/pwm_servo
west flash
```

- 信号線: GP20（PWM channel 4）
- 電源: サーボの定格に合う外部電源を使い、GND を基板と共通にする。
- 設定: 20 ms 周期、500 / 1500 / 2500 us、0～180 度。
- ログ: RTT。UART0 はログに使わない。

サンプルは端まで動かすので、使用するサーボの仕様に overlay のパルス幅・角度を合わせ、
機構を外して確認する。ロジックアナライザで各パルスと、終了後の Low 固定を確認する。

別のボードでは `servo0` alias と `fibril,pwm-servo` ノードを application overlay に定義し、
そのボードの PWM と pinctrl を有効にする。ドライバの変更は不要。

詳しい API、エラー、複数台の設定は [ドライバ文書](../../../doc/drivers/pwm_servo.md) を参照。
