# PWM RC サーボ

`fibril,pwm-servo` は Zephyr 標準 PWM API を使う RC サーボドライバ。
1 ノードが 1 台を表し、台数・GPIO・ボード固有 API を実装に持たない。

## 設定と配線

`CONFIG_PWM=y` と `CONFIG_SERVO=y` を設定する。
`CONFIG_SERVO_INIT_PRIORITY` の既定値は 85。PWM controller より後で初期化する。
ログは `CONFIG_SERVO_LOG_LEVEL` で制御できる。

RP2350-CAN の application overlay の例:

```dts
#include <zephyr/dt-bindings/pwm/pwm.h>

/ {
    servo0: servo0 {
        compatible = "fibril,pwm-servo";
        /* GP20 / PWM2A。使用サーボの基準フレームは 50 Hz。 */
        pwms = <&pwm 4 PWM_MSEC(20) PWM_POLARITY_NORMAL>;
        /* 使用サーボの信号仕様。機構に合わせて範囲を狭める。 */
        min-pulse-us = <500>;
        center-pulse-us = <1500>;
        max-pulse-us = <2500>;
        min-angle-deg = <0>;
        max-angle-deg = <180>;
    };

    servo1: servo1 {
        compatible = "fibril,pwm-servo";
        /* GP21 / PWM2B は GP20 と周期を共有するため同じ 20 ms。 */
        pwms = <&pwm 5 PWM_MSEC(20) PWM_POLARITY_NORMAL>;
        /* このサーボは中央を 0 度とする座標系で扱う。 */
        min-pulse-us = <500>;
        center-pulse-us = <1500>;
        max-pulse-us = <2500>;
        min-angle-deg = <(-90)>;
        max-angle-deg = <90>;
    };
};
```

PWM 信号線と共通 GND を接続し、サーボは定格に合う外部電源から給電する。
標準の RC サーボ信号には `PWM_POLARITY_NORMAL` を使う。
`pwms` の周期は ns、パルスのプロパティと API 引数は us。
500 / 1500 / 2500 us は High 時間で、20 ms 周期では 2.5 / 7.5 / 12.5 % となる。

すべてのプロパティは必須。各ノードの `pwms` はちょうど 1 チャネルとする。
サーボの割り当て・校正は application overlay に置く。
RP2350 では同じ slice の A/B が周期を共有する。異なる周波数の機器は別 slice に割り当てる。
このドライバは共有資源の仲裁や PWM 周期の競合検出を行わない。

## API と起動時の状態

公開ヘッダは `include/drivers/servo.h`。supervisor mode から使用する。

```c
#include <drivers/servo.h>

const struct device *servo = DEVICE_DT_GET(DT_NODELABEL(servo0));

if (!device_is_ready(servo)) {
    return -ENODEV;
}
int ret = servo_set_angle(servo, 90.0f);
if (ret < 0) {
    return ret;
}
```

| API | 動作 |
| --- | --- |
| `servo_set_angle(dev, angle_deg)` | 有限角度を範囲内にクランプし、最小・最大間を線形変換する |
| `servo_set_pulse(dev, pulse_us)` | us 単位で指定し、最小・最大間にクランプする |
| `servo_disable(dev)` | 設定周期・極性を保ち、パルス幅 0 で inactive にする |

角度変換は `min_pulse + (angle - min_angle) * (max_pulse - min_pulse) / (max_angle - min_angle)`。
最も近い整数 us に丸め、ちょうど 0.5 us は切り上げる。実際の出力分解能は PWM controller に依存する。
`center-pulse-us` は校正用の参照値で、角度変換の第 3 点ではない。
非対称な校正で中央パルスを出したい場合は `servo_set_pulse()` に渡す。

初期化は設定と PWM の ready 状態を検証してからパルス幅 0 を設定する。
起動時の中央移動は行わない。最初の角度・パルス指定で出力を開始し、停止後も同じ API で再開する。
`servo_set_pulse(dev, 0)` は最小パルスへクランプされるため、停止には `servo_disable()` を使う。
停止は信号を inactive にするだけで、電源断やトルクの解放を保証しない。
同じチャネルに対する並行コマンドの順序は呼び出し側で管理する。

## 不正設定と失敗

初期化時、以下は `-EINVAL` となりデバイスは ready にならない。

- PWM 指定が 1 チャネル以外、周期が 0
- `min-pulse-us == 0`、または `min-pulse-us >= max-pulse-us`
- 中央パルスが最小・最大の範囲外
- `min-angle-deg >= max-angle-deg`（角度は符号付き 32 bit）
- 最大パルスの ns 換算が uint32_t に収まらない、または周期を超える

PWM controller が ready でなければ `-ENODEV`。
PWM API の失敗はそのまま返す。初期化時の停止に失敗したデバイスも ready にならない。
公開 API に NULL または初期化失敗したデバイスを渡すと `-ENODEV` を返す。
NaN / ±Inf の角度は `-EINVAL` で、PWM 出力を変更しない。
範囲外の有限角度・パルスはエラーではなくクランプする。
エラー時に別の角度へ復帰する処理や自動再試行は行わない。

## 検証

```sh
west twister -T fibril_zephyr/tests/drivers/servo/pwm --integration
west twister -T fibril_zephyr/samples/drivers/pwm_servo --integration
```

native_sim の 32 / 64 bit で実ドライバと Zephyr の fake PWM を組み合わせ、
変換・クランプ・複数インスタンス・初期状態・停止再開・不正設定・エラー伝播を検証する。

実機では {download}`PWM サーボサンプルの手順 <../../samples/drivers/pwm_servo/README.md>` を使用する。
オシロスコープまたはロジックアナライザで 20 ms 周期の 500 / 1500 / 2500 us と
停止後の Low 固定を観測し、少なくとも 1 台のサーボの動作を確認する。
ビルドや native_sim の成功だけでは実機の受け入れ条件を満たさない。
