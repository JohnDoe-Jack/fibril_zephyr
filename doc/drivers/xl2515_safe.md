# XL2515耐障害CANドライバ（`fibril,xl2515-safe`）

Waveshare RP2350-CANのオンボードXL2515を、Zephyr CAN APIから利用するための
MCP2515レジスタ互換ドライバである。CAN障害をPWM、UART、USBなど別の制御経路へ
波及させないことを優先する。

## 障害の封じ込め

- GPIO ISRはsemaphoreを通知するだけで、SPIアクセスを行わない。
- 割り込み処理はプリエンプティブスレッドで実行する。既定優先度は2で、
  優先度0のアプリ制御スレッドより低い。
- 1回の起床で処理する割り込み要因は
  `CONFIG_CAN_XL2515_SAFE_IRQ_DRAIN_LIMIT`（既定32）までに制限する。
- SPI/GPIOエラー、INTが解除されない状態、BUS-OFFを検出するとCANをfault状態へ移す。
- 処理中の送信callbackは`-EIO`または`-ENETUNREACH`で完了させる。
  同期`can_send()`の呼出元も永久待ちにしない。
- `CONFIG_CAN_XL2515_SAFE_RECOVERY_DELAY_MS`（既定100 ms）待ってから、
  soft reset、bit timing、受信設定、動作モードを復元する。
- 復旧に失敗した場合も待機を挟んで再試行するため、CPUを占有し続けない。

fault中の`can_send()`は`-EIO`を返す。復旧前にCANがstart済みだった場合だけ、
復旧後に同じモードでstart状態へ戻す。stop済みのCANを勝手にstartしない。

## Devicetree

```devicetree
&spi1 {
        can0: can-controller@0 {
                compatible = "fibril,xl2515-safe";
                reg = <0>;
                spi-max-frequency = <10000000>;
                int-gpios = <&gpio0 8 GPIO_ACTIVE_LOW>;
                osc-freq = <16000000>;
        };
};
```

`microchip,mcp2515`とは別のcompatibleなので、上流ドライバと同時に同じdeviceを
生成しない。Zephyr本体のソースとバージョンは変更しない。

## 送信側の注意

制御周期から送る場合はcallback付きの非同期送信を使い、失敗をCANだけのfaultとして
扱う。`K_NO_WAIT`は送信完了までの待機を無効にする指定ではない。

```c
ret = can_send(can_dev, &frame, K_NO_WAIT, tx_callback, NULL);
```

state change callbackとTX callbackでは状態記録やwork投入だけを行い、
PWM更新など時間制約のある処理は別スレッドに維持する。

## 自動テスト

`tests/drivers/can/xl2515_safe`はnative_sim上でSPIエラーを注入し、CANワーカが
他スレッドを飢餓させないこと、同期送信が`-EIO`で完了すること、SPI復旧後に
コントローラを再初期化してstart状態へ戻ることを検証する。

```shell
west twister -T fibril_zephyr/tests/drivers/can/xl2515_safe --integration
```

## 検証境界

RP2350向けbuildでは、独自compatibleの解決、Kconfig選択、リンク、loopbackテスト
イメージ生成まで確認できる。実機では次を別途確認する。

1. CAN相手なし、bitrate不一致、終端不良でもUART/PWM周期が継続すること。
2. BUS-OFF後に送信callbackが失敗完了し、遅延復旧へ移ること。
3. SPIまたはINT異常時にもUSB CDCとサーボ制御が止まらないこと。
4. 復旧後に上位制御が意図しない非ゼロ出力を再開しないこと。
