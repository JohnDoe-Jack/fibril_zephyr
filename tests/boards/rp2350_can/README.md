# RP2350-CAN board verification

Zephyr v4.4.1 用のボード統合テスト。Twister はビルドのみを行う。

```sh
west twister -T tests/boards/rp2350_can --integration
west build -p always -b waveshare_rp2350_can/rp2350a/m33 tests/boards/rp2350_can
```

実機実行には GP0–GP1 のジャンパと SWD/RTT 接続が必要。
GP20 / GP21 / GP22 は周辺機器から切り離しておく。

- UART0 の polling TX / RX をジャンパで確認する。
- application overlay の PWM channel 4 / 5 / 6 / 9 へ順に 20 ms 周期・1.5 ms 幅を 100 ms 出力する。
- XL2515 の ready、1 Mbps 設定、標準・拡張フレームの内部 loopback を確認する。
- Flash 容量、CAN alias/chosen、UART console の未使用、RTT、UF2、Classic CAN 構成はビルド時にも確認する。

PWM 波形と外部 CAN バスは別途実測する。
[ボード文書](../../../boards/waveshare/rp2350_can/doc/index.rst) に書き込み・RTT・受け入れ確認の手順がある。
