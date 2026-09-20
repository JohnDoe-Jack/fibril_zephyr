.. SPDX-License-Identifier: Apache-2.0

Waveshare RP2350-CAN
====================

RP2350A / Cortex-M33 向けの out-of-tree ボード定義。
Zephyr v4.4.1 を使用し、ターゲットは
``waveshare_rp2350_can/rp2350a/m33`` とする。Hazard3 は対象外。

メモリとクロック
----------------

* 外付け Flash: 4 MiB、XIP アドレス ``0x10000000``、QMI 経由。
* SRAM: 520 KiB、``0x20000000``。SoC 定義を使用する。
* code partition: Flash 全域。ストレージが必要な場合は application overlay で分割する。
* システムクロック: SoC 既定の 150 MHz（12 MHz 水晶から PLL で生成）。
* XL2515: MCU と独立した 16 MHz 水晶。

ピンと既定の状態
----------------

.. list-table::
   :header-rows: 1
   :widths: 22 24 54

   * - 機能
     - GPIO
     - 既定の設定
   * - UART0 TX / RX
     - GP0 / GP1
     - 有効、115200 baud、8N1、フロー制御なし
   * - UART0 CTS / RTS
     - GP2 / GP3
     - ``uart0_default_hwflow`` に定義。使用時に overlay で選択
   * - XL2515 IRQ / CS
     - GP8 / GP9
     - ともに active low。CS は GPIO 制御
   * - SPI1 SCK / MOSI / MISO
     - GP10 / GP11 / GP12
     - XL2515 用に有効、要求する最大 SCK は 10 MHz
   * - PWM2A / PWM2B
     - GP20 / GP21
     - PWM channel 4 / 5
   * - PWM3A
     - GP22
     - PWM channel 6
   * - PWM4B / onboard LED
     - GP25
     - PWM channel 9、``pwm-led0`` / ``led0`` alias

UART0 は ``zephyr,console`` / ``zephyr,shell-uart`` に指定しない。
UART API の polling と interrupt-driven API が使用できる構成とする。
STM32 用の ``CONFIG_UART_ASYNC_API=y`` を共通設定から持ち込まない。
ハードウェアフロー制御が必要なら、アプリケーションで次を指定する。

.. code-block:: dts

   &uart0 {
       pinctrl-0 = <&uart0_default_hwflow>;
       hw-flow-control;
   };

PWM peripheral と上記 4 ピンの pinctrl を既定で有効にする。
PWM の機器割り当て、周期、パルス幅はアプリケーションが決める。
例えば GP20 を参照するだけなら、overlay に次を追加できる。

.. code-block:: dts

   #include <zephyr/dt-bindings/pwm/pwm.h>

   / {
       zephyr,user {
           pwms = <&pwm 4 PWM_MSEC(20) PWM_POLARITY_NORMAL>;
       };
   };

``PWM_DT_SPEC_GET(DT_PATH(zephyr_user))`` で取得できる。
同じ PWM slice の A/B（GP20 / GP21）は周期と分周器を共有するため、同一周期で使う。
分周器はドライバの自動選択に任せる。
GP25 の GPIO LED と PWM LED は同じ端子なので、同時に駆動しない。
PWM を使わず GPIO として利用するときは ``&pwm { status = "disabled"; };``
または必要なピンだけの pinctrl を application overlay に指定する。

CAN
---

オンボード XL2515 は ``fibril,xl2515-safe`` として登録する。
``DT_ALIAS(can0)``、``DT_CHOSEN(zephyr_canbus)``、
``DT_NODELABEL(xl2515)``、``DT_NODELABEL(can0)`` は同じデバイスを指す。
``CONFIG_CAN=y`` と SPI / XL2515 耐障害ドライバを既定で有効にする。
割り込み処理回数を制限し、SPI/GPIOエラー、INT張り付き、BUS-OFF時は
CAN送信を失敗完了させて低優先度のプリエンプティブスレッドで再初期化する。
したがってCAN故障によって他の制御スレッドを協調スレッド内で占有し続けない。

Classic CAN 2.0B の標準 ID / 拡張 ID に対応し、最大 1 Mbps とする。
``can_set_bitrate(dev, 1000000)`` は ``can_start()`` の前に呼ぶ。
起動時から 1 Mbps を使う場合は ``&xl2515 { bitrate = <1000000>; };`` を指定する。
CAN FD は使用しない。

既存の ``app/`` は RoboMaster ノード、STM32 の非同期 UART、CAN FD の設定を持つため、
本ボードの最小検証には以下の専用テストまたは Zephyr 標準サンプルを使用する。
``apps/node`` の fibril_can 通信は CAN FD を前提とする別のアプリケーションである。
RoboMaster の機器定義やアプリケーションへの組み込みは :doc:`/drivers/robomaster` を参照。

ビルド
------

workspace の manifest を ``fibril_zephyr/west.yml`` にした通常の配置を前提とする。
``west update`` により ``hal_rpi_pico`` と RTT 用 ``segger`` が導入される。
Bridle は依存に追加しない。
workspace のルートから実行する。

.. code-block:: console

   west build -p always -b waveshare_rp2350_can/rp2350a/m33 zephyr/samples/hello_world -d build/rp2350-hello
   west build -p always -b waveshare_rp2350_can/rp2350a/m33 fibril_zephyr/tests/boards/rp2350_can -d build/rp2350-test
   west twister -T fibril_zephyr/tests/boards/rp2350_can --integration

``build/<name>/zephyr/`` に ``zephyr.elf``、``zephyr.hex``、``zephyr.uf2`` が生成される。
worktree で検証する際のモジュール取り違え対策はリポジトリの ``CLAUDE.md`` に従う。

書き込みと RTT
--------------

BOOT ボタンを押した状態で USB 接続し、ROM の USB ドライブへ ``zephyr.uf2`` をコピーする。
ドライブがマウントされている環境では ``west flash -d build/rp2350-hello --runner uf2`` も使用できる。
通常動作中の USB CDC コンソールはこの定義には含めない。

RTT console を既定で有効にし、``printk()`` の出力も UART を使わず取得できる。
Zephyr logging を使うアプリケーションは ``CONFIG_LOG=y`` を指定する。
``CONFIG_LOG_BACKEND_RTT`` はそのとき既定で有効となり、UART backend は既定で無効になる。
シェルが必要な場合はアプリケーションで ``CONFIG_SHELL_BACKEND_RTT=y`` を選ぶ。

SWDIO、SWCLK、GND をデバッグプローブへ接続する。
J-Link の場合は ``RP2350_M33_0`` を選び、RTT Viewer の terminal channel 0 からログを取得する。
OpenOCD runner は Raspberry Pi の RP2350 対応版を前提とし、既定のアダプタは
``cmsis-dap``。変更する場合は CMake 引数
``-DWAVESHARE_RP2350_DEBUG_ADAPTER=<interface>`` を指定する。

実機の受け入れ確認
------------------

ビルド成功だけでは配線、XL2515 の互換性、外部 CAN バス、RTT 接続は確認できない。
以下を実機で確認する。

#. 専用テストでは GP0 と GP1 を接続し、GP20 / GP21 / GP22 を周辺機器から切り離す。
#. テストの UF2 を書き込み、RTT で ztest の結果を取得する。
   UART の TX/RX、PWM 4 チャンネルの API、XL2515 の ready と
   1 Mbps の内部 loopback（標準 ID / 拡張 ID）を確認する。
#. オシロスコープ等で GP20 / GP21 / GP22 / GP25 を観測する。
   テストは各端子へ順に周期 20 ms / パルス幅 1.5 ms を 100 ms 出力して停止する。
#. 終端を設定した CAN バスに別の CAN ノードを接続する。
   Zephyr の ``samples/drivers/can/counter`` は既定で内部 loopback になるため、
   ``CONFIG_LOOPBACK_MODE=n`` を明示して通常モードでビルドする。
   ``&xl2515 { bitrate = <1000000>; };`` の overlay も渡す。
   対向を 1 Mbps に合わせ、標準 ID / 拡張 ID の双方が双方向に届くことを確認する。
   内部 loopback の成功はトランシーバや CANH/CANL の確認を代替しない。

移植元とライセンス
------------------

Based on:
https://github.com/tiacsys/bridle/tree/ec0891f52844c8da1e35e15ddf1fc7ec4b96ba42/boards/waveshare/rp2350_can

同ディレクトリの ``board.yml``、``board.cmake``、``Kconfig.defconfig``、
``Kconfig.waveshare_rp2350_can``、``waveshare_rp2350_can.dtsi``、
``waveshare_rp2350_can-pinctrl.dtsi``、および ``waveshare_rp2350_can_rp2350a_m33``
の ``.dts`` / ``.yaml`` / ``_defconfig`` を基にしている。
TiaC Systems の copyright と Apache-2.0 SPDX を各移植ファイルに保持する。
ライセンス本文はリポジトリの ``LICENSE`` に含む。

fibril_zephyr 向けの変更点:

* Cortex-M33 のみを移植し、Bridle 固有の connector binding と対象外 peripheral の設定を省く。
* Bridle の Flash partition include を v4.4.1 の fixed-partitions 記法で置き換え、
  code 領域を 1 MiB から 4 MiB にする。3 MiB の storage 予約は持ち込まない。
* UART console / shell の chosen を削除し、RTT console / logging を既定にする。
* SPI1 の GPIO CS を pinmux から除外し、CAN alias を追加する。
* PWM slice 4 の固定分周を省き、アプリケーションが指定する周期に追従させる。
* Kconfig の既定値を本ボードに限定し、v4.4.1 にあるシンボルだけを使用する。
* M33 の OpenOCD / J-Link と UF2 runner に絞り、デバッグ時に ROM の初期化を行う。
