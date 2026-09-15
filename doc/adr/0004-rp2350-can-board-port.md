---
status: Accepted
date: 2026-09-15
---

# 0004. RP2350-CAN は標準 Zephyr 上の Classic CAN ボードとして移植する

## Status

Accepted

## Context

RP2350-CAN の XL2515 は Classic CAN のコントローラであり、既存の `app/` は
CAN FD、STM32 の非同期 UART、RoboMaster の機器定義を要求する。
Bridle に参考実装はあるが、今回の仕様は Zephyr v4.4.1 を維持し、
UART0 をアプリケーション通信に使える状態にすることを求めている。

## Decision

- Bridle の Cortex-M33 用ボード定義を Apache-2.0 で移植する。
  出典の固定コミットと変更点は [ボード文書](../boards/waveshare_rp2350_can.rst) に記録する。
- `hal_rpi_pico` と RTT 用の `segger` を Zephyr module allowlist に追加する。
  revision は Zephyr v4.4.1 の manifest に従う。
- UART の console / shell chosen を持たせず、RTT console と logging backend を使用する。
- 4 MiB Flash は全体を code partition として公開する。
  Bridle の 3 MiB storage 予約は、アプリケーションの用途が未決定なので引き継がない。
- UART / PWM の機器割り当てを board に含めない。
  PWM の分周器はドライバの自動選択とし、周期をアプリケーションで決められるようにする。
- 持ち込み検証は `tests/boards/rp2350_can` と Zephyr 標準サンプルで行う。
  Twister ではビルドのみ、配線と外部 CAN 通信は実機で検証する。

## Consequences

既存の CAN FD アプリケーションの設定を変えずに Classic CAN ボードを追加できる。
RoboMaster などの機器を使用する場合は、アプリケーション側で overlay と設定を用意する。
RTT の取得には SWD プローブが必要になる。

## 却下した選択肢

- **Bridle または独自 Zephyr fork を依存に追加する。** 指定バージョンと依存範囲を維持できない。
- **既存 `app/` の共通設定から CAN FD を無効化する。** 他のボードの動作を変更してしまう。
- **UART0 をログと共用する。** アプリケーション通信を占有しないという要件を満たさない。
