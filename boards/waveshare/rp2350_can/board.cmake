# SPDX-FileCopyrightText: Copyright (c) 2025-2026 TiaC Systems
# SPDX-License-Identifier: Apache-2.0
# Modified for fibril_zephyr; see doc/index.rst for provenance and porting notes.

if("${WAVESHARE_RP2350_DEBUG_ADAPTER}" STREQUAL "")
  set(WAVESHARE_RP2350_DEBUG_ADAPTER "cmsis-dap")
endif()

board_runner_args(openocd --cmd-pre-init "source [find interface/${WAVESHARE_RP2350_DEBUG_ADAPTER}.cfg]")
board_runner_args(openocd --cmd-pre-init "transport select swd")
board_runner_args(openocd --cmd-pre-init "source [find target/rp2350.cfg]")
board_runner_args(openocd --cmd-pre-init "set_adapter_speed_if_not_set 2000")
# Let the RP2350 boot ROM initialize its redundancy coprocessor before debugging.
board_runner_args(openocd --gdb-pre-debug "monitor reset init")
board_runner_args(jlink "--device=RP2350_M33_0")
board_runner_args(uf2 "--board-id=RP2350")

board_set_flasher_ifnset(uf2)
board_set_debugger_ifnset(openocd)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/uf2.board.cmake)
