# SPDX-License-Identifier: Apache-2.0

# 4000 can fail on long SWD wires or marginal boards; raise if your wiring is short and stable.
board_runner_args(jlink "--device=nRF54L15_M33" "--speed=1000")

if(CONFIG_BOARD_MINNEURO_RHS_DEVKIT_NRF54L15_CPUAPP_NS)
  set(TFM_PUBLIC_KEY_FORMAT "full")
endif()

if(CONFIG_TFM_FLASH_MERGED_BINARY)
  set_property(TARGET runners_yaml_props_target PROPERTY hex_file tfm_merged.hex)
endif()

# J-Link first so `west flash` / ./flash.sh default to J-Link without nrfutil-device.
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/nrfutil.board.cmake)