# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hello_world)

if(TARGET modules__fatfs)
  get_filename_component(FATFS_MODULE_DIR "$ENV{ZEPHYR_BASE}/../modules/fs/fatfs" ABSOLUTE)
  set_property(TARGET modules__fatfs PROPERTY SOURCES
    ${FATFS_MODULE_DIR}/ff.c
    ${FATFS_MODULE_DIR}/option/ffunicode.c
  )
  target_compile_definitions(modules__fatfs PRIVATE
    ZEPHYR_CONFIG_OVERRIDE=file_fatfs_config.h
  )
endif()

FILE(GLOB app_sources CONFIGURE_DEPENDS
  src/*.c
  src/inc/*.h
  neuro/*.c
  neuro/inc/*.h
  driver/*.c
  driver/*.h
)
# Zephyr SPI DAC shim — unused: stim uses dac80501.c + dac80501_hal via spim00_bus only.
list(FILTER app_sources EXCLUDE REGEX ".*/dac_dac80501_spi\\.c$")
target_sources(app PRIVATE ${app_sources} src/main.c)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
