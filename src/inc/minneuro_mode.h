#pragma once

/*
 * Global build-time feature switches.
 *
 * 0: disable optional BLE debug service
 * 1: enable optional BLE debug service
 *
 * The normal product path always remains enabled. This switch only controls
 * whether the extra debug BLE service is added on top.
 */
#ifndef MINNEURO_DEBUG_MODE
#define MINNEURO_DEBUG_MODE 1
#endif

/*
 * Stim waveform bench-test switch.
 *
 * 0: run the normal neuro controller
 * 1: on boot, continuously output the stim test sequence from main.c
 *
 * The current default keeps hardware bring-up easy. Production builds should
 * override this to 0.
 */
#ifndef MINNEURO_STIM_TEST_MODE
#define MINNEURO_STIM_TEST_MODE 1
#endif
