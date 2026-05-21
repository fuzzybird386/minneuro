#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "neuro/inc/eeg_algo.h"
#include "neuro/inc/neuro_stim_bank.h"
#include "neuro/inc/sleep_fusion.h"

typedef enum {
  NEURO_CTRL_PHASE_IDLE = 0,
  NEURO_CTRL_PHASE_EEG_ACQUIRE,
  NEURO_CTRL_PHASE_ANALYZE,
  NEURO_CTRL_PHASE_STIMULATE,
  NEURO_CTRL_PHASE_SKIP_STIM,
} neuro_ctrl_phase_t;

typedef struct {
  uint32_t cycle_index;
  neuro_ctrl_phase_t phase;
  bool running;
  bool last_cycle_stimulated;
  eeg_algo_result_t last_eeg_result;
  sleep_fusion_result_t last_fusion_result;
  neuro_stim_request_t last_stim_request;
} neuro_ctrl_state_t;

int neuro_ctrl_init(void);
int neuro_ctrl_start(void);
int neuro_ctrl_stop(void);
int neuro_ctrl_get_state(neuro_ctrl_state_t *state);
//暂时引出switch函数，用于stim_test.c中
void neuro_ctrl_set_switch(bool eeg_mode);
