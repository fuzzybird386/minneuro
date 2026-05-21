#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  SLEEP_STAGE_UNKNOWN = 0,
  SLEEP_STAGE_W,
  SLEEP_STAGE_N1,
  SLEEP_STAGE_N2,
  SLEEP_STAGE_N3,
  SLEEP_STAGE_REM,
} sleep_stage_t;

typedef struct {
  sleep_stage_t stage;
  float confidence;
} sleep_stage_result_t;

