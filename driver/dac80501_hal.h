#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

int dac80501_hal_init(void);
int dac80501_hal_enable(bool enable);
int dac80501_hal_write(const uint8_t *tx, size_t len);
