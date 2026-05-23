#pragma once

/**
 * Spawn a thread that exercises eeg_manager (frame timer + read path).
 *
 * Intended to run alongside SPI-heavy tasks (SD, stim DAC) while you tune
 * spim00 bus sharing / mutex sequencing.
 */
int eeg_test_start(void);
