#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include "driver/spim00_bus.h"
#include "inc/sd_fs_test.h"
#include "inc/stim_test.h"

#include "neuro/inc/neuro_ctrl.h"

LOG_MODULE_REGISTER(main_app, LOG_LEVEL_INF);

int main(void)
{
  int err;

  k_msleep(300); /* Wait for power to stabilize */

  printk("\n>>> main alive (printk to RTT; UART console disabled)\n");

  /* One-time SPIM00 (nrfx) init before any EEG / stim / SD task touches the bus (idempotent). */
  err = spim00_bus_init();
  if (err != 0) {
    LOG_ERR("spim00_bus_init failed (%d)", err);
  }

  /*
   * SPI / bus stress layout (adjust priorities vs. sd_fs_test / driver mutex):
   *   sd_fs_test  — mount + FAT I/O on SPIM00 (prio 3, sd_fs_test.c)
   *   eeg_test    — eeg_manager timer + frame reads (prio 5, eeg_test.c)
   *   stim_test   — DAC / neuro waveform on SPI (prio 6, stim_test.c)
   *
   * Real concurrency correctness depends on spim00_bus (or Zephyr spi00)
   * arbitration and chip-select isolation; these threads deliberately overlap.
   */
  // LOG_INF("=== start SD FAT smoke thread ===");
  // err = sd_fs_test_start();
  // if (err != 0) {
  //   LOG_ERR("sd_fs_test_start failed (%d)", err);
  // }

  // LOG_INF("=== start EEG test thread ===");
  // err = eeg_test_start();
  // if (err != 0) {
  //   LOG_ERR("eeg_test_start failed (%d)", err);
  // }

  // LOG_INF("=== start stim test thread ===");
  // err = stim_test_start();
  // if (err != 0) {
  //   LOG_ERR("stim_test_start failed (%d)", err);
  // }

  /*
   * Enables neuro_ctrl_thread_main idle→run: sets running=true after neuro_ctrl_init.
   * Idempotent init if stim_test_start already succeeded; retries init if stim failed earlier.
   * Note: simultaneous stim_test DAC streaming and neuro wave_ctrl DAC use must be coordinated.
   */
  err = neuro_ctrl_start();
  if (err != 0) {
    LOG_ERR("neuro_ctrl_start failed (%d)", err);
  } else {
    LOG_INF("neuro_ctrl main loop enabled (running=true)");
  }

  for (;;) {
    //LOG_INF("keep working :(");
    k_sleep(K_FOREVER);
  }
//never reach here
  return 0;
}
