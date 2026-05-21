#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "inc/sd_fs_test.h"

LOG_MODULE_REGISTER(main_app, LOG_LEVEL_INF);

int main(void)
{
  int err;

  k_msleep(300); /* Wait for power to stabilize */

  /*
   * SD filesystem smoke test (see sd_fs_test.c). Shares SPIM00 with other
   * devices via spim00_bus; do not run heavy stim SPI traffic in parallel
   * without sequencing.
   */
  LOG_INF("=== SD FAT smoke test thread ===");

  err = sd_fs_test_start();
  if (err != 0) {
    LOG_ERR("sd_fs_test_start failed (%d)", err);
  }

  for (;;) {
    LOG_INF("=== i am driving :)===");
    k_msleep(5000);
  }
//never reach here
  return 0;
}
