#include "inc/eeg_test.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <errno.h>

#include "neuro/inc/eeg_manager.h"

LOG_MODULE_REGISTER(eeg_test, LOG_LEVEL_INF);

enum {
  EEG_TEST_STACK_SIZE = 3072,
  /* Between sd_fs_test (prio 3) and stim_test (prio 6). */
  EEG_TEST_PRIORITY = 5,
  EEG_TEST_LOG_PERIOD_MS = 5000,
};

static struct k_thread s_thread;
K_THREAD_STACK_DEFINE(eeg_test_stack, EEG_TEST_STACK_SIZE);

static void eeg_test_thread(void *arg1, void *arg2, void *arg3)
{
  ARG_UNUSED(arg1);
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  unsigned frames_ok = 0u;
  unsigned read_eagain = 0u;

  k_msleep(150);

  (void)eeg_manager_init();

  const eeg_manager_config_t cfg = {
    .sample_rate_hz = TASK_EEG_DEFAULT_SAMPLE_RATE_HZ,
  };

  int err = eeg_manager_start(cfg);
  if (err != 0) {
    LOG_ERR("eeg_test: eeg_manager_start failed (%d)", err);
    return;
  }

  LOG_INF("eeg_test: streaming @ %u Hz (read loop)", (unsigned int)cfg.sample_rate_hz);

  int64_t next_log_ms = k_uptime_get() + EEG_TEST_LOG_PERIOD_MS;

  for (;;) {
    eeg_manager_frame_t frame;
    err = eeg_manager_read_frame(&frame);
    if (err == 0) {
      frames_ok++;
    } else if (err == -EAGAIN) {
      read_eagain++;
      k_msleep(2);
      continue;
    } else {
      LOG_WRN("eeg_test: eeg_manager_read_frame err %d", err);
      k_msleep(10);
      continue;
    }

    const int64_t now = k_uptime_get();
    if (now >= next_log_ms) {
      LOG_INF("eeg_test: frames_ok=%u eagain_cycles=%u last_n=%u", frames_ok,
              read_eagain, (unsigned int)frame.sample_count);
      next_log_ms = now + EEG_TEST_LOG_PERIOD_MS;
    }

    /* Timer drives samples slower than tight loop — yield so SPI workers run. */
    k_yield();
  }
}

int eeg_test_start(void)
{
  k_thread_create(&s_thread, eeg_test_stack, K_THREAD_STACK_SIZEOF(eeg_test_stack), eeg_test_thread,
                  NULL,
                  NULL, NULL, EEG_TEST_PRIORITY, 0, K_NO_WAIT);
  k_thread_name_set(&s_thread, "eeg_test");
  return 0;
}
