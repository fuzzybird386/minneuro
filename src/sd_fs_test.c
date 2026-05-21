#include "inc/sd_fs_test.h"

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "driver/file.h"

LOG_MODULE_REGISTER(sd_fs_test, LOG_LEVEL_INF);

enum {
  /* f_mount / mkfs need stack; prio 3 runs ahead of many stack threads (e.g. prio ~8). */
  SD_FS_TEST_STACK_SIZE = 4096,
  SD_FS_TEST_PRIORITY = 3,
};

#define TEST_APPEND_FILE "/MinNeuroStorage:/fs_test/sd_smoke.txt"

static const char s_payload[] = "sd_fs_test hello\nsecond line\n";
static struct k_thread s_thread;
K_THREAD_STACK_DEFINE(s_stack, SD_FS_TEST_STACK_SIZE);

static void sd_fs_test_thread(void *arg1, void *arg2, void *arg3)
{
  ARG_UNUSED(arg1);
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  k_msleep(200);

  LOG_INF("sd_fs_test: begin");

  LOG_INF("sd_fs_test: file_init entering (SPI mount)...");
  int rc = file_init();
  LOG_INF("sd_fs_test: file_init returned %d", rc);
  if (rc != 0) {
    LOG_ERR("sd_fs_test: file_init failed (%d)", rc);
    return;
  }

  LOG_INF("sd_fs_test: card_detect=%d (0=maybe no CD routed / open socket)",
          file_is_card_inserted());

  rc = file_create(TEST_APPEND_FILE);
  if (rc != 0) {
    LOG_ERR("sd_fs_test: file_create failed (%d)", rc);
    return;
  }

  rc = file_append(TEST_APPEND_FILE, (const uint8_t *)s_payload, sizeof(s_payload) - 1u);
  if (rc != 0) {
    LOG_ERR("sd_fs_test: file_append failed (%d)", rc);
    return;
  }

  rc = file_sync_all();
  if (rc != 0) {
    LOG_ERR("sd_fs_test: file_sync_all failed (%d)", rc);
    return;
  }

  char rb[sizeof(s_payload) + 16];
  size_t n = 0;
  rc = file_read_text(TEST_APPEND_FILE, rb, sizeof(rb), &n);
  if (rc != 0) {
    LOG_ERR("sd_fs_test: file_read_text failed (%d)", rc);
    return;
  }

  if (n != sizeof(s_payload) - 1u || memcmp(rb, s_payload, n) != 0) {
    LOG_ERR("sd_fs_test: readback mismatch (len=%zu expected=%zu)",
            n, sizeof(s_payload) - 1u);
    return;
  }

  LOG_INF("sd_fs_test: append + readback OK (%zu bytes)", n);

  char task_dir[96];
  char seg_path[128];
  LOG_INF("sd_fs_test: phase2 mkdir task dir...");
  rc = file_create_task_dir("sdsmoke", task_dir, sizeof(task_dir));
  if (rc != 0) {
    LOG_ERR("sd_fs_test: file_create_task_dir failed (%d)", rc);
    return;
  }
  LOG_INF("sd_fs_test: task_dir=%s", task_dir);

  LOG_INF("sd_fs_test: phase2 segment path...");
  rc = file_make_segment_path(task_dir, "seg", 0u, seg_path, sizeof(seg_path));
  if (rc != 0) {
    LOG_ERR("sd_fs_test: file_make_segment_path failed (%d)", rc);
    return;
  }

  LOG_INF("sd_fs_test: phase2 append segment...");
  static const uint8_t seg_blob[] = {0x01, 0x02, 0x03};
  rc = file_append(seg_path, seg_blob, sizeof(seg_blob));
  if (rc != 0) {
    LOG_ERR("sd_fs_test: segment file_append failed (%d)", rc);
    return;
  }

  LOG_INF("sd_fs_test: phase2 sync...");
  rc = file_sync_all();
  if (rc != 0) {
    LOG_ERR("sd_fs_test: second sync failed (%d)", rc);
    return;
  }

  LOG_INF("sd_fs_test: task dir OK: %s", seg_path);
  LOG_INF("sd_fs_test: all checks passed");
}

int sd_fs_test_start(void)
{
  k_thread_create(&s_thread, s_stack, K_THREAD_STACK_SIZEOF(s_stack), sd_fs_test_thread, NULL, NULL,
                  NULL, SD_FS_TEST_PRIORITY, 0, K_NO_WAIT);
  k_thread_name_set(&s_thread, "sd_fs_test");
  return 0;
}
