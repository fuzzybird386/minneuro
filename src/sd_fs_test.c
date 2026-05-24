#include "inc/sd_fs_test.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include "driver/file.h"

LOG_MODULE_REGISTER(sd_fs_test, LOG_LEVEL_INF);

/*
 * Continuous SD write benchmark (after smoke checks). Tune chunk/rounds in build flags:
 *   -DSD_FS_TEST_PERF_ENABLE=0   disable
 *   -DSD_FS_PERF_CHUNK_BYTES=1024 -DSD_FS_PERF_ROUNDS=500
 */
#ifndef SD_FS_TEST_PERF_ENABLE
#define SD_FS_TEST_PERF_ENABLE 1
#endif
#ifndef SD_FS_PERF_CHUNK_BYTES
#define SD_FS_PERF_CHUNK_BYTES 512u
#endif
#ifndef SD_FS_PERF_ROUNDS
#define SD_FS_PERF_ROUNDS 200u
#endif
#ifndef SD_FS_TEST_APPEND_LOOP_MS
#define SD_FS_TEST_APPEND_LOOP_MS 3000
#endif

enum {
  /* f_mount / mkfs need stack; prio 3 runs ahead of many stack threads (e.g. prio ~8). */
  SD_FS_TEST_STACK_SIZE = 4096,
  SD_FS_TEST_PRIORITY = 3,
};

#define TEST_APPEND_FILE "/MinNeuroStorage:/fs_test/sd_fs_test.txt"

#if SD_FS_TEST_PERF_ENABLE
#define PERF_BULK_PATH "/MinNeuroStorage:/fs_test/sd_perf_bulk.bin"
static uint8_t s_perf_chunk[SD_FS_PERF_CHUNK_BYTES];
#endif
static const char s_payload[] = " hello Gong He ,I am here bro :)!\nsecond line\n";
static struct k_thread s_thread;
K_THREAD_STACK_DEFINE(sd_fs_test_stack, SD_FS_TEST_STACK_SIZE);

static void sd_fs_test_thread(void *arg1, void *arg2, void *arg3)
{
  //防编译警告
  ARG_UNUSED(arg1);
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);

  printk("sd loop thread spawned\n");
  k_msleep(235);

  LOG_INF("begin");

  LOG_INF("file_init (SPI mount)...");
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
  LOG_INF("sd_fs_test: readback content: %s", rb);

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

#if SD_FS_TEST_PERF_ENABLE
  LOG_INF("sd_fs_test: perf: bulk append (%u B x %u rounds) → %s", (unsigned int)sizeof(s_perf_chunk),
          SD_FS_PERF_ROUNDS, PERF_BULK_PATH);
  memset(s_perf_chunk, 0xA5u, sizeof(s_perf_chunk));

  rc = file_create(PERF_BULK_PATH);
  if (rc != 0) {
    LOG_ERR("sd_fs_test: perf file_create failed (%d) — delete %s if it exists stale", rc,
            PERF_BULK_PATH);
    return;
  }

  int64_t t_append0 = k_uptime_get();
  for (uint32_t i = 0; i < SD_FS_PERF_ROUNDS; i++) {
    rc = file_append(PERF_BULK_PATH, s_perf_chunk, sizeof(s_perf_chunk));
    if (rc != 0) {
      LOG_ERR("sd_fs_test: perf append round %u failed (%d)", (unsigned int)i, rc);
      return;
    }
  }
  int64_t t_append1 = k_uptime_get();
  int64_t dt_ms = t_append1 - t_append0;
  uint64_t total_bytes = (uint64_t)SD_FS_PERF_ROUNDS * (uint64_t)sizeof(s_perf_chunk);
  if (dt_ms > 0) {
    const double secs = (double)dt_ms / 1000.0;
    const double kib_s = (double)total_bytes / 1024.0 / secs;
    const double mbit_s = (double)total_bytes * 8.0 / secs / 1e6;
    LOG_INF("sd_fs_test: perf append_done total_bytes=%llu dt_ms=%lld (~%.2f KiB/s, ~%.3f Mbit/s)",
            (unsigned long long)total_bytes, (long long)dt_ms, kib_s, mbit_s);
  } else {
    LOG_INF("sd_fs_test: perf append_done total_bytes=%llu dt_ms=<1", (unsigned long long)total_bytes);
  }

  int64_t t_sync0 = k_uptime_get();
  rc = file_sync_all();
  int64_t t_sync1 = k_uptime_get();
  if (rc != 0) {
    LOG_ERR("sd_fs_test: perf file_sync_all failed (%d)", rc);
    return;
  }
  LOG_INF("sd_fs_test: perf sync_all dt_ms=%lld", (long long)(t_sync1 - t_sync0));
#endif /* SD_FS_TEST_PERF_ENABLE */

  LOG_INF("sd_fs_test: smoke complete; infinite append loop (interval=%d ms, tail readback)...",
          SD_FS_TEST_APPEND_LOOP_MS);

  static char s_loop_tail_rb[sizeof(s_payload) + 24];
  const size_t payload_u8_len = sizeof(s_payload) - 1u;

  for (unsigned iter = 1;; iter++) {
    int64_t t0 = k_uptime_get();

    rc = file_append(TEST_APPEND_FILE, (const uint8_t *)s_payload, payload_u8_len);
    int64_t t_after_append = k_uptime_get();

    if (rc != 0) {
      LOG_ERR("sd_fs_test: loop file_append iter=%u failed (%d)", iter, rc);
      k_msleep(SD_FS_TEST_APPEND_LOOP_MS);
      continue;
    }

    rc = file_sync_all();
    int64_t t_after_sync = k_uptime_get();

    if (rc != 0) {
      LOG_ERR("sd_fs_test: loop file_sync_all iter=%u failed (%d)", iter, rc);
      k_msleep(SD_FS_TEST_APPEND_LOOP_MS);
      continue;
    }

    LOG_INF(
        "sd_fs_test: loop iter=%u store append_ms=%lld sync_ms=%lld total_ms=%lld (bytes=%zu)",
        iter, (long long)(t_after_append - t0),
        (long long)(t_after_sync - t_after_append),
        (long long)(t_after_sync - t0),
        payload_u8_len);

    size_t tail_n = 0;
    int64_t tr0 = k_uptime_get();
    rc = file_read_tail_text(TEST_APPEND_FILE, s_loop_tail_rb, sizeof(s_loop_tail_rb),
                             payload_u8_len, &tail_n);
    int64_t tr1 = k_uptime_get();
    if (rc != 0) {
      LOG_ERR("sd_fs_test: loop file_read_tail_text iter=%u failed (%d)", iter, rc);
    } else {
      if (tail_n != payload_u8_len ||
          memcmp(s_loop_tail_rb, s_payload, payload_u8_len) != 0) {
        LOG_WRN("sd_fs_test: loop tail mismatch iter=%u got_len=%zu (expected %zu)", iter,
                tail_n, payload_u8_len);
      }
      LOG_INF("sd_fs_test: loop tail read_tail_ms=%lld last %zu B: \"%.*s\"",
              (long long)(tr1 - tr0), tail_n, (int)tail_n,
              s_loop_tail_rb);
    }

    k_msleep(SD_FS_TEST_APPEND_LOOP_MS);
  }
}

int sd_fs_test_start(void)
{
  k_thread_create(&s_thread, sd_fs_test_stack, K_THREAD_STACK_SIZEOF(sd_fs_test_stack), sd_fs_test_thread,
                  NULL, NULL,
                  NULL, SD_FS_TEST_PRIORITY, 0, K_NO_WAIT);
  k_thread_name_set(&s_thread, "sd_fs_test");
  return 0;
}
