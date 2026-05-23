#pragma once

/**
 * Spawn a Zephyr thread that runs a small filesystem smoke test:
 * file_init → write/read verify → optional task-dir + segment append →
 * optional continuous-append loop every SD_FS_TEST_APPEND_LOOP_MS ms (tail readback timing).
 *
 * Paths use VFS scheme "/MinNeuroStorage:/..." per driver/file.h.
 */
int sd_fs_test_start(void);
