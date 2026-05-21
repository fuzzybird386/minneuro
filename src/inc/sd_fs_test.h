#pragma once

/**
 * Spawn a Zephyr thread that runs a small filesystem smoke test:
 * file_init → write/read verify → optional task-dir + segment append.
 *
 * Paths use VFS scheme "/MinNeuroStorage:/..." per driver/file.h.
 */
int sd_fs_test_start(void);
