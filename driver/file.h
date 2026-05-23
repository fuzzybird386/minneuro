#pragma once

#include <stdint.h>
#include <stddef.h>

/*
 * High-level file API
 * -------------------
 * All paths are absolute from mount point, e.g. "/MinNeuroStorage:/tasks/eeg_000001/seg_000001.bin"
 *
 * Data path (every byte goes through your SPI):
 *   file_append / file_read_text / file_read_tail_text
 *       → FatFs (f_write / f_read / seek + tail read)
 *       → sd_read_sector / sd_write_sector (SD SPI protocol in file.c)
 *       → file_xfer (HAL in file_nrf54_hal.c)
 *       → spim00_bus / nrfx_spim (single mutex with dac80501_hal + ks1092)
 */

int file_init(void);
int file_is_card_inserted(void);

int file_create(const char *path);
int file_append(const char *path, const uint8_t *data, size_t len);
int file_sync_all(void);
int file_read_text(const char *path, char *buf, size_t buf_len, size_t *out_len);

/* Read up to min(tail_max, file_size, buf_len-1) bytes from EOF into buf (NUL-terminated). */
int file_read_tail_text(const char *path, char *buf, size_t buf_len, size_t tail_max,
                       size_t *out_len);

int file_create_task_dir(const char *task_name, char *out_dir, size_t out_dir_len);
int file_make_segment_path(const char *task_dir, const char *prefix, uint32_t index,
                           char *out_path, size_t out_path_len);

/*
 * HAL interface — implemented per platform (e.g. file_nrf54_hal.c)
 * ----------------------------------------------------------------
 * To port to another MCU, only re-implement these 6 functions.
 */

int  file_dev_init(void);
void file_spi_set_slow(void);    /* ≤400 kHz,  SD card init phase */
void file_spi_set_fast(void);    /* up to 25 MHz, SD card data phase */
void file_cs_select(void);       /* drive CS low */
void file_cs_deselect(void);     /* drive CS high */
int  file_xfer(const uint8_t *tx, uint8_t *rx, uint32_t len);
