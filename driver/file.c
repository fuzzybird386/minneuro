/*
 * file.c — SD card SPI protocol + FatFs integration + high-level file API
 *
 * Data path every byte travels:
 *   file_append(path, buf, len)
 *     → f_write()           [FatFs]
 *       → disk_write()      [diskio layer, end of this file]
 *         → sd_write_block()
 *           → file_xfer()   [HAL: file_nrf54_hal.c → spim00_bus]
 *
 * SD protocol reference: Physical Layer Simplified Spec v9 §7 (SPI mode).
 *
 * Portability: to move to another MCU, only replace file_*_hal.c.
 * Nothing in this file touches nRF registers.
 */

#include "file.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* FatFs (CONFIG_FAT_FILESYSTEM_ELM). */
#include <ff.h>
#include <diskio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

/* ============================================================
 * Internal helpers
 * ============================================================ */

#define MOUNT_POINT   "/MinNeuroStorage:"
#define MOUNT_DRIVE   "0:"   /* FatFs drive number 0 */

static FATFS  s_fs;
static bool   s_mounted;
static struct k_mutex s_mutex;
static bool   s_mutex_inited;

/* ============================================================
 * SD SPI low-level protocol
 * ============================================================
 *
 * All transfers go through file_xfer(tx, rx, len) from the HAL.
 * CS is managed here; the HAL exposes file_cs_select/deselect.
 *
 * Card type flags.
 */
#define CT_MMC   0x01
#define CT_SD1   0x02
#define CT_SD2   0x04
#define CT_SDC   (CT_SD1 | CT_SD2)
#define CT_BLOCK 0x08

static uint8_t s_card_type;

/* --- byte-level helpers --- */

static void sd_send_byte(uint8_t b)
{
  file_xfer(&b, NULL, 1);
}

static uint8_t sd_recv_byte(void)
{
  uint8_t b = 0xFF;
  file_xfer(NULL, &b, 1);
  return b;
}

/* Flush MOSI high between transactions (≥8 clocks required by spec). */
static void sd_idle_clocks(uint32_t n)
{
  for (uint32_t i = 0; i < n; ++i) {
    sd_recv_byte();
  }
}

/* --- wait for card to deassert busy (R1 bit 0 = 0) --- */
static bool sd_wait_ready(uint32_t timeout_ms)
{
  uint32_t deadline = k_uptime_get_32() + timeout_ms;
  uint8_t b;
  do {
    b = sd_recv_byte();
  } while ((b != 0xFF) && (k_uptime_get_32() < deadline));
  return (b == 0xFF);
}

/* --- SD command frame ---
 *  [0] 0x40|cmd  [1..4] arg  [5] CRC|0x01
 * Returns R1 response byte; timeout returns 0xFF.
 */
static uint8_t sd_cmd(uint8_t cmd, uint32_t arg)
{
  /* Stuff byte required if busy from previous write, except CMD12 abort */
  if (cmd != 12U) {
    (void)sd_recv_byte();
  }

  uint8_t frame[6] = {
    (uint8_t)(0x40U | cmd),
    (uint8_t)(arg >> 24),
    (uint8_t)(arg >> 16),
    (uint8_t)(arg >> 8),
    (uint8_t)(arg),
    0x01U /* CRC placeholder */
  };
  /* Valid CRCs for the two commands that run before card switches to SPI mode */
  if (cmd == 0U)  { frame[5] = 0x95U; }
  if (cmd == 8U)  { frame[5] = 0x87U; }

  file_xfer(frame, NULL, 6);

  /* Wait for response (up to 10 bytes) */
  uint8_t r1 = 0xFF;
  for (int i = 0; i < 10; ++i) {
    r1 = sd_recv_byte();
    if ((r1 & 0x80U) == 0U) {
      break;
    }
  }
  return r1;
}

/* ACMD = CMD55 prefix + CMDn */
static uint8_t sd_acmd(uint8_t acmd, uint32_t arg)
{
  sd_cmd(55, 0);
  return sd_cmd(acmd, arg);
}

/* --- wait for start-block token (0xFE) --- */
static bool sd_wait_token(uint8_t token, uint32_t timeout_ms)
{
  uint32_t deadline = k_uptime_get_32() + timeout_ms;
  uint8_t b;
  do {
    b = sd_recv_byte();
    if (b == token) {
      return true;
    }
  } while (k_uptime_get_32() < deadline);
  return false;
}

/* ============================================================
 * SD card initialisation (SPI mode power-on sequence)
 * ============================================================ */

static int sd_init(void)
{
  uint8_t r1;

  s_card_type = 0;

  /* >=74 dummy clocks with CS high before first command */
  file_cs_deselect();
  sd_idle_clocks(10); /* 80 clocks */

  /* CMD0: software reset → idle state (R1=0x01) */
  file_cs_select();
  r1 = sd_cmd(0, 0);
  file_cs_deselect();
  if (r1 != 0x01U) {
    printk("sd_init: CMD0 idle failed r1=0x%02x (want 0x01) — check CS/MISO/MOSI/SCK, "
	   "power, and SPI mode\n",
	   r1);
    return -EIO;
  }

  /* CMD8: check voltage range (Vcc 2.7–3.6 V, check pattern 0xAA) */
  file_cs_select();
  r1 = sd_cmd(8, 0x000001AAU);
  if (r1 == 0x01U) {
    /* SD v2: read 32-bit R7 (4 bytes after R1) */
    uint8_t r7[4];
    for (int i = 0; i < 4; ++i) {
      r7[i] = sd_recv_byte();
    }
    file_cs_deselect();

    if ((r7[2] == 0x01U) && (r7[3] == 0xAAU)) {
      /* ACMD41 with HCS=1 to activate (≤1000 ms) */
      uint32_t deadline = k_uptime_get_32() + 1000U;
      do {
        file_cs_select();
        r1 = sd_acmd(41, 0x40000000UL);
        file_cs_deselect();
      } while ((r1 != 0x00U) && (k_uptime_get_32() < deadline));

      if (r1 != 0x00U) {
        printk("sd_init: ACMD41 (SDv2) timeout last r1=0x%02x (want 0x00)\n", r1);
        return -ETIMEDOUT;
      }

      /* CMD58: read OCR to check CCS (block-addressed?) */
      file_cs_select();
      r1 = sd_cmd(58, 0);
      uint8_t ocr[4];
      for (int i = 0; i < 4; ++i) {
        ocr[i] = sd_recv_byte();
      }
      file_cs_deselect();
      s_card_type = (ocr[0] & 0x40U) ? (CT_SD2 | CT_BLOCK) : CT_SD2;
    } else {
      printk("sd_init: CMD8 R7 mismatch r7=%02x %02x %02x %02x (want .. 01 AA)\n", r7[0],
	     r7[1], r7[2], r7[3]);
      return -EIO;
    }
  } else {
    printk("sd_init: CMD8 r1=0x%02x → SDv1/MMC path (expected 0x01 for SDv2)\n", r1);
    /* SD v1 or MMC */
    file_cs_deselect();
    uint32_t deadline = k_uptime_get_32() + 1000U;
    uint8_t cmd_init;
    /* Try ACMD41 first (SD v1), then CMD1 (MMC) */
    file_cs_select();
    if (sd_acmd(41, 0) <= 0x01U) {
      s_card_type = CT_SD1;
      cmd_init = 41;
    } else {
      s_card_type = CT_MMC;
      cmd_init = 1;
    }
    file_cs_deselect();

    do {
      file_cs_select();
      if (cmd_init == 41U) {
        r1 = sd_acmd(41, 0);
      } else {
        r1 = sd_cmd(1, 0);
      }
      file_cs_deselect();
    } while ((r1 != 0x00U) && (k_uptime_get_32() < deadline));

    if (r1 != 0x00U) {
      printk(
	  "sd_init: SDv1/MMC init timeout last r1=0x%02x type=%u (41=SDv1, 1=MMC path)\n",
	  r1, (unsigned)s_card_type);
      small_card_type_cleanup:
      s_card_type = 0;
      return -ETIMEDOUT;
    }

    /* CMD16: set block length to 512 for byte-addressed cards */
    file_cs_select();
    r1 = sd_cmd(16, 512);
    file_cs_deselect();
    if (r1 != 0x00U) {
      printk("sd_init: CMD16 (block len 512) failed r1=0x%02x\n", r1);
      goto small_card_type_cleanup;
    }
  }

  /* All good: switch to high-speed SPI */
  file_spi_set_fast();
  return 0;
}

/* ============================================================
 * SD sector read / write (512 bytes per sector)
 * ============================================================ */

static int sd_read_sector(uint32_t sector, uint8_t *buf)
{
  /* Byte-addressed cards need byte offset */
  uint32_t addr = (s_card_type & CT_BLOCK) ? sector : (sector * 512U);

  file_cs_select();
  uint8_t r1 = sd_cmd(17, addr);        /* CMD17: READ_SINGLE_BLOCK */
  if (r1 != 0x00U) {
    file_cs_deselect();
    return -EIO;
  }

  bool ok = sd_wait_token(0xFEU, 200);  /* start token */
  if (!ok) {
    file_cs_deselect();
    return -ETIMEDOUT;
  }

  file_xfer(NULL, buf, 512);            /* data payload */
  sd_recv_byte();                        /* CRC high */
  sd_recv_byte();                        /* CRC low  */
  file_cs_deselect();
  sd_idle_clocks(1);
  return 0;
}

static int sd_write_sector(uint32_t sector, const uint8_t *buf)
{
  uint32_t addr = (s_card_type & CT_BLOCK) ? sector : (sector * 512U);

  file_cs_select();
  if (!sd_wait_ready(500U)) {
    file_cs_deselect();
    return -EBUSY;
  }

  uint8_t r1 = sd_cmd(24, addr);        /* CMD24: WRITE_BLOCK */
  if (r1 != 0x00U) {
    file_cs_deselect();
    return -EIO;
  }

  /* Start block token */
  uint8_t token = 0xFEU;
  file_xfer(&token, NULL, 1);

  /* 512-byte payload */
  file_xfer(buf, NULL, 512);

  /* Dummy CRC */
  sd_recv_byte();
  sd_recv_byte();

  /* Data response */
  uint8_t resp = sd_recv_byte();
  file_cs_deselect();
  sd_idle_clocks(1);

  if ((resp & 0x1FU) != 0x05U) {
    return -EIO;
  }

  /* Wait for card to complete internal write */
  file_cs_select();
  bool ready = sd_wait_ready(500U);
  file_cs_deselect();
  return ready ? 0 : -ETIMEDOUT;
}

/* ============================================================
 * FatFs diskio callbacks — Zephyr's ELM FatFs calls these
 * ============================================================
 * FatFs is compiled with CONFIG_FAT_FILESYSTEM_ELM.
 * We register drive "0" and implement the required hooks.
 */

DSTATUS disk_initialize(BYTE drv)
{
  if (drv != 0U) {
    return STA_NOINIT;
  }

  /* Cold power / SPI bus contention: first CMD0 sometimes fails; retry after CS high idle. */
  enum {
    SD_DISK_INIT_TRIES = 6,
    SD_DISK_INIT_RETRY_DELAY_MS = 80,
  };

  for (unsigned try_n = 0U; try_n < SD_DISK_INIT_TRIES; try_n++) {
    if (try_n > 0U) {
      file_cs_deselect();
      k_msleep(SD_DISK_INIT_RETRY_DELAY_MS);
    }
    file_spi_set_slow();
    if (sd_init() == 0) {
      return 0;
    }
  }
  return STA_NOINIT;
}

DSTATUS disk_status(BYTE drv)
{
  if (drv != 0U) {
    return STA_NOINIT;
  }
  return (s_card_type == 0U) ? STA_NOINIT : 0;
}

DRESULT disk_read(BYTE drv, BYTE *buf, LBA_t sector, UINT count)
{
  if (drv != 0U || s_card_type == 0U) {
    return RES_NOTRDY;
  }

  for (UINT i = 0; i < count; ++i) {
    if (sd_read_sector(sector + i, buf + (i * 512U)) != 0) {
      return RES_ERROR;
    }
  }
  return RES_OK;
}

DRESULT disk_write(BYTE drv, const BYTE *buf, LBA_t sector, UINT count)
{
  if (drv != 0U || s_card_type == 0U) {
    return RES_NOTRDY;
  }

  for (UINT i = 0; i < count; ++i) {
    if (sd_write_sector(sector + i, buf + (i * 512U)) != 0) {
      return RES_ERROR;
    }
  }
  return RES_OK;
}

DRESULT disk_ioctl(BYTE drv, BYTE cmd, void *buf)
{
  if (drv != 0U || s_card_type == 0U) {
    return RES_NOTRDY;
  }

  switch (cmd) {
  case CTRL_SYNC:
    /* Nothing buffered at this layer; SD sector writes are synchronous. */
    return RES_OK;

  case GET_SECTOR_SIZE:
    *(WORD *)buf = 512;
    return RES_OK;

  case GET_BLOCK_SIZE:
    *(DWORD *)buf = 1U; /* erase block = 1 sector (safe default) */
    return RES_OK;

  default:
    return RES_PARERR;
  }
}

/* ============================================================
 * Path / directory helpers (internal)
 * ============================================================ */

/* Recursively mkdir every component of an absolute FatFs path. */
static int mkdirs_recursive(const char *path)
{
  char tmp[160];
  size_t len = strlen(path);

  if (len == 0U || len >= sizeof(tmp)) {
    return -EINVAL;
  }

  memcpy(tmp, path, len + 1U);

  /* Skip drive prefix "0:" (2 chars). First '/' yields tmp=="0:" — volume root exists;
   * calling f_mkdir("0:") is invalid on many FatFs builds and can misbehave or fail. */
  for (size_t i = 2U; i < len; ++i) {
    if (tmp[i] == '/') {
      tmp[i] = '\0';
      bool volume_only =
          (tmp[0] == (BYTE)'0' && tmp[1] == (BYTE)':' && tmp[2] == (BYTE)'\0');
      if (!volume_only) {
        FRESULT fr = f_mkdir(tmp);
        if (fr != FR_OK && fr != FR_EXIST) {
          return -EIO;
        }
      }
      tmp[i] = '/';
    }
  }
  FRESULT fr = f_mkdir(tmp);
  return (fr == FR_OK || fr == FR_EXIST) ? 0 : -EIO;
}

/* Build FatFs path from API path ("/SD:/foo") → "0:/foo". */
static int api_to_fat(const char *api_path, char *fat, size_t fat_len)
{
  if (strncmp(api_path, MOUNT_POINT, strlen(MOUNT_POINT)) != 0) {
    return -EINVAL;
  }
  const char *rest = api_path + strlen(MOUNT_POINT);
  int n = snprintf(fat, fat_len, MOUNT_DRIVE "%s", rest);
  if (n <= 0 || (size_t)n >= fat_len) {
    return -ENAMETOOLONG;
  }
  return 0;
}

/* Create every directory in fat_path. */
static int ensure_parent_dirs(const char *fat_path)
{
  char dir[160];
  size_t len = strlen(fat_path);
  if (len >= sizeof(dir)) {
    return -ENAMETOOLONG;
  }

  memcpy(dir, fat_path, len + 1U);
  char *slash = strrchr(dir, '/');
  if (slash == NULL || slash <= dir + 2U) {
    /* No parent dir beyond drive root */
    return 0;
  }

  *slash = '\0';
  return mkdirs_recursive(dir);
}

/* ============================================================
 * Open-file cache: keeps up to 4 FIL handles open for fast append
 * ============================================================ */

#define CACHE_SIZE 4

struct cache_entry {
  bool     used;
  bool     dirty;
  uint32_t last_used;
  char     fat_path[160];
  FIL      fil;
};

static struct cache_entry s_cache[CACHE_SIZE];
static uint32_t s_cache_tick;
static uint32_t s_task_id;

static struct cache_entry *cache_get_or_open(const char *fat_path)
{
  struct cache_entry *free_slot  = NULL;
  struct cache_entry *evict_slot = NULL;
  uint32_t oldest = UINT32_MAX;

  for (int i = 0; i < CACHE_SIZE; ++i) {
    struct cache_entry *e = &s_cache[i];
    if (e->used && strcmp(e->fat_path, fat_path) == 0) {
      e->last_used = ++s_cache_tick;
      return e;
    }
    if (!e->used && free_slot == NULL) {
      free_slot = e;
    } else if (e->used && e->last_used < oldest) {
      oldest     = e->last_used;
      evict_slot = e;
    }
  }

  struct cache_entry *slot = free_slot;
  if (slot == NULL) {
    /* Evict LRU entry, flushing first */
    slot = evict_slot;
    if (slot == NULL) {
      return NULL;
    }
    if (slot->dirty) {
      f_sync(&slot->fil);
    }
    f_close(&slot->fil);
    slot->used  = false;
    slot->dirty = false;
  }

  /* Open at end for append */
  FRESULT fr = f_open(&slot->fil, fat_path, FA_OPEN_ALWAYS | FA_WRITE);
  if (fr != FR_OK) {
    return NULL;
  }
  f_lseek(&slot->fil, f_size(&slot->fil));

  slot->used      = true;
  slot->dirty     = false;
  slot->last_used = ++s_cache_tick;
  (void)snprintf(slot->fat_path, sizeof(slot->fat_path), "%s", fat_path);
  return slot;
}

/* ============================================================
 * Public API
 * ============================================================ */

int file_init(void)
{
  /* Re-calling k_mutex_init() on the same object is undefined; guard once. */
  if (!s_mutex_inited) {
    k_mutex_init(&s_mutex);
    s_mutex_inited = true;
  }

  memset(s_cache, 0, sizeof(s_cache));

  int rc = file_dev_init();
  if (rc != 0) {
    return rc;
  }

  k_mutex_lock(&s_mutex, K_FOREVER);

  if (!s_mounted) {
    file_spi_set_slow();

    /* Mount; if no filesystem, format FAT32 then remount */
    FRESULT fr = f_mount(&s_fs, MOUNT_DRIVE, 1);
    if (fr == FR_NO_FILESYSTEM) {
      BYTE work[512];
      fr = f_mkfs(MOUNT_DRIVE, NULL, work, sizeof(work));
      if (fr == FR_OK) {
        fr = f_mount(&s_fs, MOUNT_DRIVE, 1);
      }
    }
    if (fr == FR_OK) {
      s_mounted = true;
    } else {
      printk("file_init: FAT f_mount/mkfs failed FRESULT=%u "
	     "(FR_DISK_ERR=1 FR_NOT_READY=3 FR_NO_FILESYSTEM=13 FR_MKFS_ABORTED=14 "
	     "FR_TIMEOUT=15)\n",
	     (unsigned)fr);
      rc = -EIO;
    }
  }

  k_mutex_unlock(&s_mutex);
  return rc;
}

int file_create(const char *path)
{
  char fat[160];
  int rc = api_to_fat(path, fat, sizeof(fat));
  if (rc != 0) {
    return rc;
  }

  k_mutex_lock(&s_mutex, K_FOREVER);
  rc = ensure_parent_dirs(fat);
  if (rc == 0) {
    FIL f;
    FRESULT fr = f_open(&f, fat, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr == FR_OK) {
      f_close(&f);
    } else {
      rc = -EIO;
    }
  }
  k_mutex_unlock(&s_mutex);
  return rc;
}

int file_append(const char *path, const uint8_t *data, size_t len)
{
  if (path == NULL || data == NULL || len == 0U) {
    return -EINVAL;
  }

  char fat[160];
  int rc = api_to_fat(path, fat, sizeof(fat));
  if (rc != 0) {
    return rc;
  }

  k_mutex_lock(&s_mutex, K_FOREVER);

  rc = ensure_parent_dirs(fat);
  if (rc != 0) {
    k_mutex_unlock(&s_mutex);
    return rc;
  }

  struct cache_entry *e = cache_get_or_open(fat);
  if (e == NULL) {
    k_mutex_unlock(&s_mutex);
    return -EMFILE;
  }

  UINT written = 0;
  FRESULT fr   = f_write(&e->fil, data, (UINT)len, &written);
  if (fr != FR_OK || written != (UINT)len) {
    k_mutex_unlock(&s_mutex);
    return -EIO;
  }

  e->dirty = true;
  k_mutex_unlock(&s_mutex);
  return 0;
}

int file_sync_all(void)
{
  k_mutex_lock(&s_mutex, K_FOREVER);

  for (int i = 0; i < CACHE_SIZE; ++i) {
    if (s_cache[i].used && s_cache[i].dirty) {
      f_sync(&s_cache[i].fil);
      s_cache[i].dirty = false;
    }
  }

  k_mutex_unlock(&s_mutex);
  return 0;
}

int file_read_text(const char *path, char *buf, size_t buf_len, size_t *out_len)
{
  if (path == NULL || buf == NULL || buf_len == 0U) {
    return -EINVAL;
  }

  char fat[160];
  int rc = api_to_fat(path, fat, sizeof(fat));
  if (rc != 0) {
    return rc;
  }

  k_mutex_lock(&s_mutex, K_FOREVER);

  FIL f;
  FRESULT fr = f_open(&f, fat, FA_READ);
  if (fr != FR_OK) {
    k_mutex_unlock(&s_mutex);
    return -ENOENT;
  }

  UINT bytes_read = 0;
  fr = f_read(&f, buf, (UINT)(buf_len - 1U), &bytes_read);
  f_close(&f);

  k_mutex_unlock(&s_mutex);

  if (fr != FR_OK) {
    return -EIO;
  }

  buf[bytes_read] = '\0';
  if (out_len != NULL) {
    *out_len = (size_t)bytes_read;
  }
  return 0;
}

int file_read_tail_text(const char *path, char *buf, size_t buf_len, size_t tail_max,
                       size_t *out_len)
{
  if (path == NULL || buf == NULL || buf_len == 0U || tail_max == 0U) {
    return -EINVAL;
  }

  char fat[160];
  int rc = api_to_fat(path, fat, sizeof(fat));
  if (rc != 0) {
    return rc;
  }

  k_mutex_lock(&s_mutex, K_FOREVER);

  FIL f;
  FRESULT fr = f_open(&f, fat, FA_READ);
  if (fr != FR_OK) {
    k_mutex_unlock(&s_mutex);
    return -ENOENT;
  }

  FSIZE_t sz = f_size(&f);
  FSIZE_t capacity = (FSIZE_t)(buf_len - 1U);
  FSIZE_t want = (FSIZE_t)tail_max;
  if (want > sz) {
    want = sz;
  }
  if (want > capacity) {
    want = capacity;
  }

  UINT bytes_read = 0;

  if (want == (FSIZE_t)0U) {
    fr = FR_OK;
  } else {
    FSIZE_t pos = sz - want;
    fr = f_lseek(&f, pos);
    if (fr == FR_OK) {
      fr = f_read(&f, buf, (UINT)want, &bytes_read);
    }
  }

  f_close(&f);

  k_mutex_unlock(&s_mutex);

  if (fr != FR_OK) {
    return -EIO;
  }

  buf[bytes_read] = '\0';
  if (out_len != NULL) {
    *out_len = (size_t)bytes_read;
  }
  return 0;
}

int file_create_task_dir(const char *task_name, char *out_dir, size_t out_dir_len)
{
  if (task_name == NULL || out_dir == NULL || out_dir_len == 0U) {
    return -EINVAL;
  }

  int n = snprintf(out_dir, out_dir_len, MOUNT_POINT "/tasks/%s_%06u",
                   task_name, (unsigned int)(++s_task_id));
  if (n <= 0 || (size_t)n >= out_dir_len) {
    return -ENAMETOOLONG;
  }

  char fat[160];
  int rc = api_to_fat(out_dir, fat, sizeof(fat));
  if (rc != 0) {
    return rc;
  }

  k_mutex_lock(&s_mutex, K_FOREVER);
  rc = mkdirs_recursive(fat);
  k_mutex_unlock(&s_mutex);
  return rc;
}

int file_make_segment_path(const char *task_dir, const char *prefix,
                           uint32_t index, char *out_path, size_t out_path_len)
{
  if (task_dir == NULL || prefix == NULL || out_path == NULL || out_path_len == 0U) {
    return -EINVAL;
  }

  int n = snprintf(out_path, out_path_len, "%s/%s_%06u.bin",
                   task_dir, prefix, (unsigned int)index);
  if (n <= 0 || (size_t)n >= out_path_len) {
    return -ENAMETOOLONG;
  }
  return 0;
}
