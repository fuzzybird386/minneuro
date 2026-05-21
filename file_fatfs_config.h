#pragma once

/*
 * Project-local FatFs overrides.
 *
 * We use direct FatFs diskio callbacks implemented in driver/file.c instead of
 * Zephyr's zfs_diskio bridge, so keep a single numeric drive "0:".
 */

#undef FF_STR_VOLUME_ID
#define FF_STR_VOLUME_ID 0

#undef FF_VOLUMES
#define FF_VOLUMES 1

#undef FF_USE_MKFS
#define FF_USE_MKFS 1

#undef FF_CODE_PAGE
#define FF_CODE_PAGE 437

#undef FF_USE_LFN
#define FF_USE_LFN 1

#undef FF_MAX_LFN
#define FF_MAX_LFN 255

#undef FF_MIN_SS
#define FF_MIN_SS 512

#undef FF_MAX_SS
#define FF_MAX_SS 512

#undef FF_FS_TINY
#define FF_FS_TINY 1
