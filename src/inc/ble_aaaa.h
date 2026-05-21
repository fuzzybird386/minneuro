#ifndef BLE_AAAA_H__
#define BLE_AAAA_H__

#include <stdbool.h>
#include <stdint.h>
#include "mpu_common.h"

int ble_aaaa_init();

int ble_aaff_notify_commit(uint8_t * data, uint8_t len);

int ble_aaef_notify_commit(uint8_t * data, uint8_t len);

int ble_aaff_notify_commit_mpu(uint32_t seq, mpu_eeg_data_t * data, uint8_t len, uint16_t channel_size);

bool ble_aaff_can_accept_frame(void);

int ble_aaef_notify_commit_mpu(uint32_t seq, mpu_imu_data_t * data, uint8_t len, uint8_t ch);

#endif
