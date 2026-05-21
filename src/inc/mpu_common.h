#pragma once
#include <stdint.h>

#define EEG_CHANNEL_SAFE_SIZE 16  // 暂定 16 通道为最大通道数，后续如果需要扩增则修改，此处调小可节省 RAM 空间
#define EEG_BUFFER_SAFE_SIZE 14   // 协议规定 10ms 一帧，此处最大支持 300 * 100Hz = 30kHz 采样率
#define IMU_BUFFER_SAFE_SIZE 14   // 协议规定 10ms 一帧，此处最大支持 10 * 100Hz = 1kHz 采样率

typedef struct {
  uint16_t temperature;
  uint16_t ax, ay, az, gx, gy, gz, mx, my, mz;
  uint16_t yaw, pitch, roll;
} mpu_imu_data_t;

typedef struct {
  uint16_t volt[EEG_CHANNEL_SAFE_SIZE]; // max: 256
} mpu_eeg_data_t;

typedef struct {
  uint16_t seq; // 可选：序列号，便于上层检测丢包
  uint8_t  imu_length;
  uint8_t  imu_channels; // channel size.
  uint16_t eeg_length;
  uint16_t eeg_channels; // channel size.
  mpu_eeg_data_t eegs[EEG_BUFFER_SAFE_SIZE];
  mpu_imu_data_t imus[IMU_BUFFER_SAFE_SIZE];
} mpu_sync_frame_t;
