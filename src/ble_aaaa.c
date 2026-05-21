#ifndef ENABLE_FACA_SERVICE__
#define ENABLE_FACA_SERVICE__

#include "inc/ble_aaaa_priv.h"

#include "inc/bsp.h"
#include "neuro/inc/eeg_manager.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(aaaa, LOG_LEVEL_INF);


/*
 **************************************************
 *                                                *
 *  Bluetooth LE.                                 *
 *                                                *
 **************************************************
 */

// SERVICE: AAAA
struct bt_uuid_128 aaaa_service_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0000AAAA, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb));

// EMG/ECG/EOG/EEG Characteristics
//// AAF0: READ STATE
struct bt_uuid_128 aaf0_char_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0000AAF0, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb));
//// AAFF: STREAM
struct bt_uuid_128 aaff_char_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0000AAFF, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb));

// IMU/MAG Characteristics
//// AAE0: READ STATE
struct bt_uuid_128 aae0_char_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0000AAE0, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb));
//// AAEF: STREAM
struct bt_uuid_128 aaef_char_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0000AAEF, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb));

// PPG Characteristics
//// AAD0: READ STATE
struct bt_uuid_128 aad0_char_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0000AAD0, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb));
//// AADF: STREAM
struct bt_uuid_128 aadf_char_uuid = BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x0000AADF, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb));


BT_GATT_SERVICE_DEFINE(aaaa_service,
  BT_GATT_PRIMARY_SERVICE(&aaaa_service_uuid),
  /** EMG/ECG/EOG/EEG **/
  // State.
  BT_GATT_CHARACTERISTIC(&aaf0_char_uuid.uuid, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP | BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
            BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, ble_on_aaf0_read_request, ble_on_aaf0_write_request, NULL),
  BT_GATT_CCC(ble_ccc_cfg_changed_aaf0, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
  // Stream.
  BT_GATT_CHARACTERISTIC(&aaff_char_uuid.uuid, BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
            BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, NULL, NULL, NULL),
  BT_GATT_CCC(ble_ccc_cfg_changed_aaff, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

  /** IMU/MAG **/
  // State.
  BT_GATT_CHARACTERISTIC(&aae0_char_uuid.uuid, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP | BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
            BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, ble_on_aae0_read_request, ble_on_aae0_write_request, NULL),
  BT_GATT_CCC(ble_ccc_cfg_changed_aae0, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
  // Stream.
  BT_GATT_CHARACTERISTIC(&aaef_char_uuid.uuid, BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_READ,
            BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, NULL, NULL, NULL),
  BT_GATT_CCC(ble_ccc_cfg_changed_aaef, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);


/*
 **************************************************
 *                                                *
 *  Worker Loop                                 *
 *                                                *
 **************************************************
*/

static mpu_sync_frame_t sync_frame[2];
volatile int sync_frame_index = 0;
volatile bool sync_frame_ready = false;
volatile uint32_t sync_seq = 0;
static uint32_t eeg_seq = 0;

int mpu_irq_on_rx_ready(mpu_sync_frame_t * frame)
{
  /* struct 赋值 = memcpy，完整值拷贝到全局静态数组，安全。
   * frame 指向 mpu_irq.c 内的 static parsed_frame，不会随函数返回而消失。*/
  sync_frame_index = (sync_frame_index + 1) % 2;
  sync_frame[sync_frame_index] = *frame;
  sync_frame_ready = true;
  sync_seq++;
  return 0;
}

// Looper

#define LOOPER_INTERVAL         1     // 1ms

static void looper_work_handler(struct k_work *work);

static K_WORK_DELAYABLE_DEFINE(looper_work, looper_work_handler);

static void looper_work_handler(struct k_work *work)
{
  eeg_manager_frame_t eeg_frame;
  static mpu_eeg_data_t eeg_notify_frame[TASK_EEG_FRAME_SIZE];
  uint8_t eeg_samples_to_send;

  // DREADY.
  // if (sync_frame_ready) {
  //   sync_frame_ready = false;
  //   const mpu_sync_frame_t *frame = &sync_frame[sync_frame_index]; // 用指针，避免栈上拷贝 ~588 字节
  //   ble_aaff_notify_commit_mpu(sync_seq, frame->eegs, frame->eeg_length, frame->eeg_channels);
  //   ble_aaef_notify_commit_mpu(sync_seq, frame->imus, frame->imu_length, frame->imu_channels);
  // }

  // static mpu_sync_frame_t mpu_sync_frame;
  // static int dready_ms = 0;
  // dready_ms++;
  // bool ready = nrf_gpio_pin_read(PIN_MPU_DREADY);
  // if (ready) {
  //   dready_ms = 0;
  //   // read by SPI.
  //   int err = mpu_sync_read(&mpu_sync_frame);
  //   if (!err) {
  //     // notify to BLE.
  //     ble_aaff_notify_commit_mpu(&mpu_sync_frame.eegs, mpu_sync_frame.eeg_length, mpu_sync_frame.eeg_channels);
  //     ble_aaef_notify_commit_mpu(&mpu_sync_frame.imus, mpu_sync_frame.imu_length);
  //   }
  // }

  if (ble_aaff_can_accept_frame() && (eeg_manager_read_frame(&eeg_frame) == 0)) {
    eeg_samples_to_send = eeg_frame.sample_count;
    if (eeg_samples_to_send > TASK_EEG_FRAME_SIZE) {
      eeg_samples_to_send = TASK_EEG_FRAME_SIZE;
    }

    for (uint8_t sample_index = 0; sample_index < eeg_samples_to_send; ++sample_index) {
      for (uint8_t channel_index = 0; channel_index < TASK_EEG_CHANNEL_SIZE; ++channel_index) {
        eeg_notify_frame[sample_index].volt[channel_index] =
          (uint16_t)eeg_frame.volts[sample_index][channel_index];
      }
    }

    (void)ble_aaff_notify_commit_mpu(eeg_seq++, eeg_notify_frame, eeg_samples_to_send, TASK_EEG_CHANNEL_SIZE);
  }

  ble_aaff_loop();
  ble_aaef_loop();
	k_work_reschedule(k_work_delayable_from_work(work), K_MSEC(LOOPER_INTERVAL));
}

int ble_aaaa_init()
{
  int err = 0;
  err = ble_aaff_init();

  // err = k_work_schedule(&looper_work, K_NO_WAIT);
  err = k_work_schedule(&looper_work, K_MSEC(200)); // 延迟 200ms 启动，确保尽可能在最后时刻
  return 0;
}

#endif
