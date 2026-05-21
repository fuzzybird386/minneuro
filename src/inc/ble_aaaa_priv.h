#pragma once
#ifndef BLE_AAAA_PRIV_H__
#define BLE_AAAA_PRIV_H__

#include "minneuro_mode.h"
#include "ble_aaaa.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>


#define AAAA_INDEX            0
#define EEG_AAF0_INDEX        2
#define EEG_AAF0_CCC_INDEX    3
#define EEG_AAFF_INDEX        5
#define EEG_AAFF_CCC_INDEX    6
#define IMU_AAE0_INDEX        8
#define IMU_AAE0_CCC_INDEX    9
#define IMU_AAEF_INDEX        11
#define IMU_AAEF_CCC_INDEX    12

// SERVICE: AAAA

extern struct bt_uuid_128 aaaa_service_uuid;

// EMG/ECG/EOG/EEG Characteristics
//// AAF0: READ STATE
extern struct bt_uuid_128 aaf0_char_uuid;
//// AAFF: STREAM
extern struct bt_uuid_128 aaff_char_uuid;

// IMU/MAG Characteristics
//// AAE0: READ STATE
extern struct bt_uuid_128 aae0_char_uuid;
//// AAEF: STREAM
extern struct bt_uuid_128 aaef_char_uuid;

// PPG Characteristics
//// AAD0: READ STATE
extern struct bt_uuid_128 aad0_char_uuid;
//// AADF: STREAM
extern struct bt_uuid_128 aadf_char_uuid;


void ble_ccc_cfg_changed_aaf0(const struct bt_gatt_attr *attr, uint16_t value);
void ble_ccc_cfg_changed_aaff(const struct bt_gatt_attr *attr, uint16_t value);
ssize_t ble_on_aaf0_read_request(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
ssize_t ble_on_aaf0_write_request(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);

void ble_ccc_cfg_changed_aae0(const struct bt_gatt_attr *attr, uint16_t value);
void ble_ccc_cfg_changed_aaef(const struct bt_gatt_attr *attr, uint16_t value);
ssize_t ble_on_aae0_read_request(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
ssize_t ble_on_aae0_write_request(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);


// void ble_ccc_cfg_changed_aad0(const struct bt_gatt_attr *attr, uint16_t value);
// void ble_ccc_cfg_changed_aadf(const struct bt_gatt_attr *attr, uint16_t value);
extern const struct bt_gatt_service_static aaaa_service;


int ble_aaff_init();
int ble_aaff_loop();

int ble_aaef_loop();

#endif
