/**
 ******************************************************************************
 * @file    car_ble_ctrl.h
 * @brief   Bluetooth control for smart car (master mode)
 ******************************************************************************
 * Copyright (c) 2023
 * Licensed under Apache-2.0
 ******************************************************************************
 */

#ifndef _CAR_BLE_CTRL_H_
#define _CAR_BLE_CTRL_H_

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Entry function for car Bluetooth control thread
 * @param parameter unused
 */
void car_ble_thread_entry(void *parameter);

/**
 * @brief Initialize Bluetooth in master mode and connect to car slave
 * @return RT_EOK on success
 */
rt_err_t car_ble_init(void);

void ble_oob_scan(void);

int ble_disc(void);
#ifdef __cplusplus
}
#endif

#endif /* _CAR_BLE_CTRL_H_ */

