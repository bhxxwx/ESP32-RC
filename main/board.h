/*
 * @Author: Wangxiang
 * @Date: 2022-03-02 16:08:47
 * @LastEditTime: 2022-04-08 10:42:49
 * @LastEditors: Wangxiang
 * @Description: 
 * @FilePath: /ESP32_VndServer_RC/main/board.h
 * 江苏大学-王翔
 */
/* board.h - Board-specific hooks */

/*
 * Copyright (c) 2017 Intel Corporation
 * Additional Copyright (c) 2018 Espressif Systems (Shanghai) PTE LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _BOARD_H_
#define _BOARD_H_

#include "driver/gpio.h"
#include "iot_button.h"
#include <string.h>
#include "CommandSystem.h"

typedef struct
{
	uint8_t pin;
	void (*Func)(void *arg);
	char *value;
	uint8_t triger;
} PinFuncMap_t;

void board_init(uint8_t mode);

void OnOff_key(void *arg);
void MainColorLightTog_key(void *arg);
void MainColorLightLevelFuncSwitch(void *arg);
void LightLevel_key(void *arg);
void ColorTempLevel_key(void *arg);
void RGBChange_key(void *arg);
void SenceMode_key(void *arg);
void Dance_key(void *arg);
void LED_op(bool states);
void change_mode(uint8_t mode);
#endif
