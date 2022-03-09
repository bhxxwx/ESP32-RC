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

void board_init();

void OnOff_key(void *arg);
void MainColorLightLevelTog_key(void *arg);
void LightLevel_key(void *arg);
void ColorTempLevel_key(void *arg);
void RGBChange_key(void *arg);
void SenceMode_key(void *arg);
void Dance_key(void *arg);


#endif
