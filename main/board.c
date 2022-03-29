/* board.c - Board-specific hooks */

/*
 * Copyright (c) 2017 Intel Corporation
 * Additional Copyright (c) 2018 Espressif Systems (Shanghai) PTE LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "board.h"
#include "ADCServer.h"
//#include "PWM.h"

#define TAG "BOARD"

#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

#define LinkPinToFunc(_pin,_func,_value,_triger)	{.pin = (_pin), .Func = (_func), .value = (_value),.triger=(_triger)}

PinFuncMap_t ShortPressPinFuncMap[] = {
LinkPinToFunc(GPIO_NUM_34,OnOff_key,"1",BUTTON_ACTIVE_LOW),
LinkPinToFunc(GPIO_NUM_39,MainColorLightLevelTog_key,"1",BUTTON_ACTIVE_LOW),
LinkPinToFunc(GPIO_NUM_35,MainColorLightLevelTog_key,"0",BUTTON_ACTIVE_LOW),
LinkPinToFunc(GPIO_NUM_32,LightLevel_key,"0",BUTTON_ACTIVE_LOW),
LinkPinToFunc(GPIO_NUM_25,LightLevel_key,"1",BUTTON_ACTIVE_LOW),

LinkPinToFunc(GPIO_NUM_26,ColorTempLevel_key,"0",BUTTON_ACTIVE_LOW),
LinkPinToFunc(GPIO_NUM_4,ColorTempLevel_key,"1",BUTTON_ACTIVE_LOW),

LinkPinToFunc(GPIO_NUM_23,RGBChange_key,"0",BUTTON_ACTIVE_LOW),
LinkPinToFunc(GPIO_NUM_16,RGBChange_key,"1",BUTTON_ACTIVE_LOW),

LinkPinToFunc(GPIO_NUM_5,Dance_key,"1",BUTTON_ACTIVE_LOW),
LinkPinToFunc(GPIO_NUM_18,Dance_key,"2",BUTTON_ACTIVE_LOW),
LinkPinToFunc(GPIO_NUM_19,Dance_key,"3",BUTTON_ACTIVE_LOW),
LinkPinToFunc(GPIO_NUM_33,Dance_key,"4",BUTTON_ACTIVE_LOW),

LinkPinToFunc(GPIO_NUM_21,SenceMode_key,"14",BUTTON_ACTIVE_LOW),
LinkPinToFunc(GPIO_NUM_22,SenceMode_key,"25",BUTTON_ACTIVE_LOW),
LinkPinToFunc(GPIO_NUM_13,SenceMode_key,"36",BUTTON_ACTIVE_LOW),
//LinkPinToFunc(GPIO_NUM_0,SenceMode_key,"36",BUTTON_ACTIVE_LOW),
};


extern void example_ble_mesh_send_vendor_message(uint8_t *data, uint8_t len);
void LED_blink();
static bool _MC_switch = false;
static uint8_t _mode;

void OnOff_key(void *arg)
{
	uint8_t data[2] = { 0 };
	data[0] = 4;
	data[1] = 1;
	example_ble_mesh_send_vendor_message(data, 2);
	LED_blink();
	ESP_LOGI(TAG, "%s(%d)", __func__, *((uint8_t* ) arg));
}

void MainColorLightLevelTog_key(void *arg)
{
	_MC_switch = *((uint8_t*) arg) == 49 ? 1 : 0;
	LED_blink();
	ESP_LOGI(TAG, "%s(%d)", __func__, *((uint8_t* ) arg));
}

void LightLevel_key(void *arg)
{
	uint8_t data[3] = { 0 };
	data[0] = 1;
	data[1] = _MC_switch ? 4 : 2;
	if (*((uint8_t*) arg) != '$')
		data[2] = *((uint8_t*) arg) == 49 ? 1 : 0;
	else
		DecodeCommandValue(arg, &data[2]);
	example_ble_mesh_send_vendor_message(data, 3);
	LED_blink();
	ESP_LOGI(TAG, "%s(%d)", __func__, *((uint8_t* ) arg));
}

void ColorTempLevel_key(void *arg)
{
	uint8_t data[3] = { 0 };
	data[0] = 1;
	data[1] = 1;
	if (*((uint8_t*) arg) != '$')
		data[2] = *((uint8_t*) arg) == 49 ? 1 : 0;
	else
		DecodeCommandValue(arg, &data[2]);
	example_ble_mesh_send_vendor_message(data, 3);
	LED_blink();
	ESP_LOGI(TAG, "%s(%d)", __func__, *((uint8_t* ) arg));
}

void RGBChange_key(void *arg)
{
	uint8_t data[3] = { 0 };
	data[0] = 1;
	data[1] = 3;
	if (*((uint8_t*) arg) != '$')
		data[2] = *((uint8_t*) arg) == 49 ? 1 : 0;
	else
		DecodeCommandValue(arg, &data[2]);
	example_ble_mesh_send_vendor_message(data, 3);
	LED_blink();
	ESP_LOGI(TAG, "%s(%d)", __func__, *((uint8_t* ) arg));
}

void Dance_key(void *arg)
{
	uint8_t data[2] = { 0 };
	data[0] = 5;
	data[1] = *((uint8_t*) arg) - 48;
	example_ble_mesh_send_vendor_message(data, 2);
	LED_blink();
	ESP_LOGI(TAG, "%s(%d)", __func__, *((uint8_t* ) arg));
}

void SenceMode_key(void *arg)
{
	uint8_t data[2] = { 0 };
	static bool long_tab=false;
	if(long_tab)
	{
		long_tab=false;
		return;
	}
	if ((*((uint8_t*) arg) - 48) <= 3)
	{
		data[0] = 2;
		data[1] = *((uint8_t*) arg) - 48;
		example_ble_mesh_send_vendor_message(data, 2);
		LED_blink();
		ESP_LOGI(TAG, "short %s(%d)", __func__, *((uint8_t* ) arg));
	}
	else
	{
		long_tab=true;
		data[0] = 3;
		data[1] = *((uint8_t*) arg) - '3';
		example_ble_mesh_send_vendor_message(data, 2);
		LED_blink();
		ESP_LOGI(TAG, "long %s(%d)", __func__, *((uint8_t* ) arg));
	}

}

void LED_blink()
{
	if(_mode)
	{
		gpio_set_level(GPIO_NUM_14, 0);
		esp_rom_delay_us(800000);
		gpio_set_level(GPIO_NUM_14, 1);
	}
	else
	{
		gpio_set_level(GPIO_NUM_14, 1);
		esp_rom_delay_us(800000);
		gpio_set_level(GPIO_NUM_14, 0);
	}
}

void LED_op(bool states)
{
	gpio_set_level(GPIO_NUM_14, states);
}
void change_mode(uint8_t mode)
{
	_mode = mode;
}
// extern void hallSenor_task(void *arg);
void board_init(uint8_t mode)
{
	gpio_config_t io_conf ;
	_mode=mode;
	ADC_Init();
	
	io_conf.intr_type=GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = 1ULL<<GPIO_NUM_14;
	//disable pull-down mode
	io_conf.pull_down_en = 0;
	//disable pull-up mode
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf );
	gpio_set_level(GPIO_NUM_14, _mode?1:0);
	for (int i = 0; i < ARRAY_SIZE(ShortPressPinFuncMap); i++)
	{
		button_handle_t btn_handle = iot_button_create(ShortPressPinFuncMap[i].pin, BUTTON_ACTIVE_LOW);
		if (btn_handle)
		{
			ESP_LOGI(TAG, "GPIO %d Init ok", ShortPressPinFuncMap[i].pin);
			iot_button_set_evt_cb(btn_handle, BUTTON_CB_RELEASE, ShortPressPinFuncMap[i].Func, ShortPressPinFuncMap[i].value);
			if(ShortPressPinFuncMap[i].Func==SenceMode_key)
			{
				iot_button_add_custom_cb(btn_handle, 2, ShortPressPinFuncMap[i].Func, ShortPressPinFuncMap[i].value+1);
			}
		}
		else
		{
			ESP_LOGE(TAG, "GPIO %d Init failure", ShortPressPinFuncMap[i].pin);
		}
	}

	// button_handle_t btn_handle = iot_button_create(GPIO_NUM_36, BUTTON_ACTIVE_LOW);
	// iot_button_set_evt_cb(btn_handle, BUTTON_CB_PUSH, hallSenor_task, "0");
	// iot_button_set_evt_cb(btn_handle, BUTTON_CB_RELEASE, hallSenor_task, "1");
}
