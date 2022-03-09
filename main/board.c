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

PinFuncMap_t PinFuncMap[] = {
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
//LinkPinToFunc(GPIO_NUM_7,Dance_key,"2",BUTTON_ACTIVE_LOW),
LinkPinToFunc(GPIO_NUM_19,Dance_key,"3",BUTTON_ACTIVE_LOW),
//LinkPinToFunc(GPIO_NUM_10,Dance_key,"4",BUTTON_ACTIVE_LOW),

LinkPinToFunc(GPIO_NUM_21,SenceMode_key,"1",BUTTON_ACTIVE_LOW),
//LinkPinToFunc(GPIO_NUM_9,SenceMode_key,"2",BUTTON_ACTIVE_LOW),
LinkPinToFunc(GPIO_NUM_13,SenceMode_key,"3",BUTTON_ACTIVE_LOW), };

extern void example_ble_mesh_send_vendor_message(uint8_t *data, uint8_t len);
static bool _MC_switch = false;

void OnOff_key(void *arg)
{
	uint8_t data[2] = { 0 };
	data[0] = 4;
	data[1] = 1;
	example_ble_mesh_send_vendor_message(data, 2);
	ESP_LOGI(TAG,"%s",__func__);
}

void MainColorLightLevelTog_key(void *arg)
{
	_MC_switch = *((uint8_t*) arg) == 49 ? 1 : 0;
	ESP_LOGI(TAG,"%s",__func__);
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
	ESP_LOGI(TAG,"%s",__func__);
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
	ESP_LOGI(TAG,"%s",__func__);
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
	ESP_LOGI(TAG,"%s",__func__);
}

void Dance_key(void *arg)
{
	uint8_t data[2] = { 0 };
	data[0] = 5;
	data[1] = *((uint8_t*) arg) - 48;
	example_ble_mesh_send_vendor_message(data, 2);
	ESP_LOGI(TAG,"%s",__func__);
}

void SenceMode_key(void *arg)
{
	uint8_t data[2] = { 0 };
	data[0] = 3;
	data[1] = *((uint8_t*) arg) - 48;
	example_ble_mesh_send_vendor_message(data, 2);
	ESP_LOGI(TAG,"%s",__func__);
}

//void button_tap_cb(void *arg)
//{
//	uint8_t key = *(uint8_t*) arg;
//	ESP_LOGI(TAG, "GPIO %d BUTTON_ACTIVE_LOW", key);
//	switch (key)
//	{
//		case GPIO_NUM_34:
//		{
//			uint8_t data[8] = { 0 };
////			SendMsg(data,8);
//			break;
//		}
//	}
//}

void board_init()
{
	ADC_Init();
	for (int i = 0; i < ARRAY_SIZE(PinFuncMap); i++)
	{
		button_handle_t btn_handle = iot_button_create(PinFuncMap[i].pin, BUTTON_ACTIVE_LOW);
		if (btn_handle)
		{
			ESP_LOGI(TAG, "GPIO %d Init ok", PinFuncMap[i].pin);
			iot_button_set_evt_cb(btn_handle, BUTTON_CB_RELEASE, PinFuncMap[i].Func, &PinFuncMap[i].value);
		}
		else
		{
			ESP_LOGE(TAG, "GPIO %d Init failure", PinFuncMap[i].pin);
		}
	}
}
