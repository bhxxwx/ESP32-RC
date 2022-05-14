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
#include "driver/rtc_io.h"
#include "esp32/ulp.h"
#include "ulp_main.h"
#include "esp_sleep.h"
//#include "PWM.h"

#define TAG "BOARD"
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_main_bin_end");
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

#define LinkPinToFunc(_pin, _func, _value, _triger)                            \
	{                                                                          \
		.pin = (_pin), .Func = (_func), .value = (_value), .triger = (_triger) \
	}

static void init_ulp_program(void);

PinFuncMap_t ShortPressPinFuncMap[] = {
	LinkPinToFunc(GPIO_NUM_34, OnOff_key, "1", BUTTON_ACTIVE_LOW),
	LinkPinToFunc(GPIO_NUM_39, MainColorLightTog_key, "1", BUTTON_ACTIVE_LOW),
	LinkPinToFunc(GPIO_NUM_35, MainColorLightTog_key, "0", BUTTON_ACTIVE_LOW),
	LinkPinToFunc(GPIO_NUM_32, LightLevel_key, "0", BUTTON_ACTIVE_LOW),
	LinkPinToFunc(GPIO_NUM_25, LightLevel_key, "1", BUTTON_ACTIVE_LOW),

	LinkPinToFunc(GPIO_NUM_15, MainColorLightLevelFuncSwitch, "0", BUTTON_ACTIVE_HIGH),
	LinkPinToFunc(GPIO_NUM_26, ColorTempLevel_key, "0", BUTTON_ACTIVE_LOW),
	LinkPinToFunc(GPIO_NUM_4, ColorTempLevel_key, "1", BUTTON_ACTIVE_LOW),

	LinkPinToFunc(GPIO_NUM_2, MainColorLightLevelFuncSwitch, "1", BUTTON_ACTIVE_HIGH),
	LinkPinToFunc(GPIO_NUM_23, RGBChange_key, "0", BUTTON_ACTIVE_LOW),
	LinkPinToFunc(GPIO_NUM_16, RGBChange_key, "1", BUTTON_ACTIVE_LOW),

	LinkPinToFunc(GPIO_NUM_5, Dance_key, "1", BUTTON_ACTIVE_LOW),
	LinkPinToFunc(GPIO_NUM_18, Dance_key, "2", BUTTON_ACTIVE_LOW),
	LinkPinToFunc(GPIO_NUM_19, Dance_key, "3", BUTTON_ACTIVE_LOW),
	LinkPinToFunc(GPIO_NUM_33, Dance_key, "4", BUTTON_ACTIVE_LOW),

	LinkPinToFunc(GPIO_NUM_21, SenceMode_key, "14", BUTTON_ACTIVE_LOW),
	LinkPinToFunc(GPIO_NUM_22, SenceMode_key, "25", BUTTON_ACTIVE_LOW),
	LinkPinToFunc(GPIO_NUM_13, SenceMode_key, "36", BUTTON_ACTIVE_LOW),
	// LinkPinToFunc(GPIO_NUM_0,SenceMode_key,"36",BUTTON_ACTIVE_LOW),
};

extern void
example_ble_mesh_send_vendor_message(uint8_t *data, uint8_t len);
void LED_blink();
static bool _MC_switch = false;
static uint8_t _mode;

void OnOff_key(void *arg)
{
	uint8_t data[2] = {0};
	data[0] = 4;
	data[1] = 1;
	example_ble_mesh_send_vendor_message(data, 2);
	LED_blink();
	ESP_LOGI(TAG, "%s(%d)", __func__, *((uint8_t *)arg));
}

void MainColorLightTog_key(void *arg)
{
	uint8_t data[2] = {0};
	// _MC_switch = *((uint8_t*) arg) == 49 ? 1 : 0;
	data[0] = 4;
	data[1] = *((uint8_t *)arg) == 49 ? 2 : 3;
	example_ble_mesh_send_vendor_message(data, 2);
	LED_blink();
	ESP_LOGI(TAG, "%s(%d)", __func__, *((uint8_t *)arg));
}

void MainColorLightLevelFuncSwitch(void *arg)
{
	_MC_switch = *((uint8_t *)arg) == 49 ? 1 : 0;
	LED_blink();
	ESP_LOGI(TAG, "%s(%d)", __func__, *((uint8_t *)arg));
}

void LightLevel_key(void *arg)
{
	uint8_t data[3] = {0};
	data[0] = 1;
	data[1] = _MC_switch ? 4 : 2;
	if (*((uint8_t *)arg) != '$')
		data[2] = *((uint8_t *)arg) == 49 ? 1 : 0;
	else
		DecodeCommandValue(arg, &data[2]);
	example_ble_mesh_send_vendor_message(data, 3);
	LED_blink();
	ESP_LOGI(TAG, "%s(%d)", __func__, *((uint8_t *)arg));
}

void ColorTempLevel_key(void *arg)
{
	uint8_t data[3] = {0};
	data[0] = 1;
	data[1] = 1;
	if (*((uint8_t *)arg) != '$')
		data[2] = *((uint8_t *)arg) == 49 ? 1 : 0;
	else
		DecodeCommandValue(arg, &data[2]);
	example_ble_mesh_send_vendor_message(data, 3);
	LED_blink();
	ESP_LOGI(TAG, "%s(%d)", __func__, *((uint8_t *)arg));
}

void RGBChange_key(void *arg)
{
	uint8_t data[3] = {0};
	data[0] = 1;
	data[1] = 3;
	if (*((uint8_t *)arg) != '$')
		data[2] = *((uint8_t *)arg) == 49 ? 1 : 0;
	else
		DecodeCommandValue(arg, &data[2]);
	example_ble_mesh_send_vendor_message(data, 3);
	LED_blink();
	ESP_LOGI(TAG, "%s(%d)", __func__, *((uint8_t *)arg));
}

void Dance_key(void *arg)
{
	uint8_t data[2] = {0};
	data[0] = 5;
	data[1] = *((uint8_t *)arg) - 48;
	example_ble_mesh_send_vendor_message(data, 2);
	LED_blink();
	ESP_LOGI(TAG, "%s(%d)", __func__, *((uint8_t *)arg));
}

void SenceMode_key(void *arg)
{
	uint8_t data[2] = {0};
	static bool long_tab = false;
	if (long_tab)
	{
		long_tab = false;
		return;
	}
	if ((*((uint8_t *)arg) - 48) <= 3)
	{
		data[0] = 2;
		data[1] = *((uint8_t *)arg) - 48;
		example_ble_mesh_send_vendor_message(data, 2);
		LED_blink();
		ESP_LOGI(TAG, "short %s(%d)", __func__, *((uint8_t *)arg));
	}
	else
	{
		long_tab = true;
		data[0] = 3;
		data[1] = *((uint8_t *)arg) - '3';
		example_ble_mesh_send_vendor_message(data, 2);
		LED_blink();
		ESP_LOGI(TAG, "long %s(%d)", __func__, *((uint8_t *)arg));
	}
}

void LED_blink()
{
	if (_mode)
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
	gpio_config_t io_conf;
	_mode = mode;
	ADC_Init();
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = (1ULL << GPIO_NUM_14)|(1UL<<GPIO_NUM_0);
	// disable pull-down mode
	io_conf.pull_down_en = 0;
	// disable pull-up mode
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);
	gpio_set_level(GPIO_NUM_14, _mode ? 1 : 0);
	for (int i = 0; i < ARRAY_SIZE(ShortPressPinFuncMap); i++)
	{
		button_handle_t btn_handle = iot_button_create(ShortPressPinFuncMap[i].pin, ShortPressPinFuncMap[i].triger);
		if (btn_handle)
		{
			// ESP_LOGI(TAG, "GPIO %d Init ok", ShortPressPinFuncMap[i].pin);
			iot_button_set_evt_cb(btn_handle, BUTTON_CB_RELEASE, ShortPressPinFuncMap[i].Func, ShortPressPinFuncMap[i].value);
			if (ShortPressPinFuncMap[i].Func == SenceMode_key)
			{
				iot_button_add_custom_cb(btn_handle, 2, ShortPressPinFuncMap[i].Func, ShortPressPinFuncMap[i].value + 1);
			}
		}
		else
		{
			ESP_LOGE(TAG, "GPIO %d Init failure", ShortPressPinFuncMap[i].pin);
		}
	}
}

void power_manage_init()
{
	esp_err_t err = ESP_OK;

	esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
	if (cause != ESP_SLEEP_WAKEUP_ULP && cause != ESP_SLEEP_WAKEUP_TIMER)
	{
		printf("Not ULP and Timer wakeup, initializing ULP\n");
		init_ulp_program();
	}
	else
	{
		if (cause == ESP_SLEEP_WAKEUP_ULP)
		{
			ESP_LOGI("PowerManage", "ESP_SLEEP_WAKEUP_ULP");
			uint8_t idx = 0;
			idx = (ulp_wake_up_index & UINT16_MAX);
			ShortPressPinFuncMap[idx].Func(ShortPressPinFuncMap[idx].value);
		}
			
		if (cause == ESP_SLEEP_WAKEUP_TIMER)
			ESP_LOGI("PowerManage", "ESP_SLEEP_WAKEUP_TIMER");
	}
	// err=esp_sleep_enable_ulp_wakeup();
	ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());

	
}

static void init_ulp_program(void)
{
	esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
									(ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
	ESP_ERROR_CHECK(err);
	
	/* Set ULP wake up period to T = 20ms.
	 * Minimum pulse width has to be T * (ulp_debounce_counter + 1) = 80ms.
	 */
	ulp_set_wakeup_period(0, 50000);
	ulp_dect_io = 0;
	/* Start the program */
	err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
	ESP_ERROR_CHECK(err);
}

void rtc_io_init()
{
	uint32_t *io_active_low = &ulp_io_number_active_low;
	uint8_t count = 0;
	for (int i = 0; i < ARRAY_SIZE(ShortPressPinFuncMap); i++)
	{
		gpio_num_t gpio_num = ShortPressPinFuncMap[i].pin;
		if (gpio_num == GPIO_NUM_12 || gpio_num == GPIO_NUM_15 || gpio_num==GPIO_NUM_2)
			continue;
		int rtcio_num = rtc_io_number_get(gpio_num);
		if (rtcio_num==-1)
		{
			continue;
		}
		io_active_low[count] = rtcio_num;
		count++;
		assert(rtc_gpio_is_valid_gpio(gpio_num) && "GPIO used for pulse counting must be an RTC IO");
		rtc_gpio_init(gpio_num);
		rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_INPUT_ONLY);
		rtc_gpio_pulldown_dis(gpio_num);
		rtc_gpio_pullup_dis(gpio_num);
		rtc_gpio_hold_en(gpio_num);
	}
	ulp_io_numbers_active_low = count;
	rtc_gpio_isolate(GPIO_NUM_12);
	rtc_gpio_isolate(GPIO_NUM_15);
	rtc_gpio_isolate(GPIO_NUM_2);
	// rtc_gpio_isolate(GPIO_NUM_0);
	esp_deep_sleep_disable_rom_logging(); // suppress boot messages

	// for (int j = 0; j < count;j++)
	// {
	// 	ESP_LOGI("PIN", "---->%d", io_active_low[j] & UINT16_MAX);
	// }
}
void rtc_io_deinit()
{
	for (int i = 0; i < ARRAY_SIZE(ShortPressPinFuncMap); i++)
	{
		gpio_num_t gpio_num = ShortPressPinFuncMap[i].pin;
		if (gpio_num == GPIO_NUM_12 || gpio_num == GPIO_NUM_15)
			continue;
		if (rtc_io_number_get(gpio_num) == -1)
		{
			continue;
		}
		rtc_gpio_deinit(gpio_num);
	}
}

void enter_light_sleep()
{
	uint8_t idx = 0;
	rtc_io_init();
	ESP_LOGI("PowerManage", "Enter light sleep");
	ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(60000000));
	ulp_dect_io = 1;
	gpio_set_level(GPIO_NUM_0, 0);
	rtc_gpio_hold_en(GPIO_NUM_0);
	esp_light_sleep_start();
	printf("active pin:%d", ulp_wake_up_io & UINT16_MAX);
	esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
	ulp_dect_io = 0;
	rtc_io_deinit();
	esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
	if (cause == ESP_SLEEP_WAKEUP_TIMER)
	{
		ESP_LOGI("PowerManage", "Enter deep sleep");
		vTaskDelay(100 / portTICK_RATE_MS);
		enter_deep_sleep();
	}
	gpio_set_level(GPIO_NUM_0, 1);
}
void enter_deep_sleep()
{
	rtc_io_init();
	gpio_set_level(GPIO_NUM_0, 0);
	rtc_gpio_hold_en(GPIO_NUM_0);
	ulp_dect_io = 1;
	esp_deep_sleep_start();
}
