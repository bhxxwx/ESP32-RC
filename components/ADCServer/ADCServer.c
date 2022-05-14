/*
 * ADC.c
 *
 *  Created on: 2021年9月24日
 *      Author: wxujs
 */

#include "ADCServer.h"

void ADCTest()
{
	ESP_LOGI("ADC.c", "This is ADCTest Func");
	ADC_Init();
	uint16_t ADC1Value = ADC_GetValue();
	ESP_LOGI("ADC.c", "ADC1_CHANNEL_0 Value is: %d",ADC1Value);
}

/**
 * @fn void ADC_Init()
 * @brief ESP32 ADC1初始化函数,对应的衰减倍数如下表
 *
+----------+-------------+-----------------+
|          | attenuation | suggested range |
|    SoC   |     (dB)    |      (mV)       |
+==========+=============+=================+
|          |       0     |    100 ~  950   |
|          +-------------+-----------------+
|          |       2.5   |    100 ~ 1250   |
|   ESP32  +-------------+-----------------+
|          |       6     |    150 ~ 1750   |
|          +-------------+-----------------+
|          |      11     |    150 ~ 2450   |
+----------+-------------+-----------------+
 */
void ADC_Init()
{
	adc1_config_width(ADC1_CHANNEL_0);
	adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_11);
	adc1_config_width(ADC_WIDTH_BIT_12);
}

uint16_t ADC_GetValue()
{
	return adc1_get_raw(ADC1_CHANNEL_0);
}

