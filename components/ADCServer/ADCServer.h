/*
 * ADC.h
 *
 *  Created on: 2021年9月24日
 *      Author: wxujs
 */

#ifndef MAIN_DRIVER_ADC_ADC_H_
#define MAIN_DRIVER_ADC_ADC_H_
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include <stdio.h>
void ADCTest();
uint16_t ADC_GetValue();
void ADC_Init();

#endif /* MAIN_DRIVER_ADC_ADC_H_ */
