/*
 * PWM.h
 *
 *  Created on: 2021年9月24日
 *      Author: wxujs
 */

#ifndef COMPONENTS_PWM_PWM_H_
#define COMPONENTS_PWM_PWM_H_

#include "esp_log.h"
#include "driver/ledc.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void PWMTest();

void PWM_Init();

void PWM_SetRGB_8Bit(uint8_t R,uint8_t G,uint8_t B);

void PWM_SetCW_8Bit(uint8_t C,uint8_t W);

#endif /* COMPONENTS_PWM_PWM_H_ */
