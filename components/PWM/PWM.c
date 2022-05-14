/*
 * PWM.c
 *
 *  Created on: 2021年9月24日
 *      Author: wxujs
 */
#include "PWM.h"

#define LEDC_HS_CH0_GPIO 3//R
#define LEDC_HS_CH1_GPIO 6//G
#define LEDC_HS_CH2_GPIO 7//B
#define LEDC_HS_CH3_GPIO 18//C
#define LEDC_HS_CH4_GPIO 19//W
ledc_channel_config_t config_ch[5] = {

{
	.channel = LEDC_CHANNEL_0,
	.duty = 250,
	.gpio_num = LEDC_HS_CH0_GPIO,
	.speed_mode = LEDC_LOW_SPEED_MODE,
	.hpoint = 0,
	.timer_sel = LEDC_TIMER_1 }, {
	.channel = LEDC_CHANNEL_1,
	.duty = 0,
	.gpio_num = LEDC_HS_CH1_GPIO,
	.speed_mode = LEDC_LOW_SPEED_MODE,
	.hpoint = 0,
	.timer_sel = LEDC_TIMER_1 },

{
	.channel = LEDC_CHANNEL_2,
	.duty = 0,
	.gpio_num = LEDC_HS_CH2_GPIO,
	.speed_mode = LEDC_LOW_SPEED_MODE,
	.hpoint = 0,
	.timer_sel = LEDC_TIMER_1 }, {
	.channel = LEDC_CHANNEL_3,
	.duty = 255,
	.gpio_num = LEDC_HS_CH3_GPIO,
	.speed_mode = LEDC_LOW_SPEED_MODE,
	.hpoint = 0,
	.timer_sel = LEDC_TIMER_1 }, {
	.channel = LEDC_CHANNEL_4,
	.duty = 255,
	.gpio_num = LEDC_HS_CH4_GPIO,
	.speed_mode = LEDC_LOW_SPEED_MODE,
	.hpoint = 0,
	.timer_sel = LEDC_TIMER_1 } };

void PWMTest()
{
	PWM_Init();
	PWM_SetRGB_8Bit(100, 200, 200);
	PWM_SetCW_8Bit(50,51);
}

void PWM_Init()
{
	ledc_timer_config_t config;
	config.speed_mode = LEDC_LOW_SPEED_MODE;
	config.clk_cfg = LEDC_AUTO_CLK;
	config.freq_hz = 1000;
	config.timer_num = LEDC_TIMER_1;
	config.duty_resolution = LEDC_TIMER_8_BIT;
	ESP_LOGI("PWM", "PWM1: %d", ledc_timer_config(&config));

	for (int i = 0; i < 5; i++)
	{
		ESP_LOGI("PWM", "PWM ledc_channel_config: %d", ledc_channel_config(&config_ch[i]));
	}

	ledc_fade_func_install(0);
}

void PWM_SetRGB_8Bit(uint8_t R, uint8_t G, uint8_t B)
{
//	ledc_fade_stop(config_ch[0].speed_mode, config_ch[0].channel);
//	ledc_fade_stop(config_ch[1].speed_mode, config_ch[1].channel);
//	ledc_fade_stop(config_ch[2].speed_mode, config_ch[2].channel);
	ESP_LOGI("ledc", "PWM_SetRGB_8Bit");
	ledc_set_fade_with_time(config_ch[0].speed_mode, config_ch[0].channel, R, 500);
	ledc_set_fade_with_time(config_ch[1].speed_mode, config_ch[1].channel, G, 500);
	ledc_set_fade_with_time(config_ch[2].speed_mode, config_ch[2].channel, B, 500);
	ledc_fade_start(config_ch[0].speed_mode, config_ch[0].channel, LEDC_FADE_NO_WAIT);
	ledc_fade_start(config_ch[1].speed_mode, config_ch[1].channel, LEDC_FADE_NO_WAIT);
	ledc_fade_start(config_ch[2].speed_mode, config_ch[2].channel, LEDC_FADE_NO_WAIT);
	vTaskDelay(500 / portTICK_PERIOD_MS);
}

void PWM_SetCW_8Bit(uint8_t C,uint8_t W)
{
//	ledc_fade_stop(config_ch[3].speed_mode, config_ch[3].channel);
//	ledc_fade_stop(config_ch[4].speed_mode, config_ch[4].channel);
	ESP_LOGI("ledc", "PWM_SetCW_8BitC:%d w:%d",C,W);
	ledc_set_fade_with_time(config_ch[3].speed_mode, config_ch[3].channel, C, 500);
	ledc_set_fade_with_time(config_ch[4].speed_mode, config_ch[4].channel, W, 500);
    ledc_fade_start(config_ch[3].speed_mode, config_ch[3].channel, LEDC_FADE_NO_WAIT);
	ledc_fade_start(config_ch[4].speed_mode, config_ch[4].channel, LEDC_FADE_NO_WAIT);
	vTaskDelay(500 / portTICK_PERIOD_MS);
}
