/*
 * led.c
 *
 *  Created on: Oct 29, 2023
 *      Author: legob
 */

#include "led.h"

static inline GPIO_PinState blink(void)
{
	return (HAL_GetTick() % 1000) > 500;
}

void led_r_on(void)
{
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
}

void led_r_blink(void)
{
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, blink());
}

void led_r_off(void)
{
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
}

void led_y_on(void)
{
	HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);
}

void led_y_blink(void)
{
	HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, blink());
}

void led_y_off(void)
{
	HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_RESET);
}

void led_g_on(void)
{
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
}

void led_g_blink(void)
{
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, blink());
}

void led_g_off(void)
{
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
}
