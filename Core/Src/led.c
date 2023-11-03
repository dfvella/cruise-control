/*
 * led.c
 *
 *  Created on: Oct 29, 2023
 *      Author: legob
 */

#include "led.h"

static Led_Mode led_r_mode;
static Led_Mode led_y_mode;
static Led_Mode led_g_mode;

static GPIO_PinState blink(void)
{
	return (HAL_GetTick() % LED_BLINK_PERIOD_MS) > (LED_BLINK_PERIOD_MS / 2);
}

static GPIO_PinState strobe(void)
{
	return (HAL_GetTick() % LED_STROBE_PERIOD_MS) > (LED_STROBE_PERIOD_MS / 2);
}

static void led_write(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, Led_Mode mode)
{
	GPIO_PinState state = 0;

	switch (mode)
	{
	case LED_OFF:
		state = GPIO_PIN_RESET;
		break;

	case LED_BLINK:
		state = blink();
		break;

	case LED_STROBE:
		state = strobe();
		break;

	case LED_ON:
		state = GPIO_PIN_SET;
		break;

	default:
		break;
	}

	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, state);
}

void led_r_set(Led_Mode mode)
{
	led_r_mode = mode;
}

void led_y_set(Led_Mode mode)
{
	led_y_mode = mode;
}

void led_g_set(Led_Mode mode)
{
	led_g_mode = mode;
}

void led_update(void)
{
	led_write(LED_R_GPIO_Port, LED_R_Pin, led_r_mode);
	led_write(LED_Y_GPIO_Port, LED_Y_Pin, led_y_mode);
	led_write(LED_G_GPIO_Port, LED_G_Pin, led_g_mode);
}
