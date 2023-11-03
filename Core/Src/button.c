/*
 * button.c
 *
 *  Created on: Nov 3, 2023
 *      Author: legob
 */

#include "button.h"

static uint32_t button_r_last_press;
static uint32_t button_y_last_press;
static uint32_t button_g_last_press;

static uint32_t button_r_last_get;
static uint32_t button_y_last_get;
static uint32_t button_g_last_get;

static uint8_t button_r_prev;
static uint8_t button_y_prev;
static uint8_t button_g_prev;

static Button_Mode button_r_mode;
static Button_Mode button_y_mode;
static Button_Mode button_g_mode;

static uint8_t button_get(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t *last_press, uint32_t *last_get, uint8_t *pressed_prev, Button_Mode mode)
{
	uint8_t res = 0;

	uint8_t pressed = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);

	if (pressed)
	{
		if (!*pressed_prev)
		{
			*last_press = HAL_GetTick();
			*last_get = HAL_GetTick();
			res = 1;
		}
		else if (mode == BUTTON_MULTI_PRESS)
		{
			if (HAL_GetTick() - *last_press > BUTTON_COOLDOWN_INITIAL_MS)
			{
				if (HAL_GetTick() - *last_get > BUTTON_COOLDOWN_HOLD_MS)
				{
					*last_get = HAL_GetTick();
					res = 1;
				}
			}
		}
	}

	*pressed_prev = pressed;

	return res;
}

void button_r_set_mode(Button_Mode mode)
{
	button_r_mode = mode;
}

void button_y_set_mode(Button_Mode mode)
{
	button_y_mode = mode;
}

void button_g_set_mode(Button_Mode mode)
{
	button_g_mode = mode;
}

uint8_t button_r_get(void)
{
	return button_get(BUTTON_R_GPIO_Port, BUTTON_R_Pin, &button_r_last_press, &button_r_last_get, &button_r_prev, button_r_mode);
}

uint8_t button_y_get(void)
{
	return button_get(BUTTON_Y_GPIO_Port, BUTTON_Y_Pin, &button_y_last_press, &button_y_last_get, &button_y_prev, button_y_mode);
}

uint8_t button_g_get(void)
{
	return button_get(BUTTON_G_GPIO_Port, BUTTON_G_Pin, &button_g_last_press, &button_g_last_get, &button_g_prev, button_g_mode);
}
