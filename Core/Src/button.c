/*
 * button.c
 *
 *  Created on: Nov 3, 2023
 *      Author: legob
 */

#include "button.h"

static Button button_r;
static Button button_y;
static Button button_g;

static uint8_t button_get(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, Button *button)
{
	uint8_t res = 0;

	uint8_t pressed = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);

	if (pressed)
	{
		if (!button->pressed_prev)
		{
			button->last_press = HAL_GetTick();
			button->last_get = HAL_GetTick();
			res = 1;
		}
		else if (button->mode == BUTTON_MULTI_PRESS)
		{
			if (HAL_GetTick() - button->last_press > BUTTON_COOLDOWN_INITIAL_MS)
			{
				if (HAL_GetTick() - button->last_get > BUTTON_COOLDOWN_HOLD_MS)
				{
					button->last_get = HAL_GetTick();
					res = 1;
				}
			}
		}
	}

	button->pressed_prev = pressed;

	return res;
}

void button_r_set_mode(Button_Mode mode)
{
	button_r.mode = mode;
}

void button_y_set_mode(Button_Mode mode)
{
	button_y.mode = mode;
}

void button_g_set_mode(Button_Mode mode)
{
	button_g.mode = mode;
}

uint8_t button_r_get(void)
{
	return button_get(BUTTON_R_GPIO_Port, BUTTON_R_Pin, &button_r);
}

uint8_t button_y_get(void)
{
	return button_get(BUTTON_Y_GPIO_Port, BUTTON_Y_Pin, &button_y);
}

uint8_t button_g_get(void)
{
	return button_get(BUTTON_G_GPIO_Port, BUTTON_G_Pin, &button_g);
}
