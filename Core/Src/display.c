/*
 * display.c
 *
 *  Created on: Oct 27, 2023
 *      Author: legob
 */

#include "display.h"

static Display_t display;

static void draw_mask(uint8_t mask_1, uint8_t mask_2)
{
	HAL_GPIO_WritePin(DISP_1A_GPIO_Port, DISP_1A_Pin, mask_1 & (1 << 0));
	HAL_GPIO_WritePin(DISP_1B_GPIO_Port, DISP_1B_Pin, mask_1 & (1 << 1));
	HAL_GPIO_WritePin(DISP_1C_GPIO_Port, DISP_1C_Pin, mask_1 & (1 << 2));
	HAL_GPIO_WritePin(DISP_1D_GPIO_Port, DISP_1D_Pin, mask_1 & (1 << 3));
	HAL_GPIO_WritePin(DISP_1E_GPIO_Port, DISP_1E_Pin, mask_1 & (1 << 4));
	HAL_GPIO_WritePin(DISP_1F_GPIO_Port, DISP_1F_Pin, mask_1 & (1 << 5));
	HAL_GPIO_WritePin(DISP_1G_GPIO_Port, DISP_1G_Pin, mask_1 & (1 << 6));

	HAL_GPIO_WritePin(DISP_2A_GPIO_Port, DISP_2A_Pin, mask_2 & (1 << 0));
	HAL_GPIO_WritePin(DISP_2B_GPIO_Port, DISP_2B_Pin, mask_2 & (1 << 1));
	HAL_GPIO_WritePin(DISP_2C_GPIO_Port, DISP_2C_Pin, mask_2 & (1 << 2));
	HAL_GPIO_WritePin(DISP_2D_GPIO_Port, DISP_2D_Pin, mask_2 & (1 << 3));
	HAL_GPIO_WritePin(DISP_2E_GPIO_Port, DISP_2E_Pin, mask_2 & (1 << 4));
	HAL_GPIO_WritePin(DISP_2F_GPIO_Port, DISP_2F_Pin, mask_2 & (1 << 5));
	HAL_GPIO_WritePin(DISP_2G_GPIO_Port, DISP_2G_Pin, mask_2 & (1 << 6));
}

static uint8_t digit_mask(uint8_t digit)
{
	switch (digit)
	{
	case 0:
		return 0b0111111;
	case 1:
		return 0b0000110;
	case 2:
		return 0b1011011;
	case 3:
		return 0b1001111;
	case 4:
		return 0b1100110;
	case 5:
		return 0b1101101;
	case 6:
		return 0b1111101;
	case 7:
		return 0b0000111;
	case 8:
		return 0b1111111;
	case 9:
		return 0b1101111;
	default:
		break;
	}
	return 0;
}

static void draw_number(uint8_t number)
{
	uint8_t mask_1 = number / 10 ? digit_mask(number / 10) : 0;
	uint8_t mask_2 = digit_mask(number % 10);

	draw_mask(mask_1, mask_2);
}

static void draw_spin(void)
{
	static uint8_t mask = 0b0000011;

	if (HAL_GetTick() - display.last_draw > 50)
	{
		draw_mask(mask, mask);

		if (mask == 0b0110000)
		{
			mask=  0b0100001;
		}
		else if (mask == 0b0100001)
		{
			mask = 0b0000011;
		}
		else
		{
			mask <<= 1;
		}

		display.last_draw = HAL_GetTick();
	}
}

static void turn_off(void)
{
	draw_mask(0, 0);
}

void Display_init(void)
{
	display.last_draw = 0;
	display.number = 0;
	display.state = DISPLAY_OFF;
}

void Display_spin(void)
{
	display.state = DISPLAY_SPIN;
}

void Display_set(uint8_t number)
{
	display.state = DISPLAY_ON;
	display.number = number;
}

void Display_draw(void)
{
	switch (display.state)
	{
	case DISPLAY_OFF:
		turn_off();
		break;

	case DISPLAY_SPIN:
		draw_spin();
		break;

	case DISPLAY_ON:
		draw_number(display.number);
		break;
	}
}

void Display_off(void)
{
	display.state = DISPLAY_OFF;
}
