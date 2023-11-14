/*
 * button.h
 *
 *  Created on: Nov 3, 2023
 *      Author: legob
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include "stm32g4xx_hal.h"

#include "main.h"

#define BUTTON_COOLDOWN_INITIAL_MS 500
#define BUTTON_COOLDOWN_HOLD_MS 100

typedef enum {
	BUTTON_SINGLE_PRESS,
	BUTTON_MULTI_PRESS
} Button_Mode;

typedef struct {
	uint32_t last_press;
	uint32_t last_get;
	uint8_t pressed_prev;
	Button_Mode mode;
} Button;

void button_r_set_mode(Button_Mode mode);
void button_y_set_mode(Button_Mode mode);
void button_g_set_mode(Button_Mode mode);

uint8_t button_r_get(void);
uint8_t button_y_get(void);
uint8_t button_g_get(void);

#endif /* INC_BUTTON_H_ */
