/*
 * display.h
 *
 *  Created on: Oct 27, 2023
 *      Author: legob
 */

#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

#include <stdint.h>

#include "stm32g4xx_hal.h"

#include "main.h"

typedef enum {
	DISPLAY_OFF,
	DISPLAY_SPIN,
	DISPLAY_ON
} Display_State_t;

typedef struct {
	Display_State_t state;
	uint8_t number;
	uint32_t last_draw;
} Display_t;

void Display_init(void);
void Display_spin(void);
void Display_set(uint8_t number);
void Display_draw(void);
void Display_off(void);

#endif /* INC_DISPLAY_H_ */
