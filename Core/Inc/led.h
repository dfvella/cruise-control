/*
 * led.h
 *
 *  Created on: Oct 29, 2023
 *      Author: legob
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include "stm32g4xx_hal.h"

#include "main.h"

void led_r_on(void);
void led_r_blink(void);
void led_r_off(void);

void led_y_on(void);
void led_y_blink(void);
void led_y_off(void);

void led_g_on(void);
void led_g_blink(void);
void led_g_off(void);

#endif /* INC_LED_H_ */
