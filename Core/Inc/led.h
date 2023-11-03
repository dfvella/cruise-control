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

#define LED_BLINK_PERIOD_MS 1000
#define LED_STROBE_PERIOD_MS 200

typedef enum {
	LED_ON,
	LED_BLINK,
	LED_STROBE,
	LED_OFF
} Led_Mode;

void led_r_set(Led_Mode mode);
void led_y_set(Led_Mode mode);
void led_g_set(Led_Mode mode);

void led_update(void);

#endif /* INC_LED_H_ */
