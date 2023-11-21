/*
 * controller.h
 *
 *  Created on: Nov 3, 2023
 *      Author: legob
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include <stdint.h>
#include <stdio.h>

#include "stm32g4xx_hal.h"

#include "throttle_map.h"
#include "fir_filter.h"
#include "lsm6dsl.h"
#include "led.h"
#include "button.h"
#include "display.h"

#define ACCEL_FILTER_LEN 20

#define CONTROLLER_LOOP_PERIOD_MS 50

#define CONTROLLER_WAKE_TIME_MS 1000
#define CONTROLLER_ABORT_TIME_MS 1000
#define CONTROLLER_ERROR_TIME_MS 5000

#define CONTROLLER_KP 4
#define CONTROLLER_KI 1
#define CONTROLLER_IMAX 20

#define CONTROLLER_THROTTLE_MAX 60

#define CONTROLLER_MINIMUM_SPEED 5

#define CONTROLLER_ACCEL_THRESHOLD 0.2

typedef enum {
	CONTROLLER_STATE_OFF,
	CONTROLLER_STATE_IDLE,
	CONTROLLER_STATE_WAKE,
	CONTROLLER_STATE_WAIT,
	CONTROLLER_STATE_READY,
	CONTROLLER_STATE_ACTIVE,
	CONTROLLER_STATE_ABORT,
	CONTROLLER_STATE_ERROR
} Controller_State;

typedef enum {
	CONTROLLER_ERROR_OK,
	CONTROLLER_ERROR_CAN,
	CONTROLLER_ERROR_APS
} Controller_Error;

typedef struct {
	Controller_State state;

	uint32_t last_state_transition;
	float set_speed;
	float error_integral;
	Controller_Error error_flag;

	fir_filter_32f_t accel_filter;
} Controller;

void controller_init(void);
float controller_run(float current_speed, float throttle_in, Controller_Error error_flag);

#endif /* INC_CONTROLLER_H_ */
