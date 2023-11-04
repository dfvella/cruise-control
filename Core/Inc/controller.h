/*
 * controller.h
 *
 *  Created on: Nov 3, 2023
 *      Author: legob
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include <stdint.h>

#include "throttle_map.h"

#define CONTROLLER_LOOP_PERIOD_MS 50

#define CONTROLLER_WAKE_TIME_MS 1000
#define CONTROLLER_ABORT_TIME_MS 1000

#define CONTROLLER_KP 4
#define CONTROLLER_KI 1
#define CONTROLLER_IMAX 20

#define CONTROLLER_THROTTLE_MAX 60

typedef enum {
	CONTROLLER_STATE_OFF,
	CONTROLLER_STATE_IDLE,
	CONTROLLER_STATE_WAKE,
	CONTROLLER_STATE_READY,
	CONTROLLER_STATE_ACTIVE,
	CONTROLLER_STATE_ABORT,
	CONTROLLER_STATE_ERROR
} Controller_State;

typedef struct {
	Controller_State state;

} Controller;

float constrain(float val, float min, float max);

#endif /* INC_CONTROLLER_H_ */
