/*
 * controller.c
 *
 *  Created on: Nov 3, 2023
 *      Author: legob
 */

#include "controller.h"
#include "obd2.h"

static Controller controller;

extern uint8_t can_data[8];
extern uint8_t can_len;

static const float accel_filter_resp[ACCEL_FILTER_LEN] = {
    0.25, 0.25, 0.25, 0.25, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0
};

static float constrain(float val, float min, float max)
{
	if (val > max)
	{
		return max;
	}

	if (val < min)
	{
		return min;
	}

	return val;
}

static float calculate_accel(void)
{
	int16_t accel_data[3];
	float accel_long;

	LSM6DSL_AccReadXYZ(accel_data);

//	printf("%ld, %f, %d, %d, %d\r\n", HAL_GetTick(), obd2_get_speed(), accel_data[0], accel_data[1], accel_data[2]);

	accel_long = (8.0f * accel_data[2]) / INT16_MAX;

	fir_filter_32f_update(&controller.accel_filter, accel_long);

	return fir_filter_32f_calculate(&controller.accel_filter);
}

static float calculate_set_speed(float current_speed, uint8_t button_y, uint8_t button_g)
{
	float set_speed;

	set_speed = controller.set_speed;

	if (controller.state == CONTROLLER_STATE_ACTIVE)
	{
		if (button_g)
		{
			set_speed++;
		}
		if (button_y)
		{
			set_speed--;
		}
	}
	else
	{
		set_speed = current_speed;
	}

	return set_speed;
}

static float calculate_throttle_out(float throttle_in, float current_speed, float *error_integral)
{
	float throttle_out;
	float error;

	error = controller.set_speed - current_speed;

	if (controller.state == CONTROLLER_STATE_ACTIVE)
	{
		*error_integral += error * (CONTROLLER_LOOP_PERIOD_MS / 1000.0f);
		*error_integral = constrain(*error_integral, 0, CONTROLLER_IMAX);
		throttle_out = (error * CONTROLLER_KP) + (*error_integral * CONTROLLER_KI);
		throttle_out = constrain(throttle_out, 0, CONTROLLER_THROTTLE_MAX);
	}
	else
	{
		*error_integral = 0;
		throttle_out = throttle_in;
	}

	return throttle_out;
}

static Controller_State calculate_next_state(uint8_t abort, uint8_t ready, uint8_t e_stop, uint8_t button_r, uint8_t button_y, uint8_t button_g, Controller_Error error)
{
	Controller_State next_state;
	uint32_t last_state_transition;
	uint8_t exit_wake;
	uint8_t exit_abort;
	uint8_t exit_error;

	next_state = controller.state;

	last_state_transition = HAL_GetTick() - controller.last_state_transition;

	exit_wake = last_state_transition > CONTROLLER_WAKE_TIME_MS;
	exit_abort = last_state_transition > CONTROLLER_ABORT_TIME_MS;
	exit_error = last_state_transition > CONTROLLER_ERROR_TIME_MS;

	if (e_stop)
	{
		next_state = CONTROLLER_STATE_OFF;
	}
	else
	{
		switch (controller.state)
		{
		case CONTROLLER_STATE_OFF:
			if (!e_stop)
			{
				next_state = CONTROLLER_STATE_IDLE;
			}
			break;

		case CONTROLLER_STATE_IDLE:
			if (button_r || button_y || button_g)
			{
				next_state = CONTROLLER_STATE_WAKE;
			}
			break;

		case CONTROLLER_STATE_WAKE:
			if (button_r)
			{
				next_state = CONTROLLER_STATE_IDLE;
			}
			else if (exit_wake)
			{
				next_state = CONTROLLER_STATE_WAIT;
			}
			break;

		case CONTROLLER_STATE_WAIT:
			if (button_r)
			{
				next_state = CONTROLLER_STATE_IDLE;
			}
			else if (ready)
			{
				next_state = CONTROLLER_STATE_READY;
			}
			break;

		case CONTROLLER_STATE_READY:
			if (button_r)
			{
				next_state = CONTROLLER_STATE_IDLE;
			}
			else if (!ready)
			{
				next_state = CONTROLLER_STATE_WAIT;
			}
			else if (button_g)
			{
				next_state = CONTROLLER_STATE_ACTIVE;
			}
			break;

		case CONTROLLER_STATE_ACTIVE:
			if (error)
			{
				next_state = CONTROLLER_STATE_ERROR;
			}
			else if (button_r)
			{
				next_state = CONTROLLER_STATE_WAIT;
			}
			else if (abort)
			{
				next_state = CONTROLLER_STATE_ABORT;
			}
			break;

		case CONTROLLER_STATE_ABORT:
			if (exit_abort)
			{
				next_state = CONTROLLER_STATE_WAIT;
			}
			break;

		case CONTROLLER_STATE_ERROR:
			if (exit_error && button_r)
			{
				next_state = CONTROLLER_STATE_IDLE;
			}
			break;

		default:
			break;
		}
	}

	return next_state;
}

static void update_ui(float current_speed)
{
	uint32_t last_state_transition = HAL_GetTick() - controller.last_state_transition;

	switch (controller.state)
	{
	case CONTROLLER_STATE_OFF:
	case CONTROLLER_STATE_IDLE:
		Display_off();
		led_r_set(LED_OFF);
		led_y_set(LED_OFF);
		led_g_set(LED_OFF);
		button_r_set_mode(BUTTON_SINGLE_PRESS);
		button_y_set_mode(BUTTON_SINGLE_PRESS);
		button_g_set_mode(BUTTON_SINGLE_PRESS);
		break;

	case CONTROLLER_STATE_WAKE:
		Display_spin();
		led_r_set(LED_ON);
		led_y_set(LED_OFF);
		led_g_set(LED_OFF);
		if (last_state_transition > CONTROLLER_WAKE_TIME_MS / 3) led_y_set(LED_ON);
		if (last_state_transition > (2*CONTROLLER_WAKE_TIME_MS) / 3) led_g_set(LED_ON);
		button_r_set_mode(BUTTON_SINGLE_PRESS);
		button_y_set_mode(BUTTON_SINGLE_PRESS);
		button_g_set_mode(BUTTON_SINGLE_PRESS);
		break;

	case CONTROLLER_STATE_WAIT:
		Display_set(current_speed);
		led_r_set(LED_ON);
		led_y_set(LED_OFF);
		led_g_set(LED_OFF);
		button_r_set_mode(BUTTON_SINGLE_PRESS);
		button_y_set_mode(BUTTON_SINGLE_PRESS);
		button_g_set_mode(BUTTON_SINGLE_PRESS);
		break;

	case CONTROLLER_STATE_READY:
		Display_set(current_speed);
		led_r_set(LED_ON);
		led_y_set(LED_OFF);
		led_g_set(LED_BLINK);
		button_r_set_mode(BUTTON_SINGLE_PRESS);
		button_y_set_mode(BUTTON_SINGLE_PRESS);
		button_g_set_mode(BUTTON_SINGLE_PRESS);
		break;

	case CONTROLLER_STATE_ACTIVE:
		Display_set(controller.set_speed);
		led_r_set(LED_ON);
		led_y_set(LED_ON);
		led_g_set(LED_ON);
		button_r_set_mode(BUTTON_SINGLE_PRESS);
		button_y_set_mode(BUTTON_MULTI_PRESS);
		button_g_set_mode(BUTTON_MULTI_PRESS);
		break;

	case CONTROLLER_STATE_ABORT:
		Display_set(current_speed);
		led_r_set(LED_STROBE);
		led_y_set(LED_STROBE);
		led_g_set(LED_STROBE);
		button_r_set_mode(BUTTON_SINGLE_PRESS);
		button_y_set_mode(BUTTON_SINGLE_PRESS);
		button_g_set_mode(BUTTON_SINGLE_PRESS);
		break;

	case CONTROLLER_STATE_ERROR:
		Display_off();
		led_r_set(LED_BLINK);
		led_y_set(LED_OFF);
		led_g_set(LED_OFF);
		button_r_set_mode(BUTTON_SINGLE_PRESS);
		button_y_set_mode(BUTTON_SINGLE_PRESS);
		button_g_set_mode(BUTTON_SINGLE_PRESS);
		break;

	default:
		break;
	}

	Display_draw();
	led_update();
}

void controller_init(void)
{
	Display_init();

	uint16_t initStruct;
	initStruct = 0;
	initStruct |= LSM6DSL_ODR_52Hz | LSM6DSL_ACC_FULLSCALE_8G;
	initStruct |= (LSM6DSL_ACC_GYRO_IF_INC_ENABLED | LSM6DSL_BDU_CONTINUOS) << 8;
	LSM6DSL_AccInit(initStruct);
	initStruct = 0;
	initStruct |= LSM6DSL_ODR_52Hz | LSM6DSL_GYRO_FS_500;
	initStruct |= (LSM6DSL_ACC_GYRO_IF_INC_ENABLED | LSM6DSL_BDU_CONTINUOS) << 8;
	LSM6DSL_GyroInit(initStruct);

	fir_filter_32f_init(&controller.accel_filter, accel_filter_resp, ACCEL_FILTER_LEN);
}

float controller_run(float current_speed, float throttle_in, Controller_Error error_flag)
{
	uint8_t e_stop;
	uint8_t button_r, button_y, button_g;

	uint8_t minimum_speed_cond;
	uint8_t accel_cond;

	uint8_t abort;
	uint8_t ready;

	float accel;
	float throttle_out;

	Controller_State next_state;

	e_stop = !HAL_GPIO_ReadPin(E_STOP_GPIO_Port, E_STOP_Pin);
	button_r = button_r_get();
	button_y = button_y_get();
	button_g = button_g_get();

	minimum_speed_cond = current_speed >= CONTROLLER_MINIMUM_SPEED;

	accel = calculate_accel();
	accel_cond = accel < CONTROLLER_ACCEL_THRESHOLD;

	abort = accel_cond || !minimum_speed_cond;
	ready = minimum_speed_cond;

	controller.set_speed = calculate_set_speed(current_speed, button_y, button_g);
	throttle_out = calculate_throttle_out(throttle_in, current_speed, &controller.error_integral);
	next_state = calculate_next_state(abort, ready, e_stop, button_r, button_y, button_g, error_flag);

	if (next_state != controller.state)
	{
		controller.state = next_state;
		controller.last_state_transition = HAL_GetTick();
	}

	update_ui(current_speed);

//	printf("state:%d  estop:%d  a:%f  len:%X  d0:%X  d1:%X  d2:%X  d3:%X\n", next_state, e_stop, accel, can_len, can_data[0], can_data[1], can_data[2], can_data[3]);

	return throttle_out;
}
