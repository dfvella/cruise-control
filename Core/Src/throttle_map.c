/*
 * throttle_map.c
 *
 *  Created on: Nov 3, 2023
 *      Author: legob
 */

#include "throttle_map.h"

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

static float interpolate(float x, float x1, float x2, float y1, float y2)
{
	float m = (y2 - y1) / (x2 - x1);
	return (m * (x - x1)) + y1;
}

float throttle_map(uint16_t aps_a, uint16_t aps_b, uint8_t *aps_agreement)
{
	uint8_t aps_a_plausible;
	uint8_t aps_b_plausible;
	uint8_t aps_ratio_plausible;

	float throttle_a;
	float throttle_b;
	float aps_ratio;

	float result;

	throttle_a = interpolate(aps_a, APS_A_MIN, APS_A_MAX, 0, 100);
	throttle_b = interpolate(aps_b, APS_B_MIN, APS_B_MAX, 0, 100);

	aps_ratio = (float)(aps_a) / (float)(aps_b);

	aps_a_plausible = (APS_PLAUS_MIN < aps_a) && (aps_a < APS_PLAUS_MAX);
	aps_b_plausible = (APS_PLAUS_MIN < aps_b) && (aps_b < APS_PLAUS_MAX);

	aps_ratio_plausible = (APS_RATIO_MIN < aps_ratio) && (aps_ratio < APS_RATIO_MAX);

	*aps_agreement = aps_a_plausible && aps_b_plausible && aps_ratio_plausible;

	if (*aps_agreement)
	{
		result = (throttle_a + throttle_b) / 2;
	}
	else
	{
		result = 0;
	}

	return result;
}

void throttle_map_inv(float throttle, uint8_t *aps_a_out, uint8_t *aps_b_out)
{
	float a_out;
	float b_out;

	a_out = interpolate(throttle, 0, 100, APS_A_OUT_MIN, APS_A_OUT_MAX);
	b_out = interpolate(throttle, 0, 100, APS_B_OUT_MIN, APS_B_OUT_MAX);

	*aps_a_out = constrain(a_out, APS_A_OUT_MIN, APS_A_OUT_MAX);
	*aps_b_out = constrain(b_out, APS_B_OUT_MIN, APS_B_OUT_MAX);
}
