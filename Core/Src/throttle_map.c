/*
 * throttle_map.c
 *
 *  Created on: Nov 3, 2023
 *      Author: legob
 */

#include "throttle_map.h"

static float interpolate(float x, float x1, float y1, float x2, float y2)
{
	float m = (y2 - y1) / (x2 - x1);
	return (m * (x - x1)) + y1;
}

static float linear_map(float x, float x1, float x2, float y1, float y2)
{
	float p = (x - x1) / x2;
	return (p * (y2 - y1)) + y1;
}

float throttle_map(uint16_t aps_a, uint16_t aps_b, uint8_t *aps_agreement)
{
	(void) aps_b;
	*aps_agreement = 1;
	return aps_a / 40.95f;
}

void throttle_map_inv(float throttle, uint8_t *aps_a_out, uint8_t *aps_b_out)
{
	*aps_a_out = (uint8_t) throttle * 2.55f;
	*aps_b_out = (uint8_t) throttle * 2.55f;
}
