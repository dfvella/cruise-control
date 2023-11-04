/*
 * throttle_map.c
 *
 *  Created on: Nov 3, 2023
 *      Author: legob
 */

#include "throttle_map.h"

float throttle_map(uint16_t aps_a, uint16_t aps_b)
{
	(void) aps_b;
	return aps_a / 40.95f;
}

void throttle_map_inv(float throttle, uint8_t *aps_a_out, uint8_t *aps_b_out)
{
	*aps_a_out = (uint8_t) throttle * 2.55f;
	*aps_b_out = (uint8_t) throttle * 2.55f;
}
