/*
 * controller.c
 *
 *  Created on: Nov 3, 2023
 *      Author: legob
 */

#include "controller.h"

float constrain(float val, float min, float max)
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
