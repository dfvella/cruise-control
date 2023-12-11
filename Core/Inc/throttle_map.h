/*
 * throttle_map.h
 *
 *  Created on: Nov 3, 2023
 *      Author: legob
 */

#ifndef INC_THROTTLE_MAP_H_
#define INC_THROTTLE_MAP_H_

#include <stdint.h>

#define APS_A_MIN 1.0
#define APS_A_MAX 4.0
#define APS_B_MIN 0.5
#define APS_B_MAX 2.0
#define APS_RATIO_MIN 1.9
#define APS_RATIO_MAX 2.1
#define APS_PLAUS_MIN 0.4
#define APS_PLAUS_MAX 4.1

float throttle_map(float aps_a, float aps_b, uint8_t *aps_agreement);
void throttle_map_inv(float throttle, float *aps_a_out, float *aps_b_out);

#endif /* INC_THROTTLE_MAP_H_ */
