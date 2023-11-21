/*
 * throttle_map.h
 *
 *  Created on: Nov 3, 2023
 *      Author: legob
 */

#ifndef INC_THROTTLE_MAP_H_
#define INC_THROTTLE_MAP_H_

#include <stdint.h>

#define APS_A_MIN 730
#define APS_A_MAX 3300
#define APS_B_MIN 340
#define APS_B_MAX 1600
#define APS_RATIO_MIN 2.0
#define APS_RATIO_MAX 2.2
#define APS_PLAUS_MIN 200
#define APS_PLAUS_MAX 3800
#define APS_A_OUT_MIN 50
#define APS_A_OUT_MAX 209
#define APS_B_OUT_MIN 26
#define APS_B_OUT_MAX 105

float throttle_map(uint16_t aps_a, uint16_t aps_b, uint8_t *aps_agreement);
void throttle_map_inv(float throttle, uint8_t *aps_a_out, uint8_t *aps_b_out);

#endif /* INC_THROTTLE_MAP_H_ */
