/*
 * throttle_map.h
 *
 *  Created on: Nov 3, 2023
 *      Author: legob
 */

#ifndef INC_THROTTLE_MAP_H_
#define INC_THROTTLE_MAP_H_

#include <stdint.h>

float throttle_map(uint16_t aps_a, uint16_t aps_b);
void throttle_map_inv(float throttle, uint8_t *aps_a_out, uint8_t *aps_b_out);

#endif /* INC_THROTTLE_MAP_H_ */
