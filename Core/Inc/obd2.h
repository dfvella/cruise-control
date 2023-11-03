/*
 * obd2.h
 *
 *  Created on: Nov 1, 2023
 *      Author: legob
 */

#ifndef INC_OBD2_H_
#define INC_OBD2_H_

#include "stm32g4xx_hal.h"

#define OBD2_REQUEST_TIMEOUT_MS 5
#define OBD2_RESPONSE_TIMEOUT_MS 500

#define OBD2_REQUEST_LENGTH 8
#define OBD2_REQUEST_ID 0x7DF
#define OBD2_REQUEST_DLEN 0x02
#define OBD2_REQUEST_MODE 0x01
#define OBD2_REQUEST_PID 0x0D

#define OBD2_RESPONSE_LENGTH 8
#define OBD2_RESPONSE_ID 0x7E8
#define OBD2_RESPONSE_DLEN 0x03
#define OBD2_RESPONSE_MODE 0x41
#define OBD2_RESPONSE_PID 0x0D

typedef struct {
	uint8_t dlen;
	uint8_t mode;
	uint8_t pid;
	uint8_t a;
	uint8_t b;
	uint8_t c;
	uint8_t d;
	uint8_t unused;
} OBD2_MSG;

HAL_StatusTypeDef obd2_init(FDCAN_HandleTypeDef *hcan);
void obd2_handle_rx(FDCAN_HandleTypeDef *hcan);
HAL_StatusTypeDef obd2_request_speed(FDCAN_HandleTypeDef *hcan);
float obd2_get_speed(void);
HAL_StatusTypeDef obd2_get_status(void);

#endif /* INC_OBD2_H_ */
