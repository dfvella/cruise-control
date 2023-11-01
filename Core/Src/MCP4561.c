/*
 * MCP4561.c
 *
 *  Created on: Oct 29, 2023
 *      Author: legob
 */

#include "MCP4561.h"

extern I2C_HandleTypeDef hi2c1;

static HAL_StatusTypeDef MCP4561_Set(uint16_t n, uint8_t wiper)
{
	uint8_t buf[2];

	uint8_t n_upper = (n >> 8) & 0x3;
	uint8_t n_lower = n & 0xFF;

	buf[0] = 0;
	buf[1] = 0;

	buf[0] |= wiper << MCP4561_ADDR_OFFSET;
	buf[0] |= MCP4561_CMD_WRITE << MCP4561_CMD_OFFSET;
	buf[0] |= n_upper;

	buf[1] |= n_lower;

	return HAL_I2C_Master_Transmit(&hi2c1, MCP4561_I2C_ADDR << 1, buf, 2, 5);
}

HAL_StatusTypeDef MCP4561_Set_A(uint16_t n)
{
	return MCP4561_Set(n, MCP4561_ADDR_WIPER_A);
}

HAL_StatusTypeDef MCP4561_Set_B(uint16_t n)
{
	return MCP4561_Set(n, MCP4561_ADDR_WIPER_B);
}
