/*
 * MCP4561.h
 *
 *  Created on: Oct 29, 2023
 *      Author: legob
 */

#include "stm32g4xx_hal.h"

#define MCP4561_I2C_ADDR 0x28

#define MCP4561_ADDR_OFFSET 4
#define MCP4561_ADDR_WIPER_A 0x00
#define MCP4561_ADDR_WIPER_B 0x01

#define MCP4561_CMD_OFFSET 2
#define MCP4561_CMD_WRITE 0b00
#define MCP4561_CMD_INCREMENT 0b01
#define MCP4561_CMD_DECREMENT 0b10
#define MCP4561_CMD_READ 0b11

HAL_StatusTypeDef MCP4561_Set_A(uint16_t n);
HAL_StatusTypeDef MCP4561_Set_B(uint16_t n);
