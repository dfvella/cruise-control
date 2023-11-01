#ifndef __EEPROM_H__
#define __EEPROM_H__

#include <malloc.h>
#include <memory.h>
#include <stdint.h>

#include "main.h"
#include "stm32g4xx_hal.h"

#define EEPROM_I2C_ADDR 0x50

#define EEPROM_MAX_TRANSFER 16

HAL_StatusTypeDef eeprom_init();
HAL_StatusTypeDef eeprom_read(uint8_t addr, uint8_t *data, uint8_t n);
HAL_StatusTypeDef eeprom_write(uint8_t addr, const uint8_t *data, uint8_t n);

#endif /* __EEPROM_H__ */
