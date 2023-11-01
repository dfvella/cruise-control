#include "eeprom.h"

extern I2C_HandleTypeDef hi2c1;

HAL_StatusTypeDef eeprom_init()
{
    HAL_GPIO_WritePin(EEPROM_NWC_GPIO_Port, EEPROM_NWC_Pin, GPIO_PIN_SET);

    return HAL_OK;
}

HAL_StatusTypeDef eeprom_read(uint8_t addr, uint8_t *data, uint8_t n)
{
    if (data == NULL)
    {
        return HAL_ERROR;
    }
    if (n > EEPROM_MAX_TRANSFER)
    {
        return HAL_ERROR;
    }
    if (addr % EEPROM_MAX_TRANSFER)
    {
        return HAL_ERROR;
    }

    HAL_GPIO_WritePin(EEPROM_NWC_GPIO_Port, EEPROM_NWC_Pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef res;

    res = HAL_I2C_Master_Transmit(&hi2c1, EEPROM_I2C_ADDR << 1, &addr, 1, 100);

    if (res != HAL_OK)
    {
        HAL_GPIO_WritePin(EEPROM_NWC_GPIO_Port, EEPROM_NWC_Pin, GPIO_PIN_SET);
        return res;
    }

    res = HAL_I2C_Master_Receive(&hi2c1, EEPROM_I2C_ADDR << 1, data, n, 100);

    HAL_GPIO_WritePin(EEPROM_NWC_GPIO_Port, EEPROM_NWC_Pin, GPIO_PIN_SET);

    return res;
}

HAL_StatusTypeDef eeprom_write(uint8_t addr, const uint8_t *data, uint8_t n)
{
    if (data == NULL)
    {
        return HAL_ERROR;
    }
    if (n > EEPROM_MAX_TRANSFER)
    {
        return HAL_ERROR;
    }
    if (addr % EEPROM_MAX_TRANSFER)
    {
        return HAL_ERROR;
    }

    HAL_GPIO_WritePin(EEPROM_NWC_GPIO_Port, EEPROM_NWC_Pin, GPIO_PIN_RESET);

    uint8_t *payload = (uint8_t *) malloc(n + 1);

    payload[0] = addr;
    memcpy(&payload[1], data, n);

    HAL_StatusTypeDef res = HAL_I2C_Master_Transmit(&hi2c1, EEPROM_I2C_ADDR << 1, payload, n + 1, 100);

    free(payload);

    HAL_GPIO_WritePin(EEPROM_NWC_GPIO_Port, EEPROM_NWC_Pin, GPIO_PIN_SET);

    return res;
}
