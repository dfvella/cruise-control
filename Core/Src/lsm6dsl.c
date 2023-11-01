/**
 ******************************************************************************
 * @file    lsm6dsl.c
 * @author  MCD Application Team
 * @brief   This file provides a set of functions needed to manage the LSM6DSL
 *          accelero and gyro devices
 ******************************************************************************
 * @attention
 *
 * <<h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "lsm6dsl.h"

extern I2C_HandleTypeDef hi2c1;

static void SENSOR_IO_Init(void)
{

}

static void SENSOR_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
    UNUSED(Addr);

    HAL_I2C_Mem_Write(&hi2c1, Addr, Reg, 1, &Value, 1, 5);
}

static uint8_t  SENSOR_IO_Read(uint8_t Addr, uint8_t Reg)
{
    UNUSED(Addr);

    uint8_t Value;

    HAL_I2C_Mem_Read(&hi2c1, Addr, Reg, 1, &Value, 1, 5);

    return Value;
}

static uint16_t SENSOR_IO_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
    UNUSED(Addr);

    HAL_I2C_Mem_Read(&hi2c1, Addr, Reg, 1, Buffer, Length, 5);

    return 0;
}

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Component
  * @{
  */

/** @defgroup LSM6DSL LSM6DSL
  * @{
  */

/** @defgroup LSM6DSL_ACC_Private_Functions LSM6DSL ACC Private Functions
  * @{
  */
/**
  * @brief  Set LSM6DSL Accelerometer Initialization.
  * @param  InitStruct: Init parameters
  */
void LSM6DSL_AccInit(uint16_t InitStruct)
{  
  uint8_t ctrl = 0x00;
  uint8_t tmp;

  /* Read CTRL1_XL */
  tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL1_XL);

  /* Write value to ACC MEMS CTRL1_XL register: FS and Data Rate */
  ctrl = (uint8_t) InitStruct;
  tmp &= ~(0xFC);
  tmp |= ctrl;
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL1_XL, tmp);

  /* Read CTRL3_C */
  tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL3_C);

  /* Write value to ACC MEMS CTRL3_C register: BDU and Auto-increment */
  ctrl = ((uint8_t) (InitStruct >> 8));
  tmp &= ~(0x44);
  tmp |= ctrl; 
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL3_C, tmp);
}

/**
  * @brief  LSM6DSL Accelerometer De-initialization.
  */
void LSM6DSL_AccDeInit(void)
{
  uint8_t ctrl = 0x00;
  
  /* Read control register 1 value */
  ctrl = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL1_XL);

  /* Clear ODR bits */
  ctrl &= ~(LSM6DSL_ODR_BITPOSITION);

  /* Set Power down */
  ctrl |= LSM6DSL_ODR_POWER_DOWN;
  
  /* write back control register */
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL1_XL, ctrl);
}

/**
  * @brief  Read LSM6DSL ID.
  * @retval ID 
  */
uint8_t LSM6DSL_AccReadID(void)
{  
  /* IO interface initialization */
  SENSOR_IO_Init();
  /* Read value at Who am I register address */
  return (SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_WHO_AM_I_REG));
}

/**
  * @brief  Set/Unset Accelerometer in low power mode.
  * @param  status 0 means disable Low Power Mode, otherwise Low Power Mode is enabled
  */
void LSM6DSL_AccLowPower(uint16_t status)
{
  uint8_t ctrl = 0x00;
  
  /* Read CTRL6_C value */
  ctrl = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL6_C);

  /* Clear Low Power Mode bit */
  ctrl &= ~(0x10);

  /* Set Low Power Mode */
  if(status)
  {
    ctrl |= LSM6DSL_ACC_GYRO_LP_XL_ENABLED;
  }else
  {
    ctrl |= LSM6DSL_ACC_GYRO_LP_XL_DISABLED;
  }
  
  /* write back control register */
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL6_C, ctrl);
}

/**
  * @brief  Read X, Y & Z Acceleration values 
  * @param  pData: Data out pointer
  */
void LSM6DSL_AccReadXYZ(int16_t* pData)
{
  uint8_t buffer[6];
  uint8_t i = 0;
  
  /* Read output register X, Y & Z acceleration */
  SENSOR_IO_ReadMultiple(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_OUTX_L_XL, buffer, 6);
  
  for(i=0; i<3; i++)
  {
      pData[i]=((((uint16_t)buffer[2*i+1]) << 8) + (uint16_t)buffer[2*i]);
  }
}

/**
  * @}
  */ 

/** @defgroup LSM6DSL_GYRO_Private_Functions LSM6DSL GYRO Private Functions
  * @{
  */

/**
  * @brief  Set LSM6DSL Gyroscope Initialization.
  * @param  InitStruct: pointer to a LSM6DSL_InitTypeDef structure 
  *         that contains the configuration setting for the LSM6DSL.
  */
void LSM6DSL_GyroInit(uint16_t InitStruct)
{  
  uint8_t ctrl = 0x00;
  uint8_t tmp;

  /* Read CTRL2_G */
  tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL2_G);

  /* Write value to GYRO MEMS CTRL2_G register: FS and Data Rate */
  ctrl = (uint8_t) InitStruct;
  tmp &= ~(0xFC);
  tmp |= ctrl;
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL2_G, tmp);

  /* Read CTRL3_C */
  tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL3_C);

  /* Write value to GYRO MEMS CTRL3_C register: BDU and Auto-increment */
  ctrl = ((uint8_t) (InitStruct >> 8));
  tmp &= ~(0x44);
  tmp |= ctrl; 
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL3_C, tmp);
}


/**
  * @brief LSM6DSL Gyroscope De-initialization
  */
void LSM6DSL_GyroDeInit(void)
{
  uint8_t ctrl = 0x00;
  
  /* Read control register 1 value */
  ctrl = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL2_G);

  /* Clear ODR bits */
  ctrl &= ~(LSM6DSL_ODR_BITPOSITION);

  /* Set Power down */
  ctrl |= LSM6DSL_ODR_POWER_DOWN;
  
  /* write back control register */
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL2_G, ctrl);
}

/**
  * @brief  Read ID address of LSM6DSL
  * @retval ID 
  */
uint8_t LSM6DSL_GyroReadID(void)
{
  /* IO interface initialization */
  SENSOR_IO_Init();  
  /* Read value at Who am I register address */
  return SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_WHO_AM_I_REG);
}

/**
  * @brief Set/Unset LSM6DSL Gyroscope in low power mode
  * @param  status 0 means disable Low Power Mode, otherwise Low Power Mode is enabled 
  */
void LSM6DSL_GyroLowPower(uint16_t status)
{  
  uint8_t ctrl = 0x00;
  
  /* Read CTRL7_G value */
  ctrl = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL7_G);

  /* Clear Low Power Mode bit */
  ctrl &= ~(0x80);

  /* Set Low Power Mode */
  if(status)
  {
    ctrl |= LSM6DSL_ACC_GYRO_LP_G_ENABLED;
  }else
  {
    ctrl |= LSM6DSL_ACC_GYRO_LP_G_DISABLED;
  }
  
  /* write back control register */
  SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL7_G, ctrl);
}

/**
* @brief  Calculate the LSM6DSL angular data.
* @param  pfData: Data out pointer
*/
void LSM6DSL_GyroReadXYZAngRate(int16_t *pfData)
{
  uint8_t buffer[6];
  uint8_t i = 0;
  
  /* Read output register X, Y & Z acceleration */
  SENSOR_IO_ReadMultiple(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_OUTX_L_G, buffer, 6);
  
  for(i=0; i<3; i++)
  {
      pfData[i]=((((uint16_t)buffer[2*i+1]) << 8) + (uint16_t)buffer[2*i]);
  }
}

/**
  * @}
  */ 

/**
  * @}
  */ 
  
/**
  * @}
  */ 
  
/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

