/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_G_Pin GPIO_PIN_0
#define LED_G_GPIO_Port GPIOC
#define BUTTON_G_Pin GPIO_PIN_1
#define BUTTON_G_GPIO_Port GPIOC
#define LED_Y_Pin GPIO_PIN_2
#define LED_Y_GPIO_Port GPIOC
#define BUTTON_Y_Pin GPIO_PIN_3
#define BUTTON_Y_GPIO_Port GPIOC
#define LED_R_Pin GPIO_PIN_0
#define LED_R_GPIO_Port GPIOA
#define BUTTON_R_Pin GPIO_PIN_1
#define BUTTON_R_GPIO_Port GPIOA
#define EEPROM_NWC_Pin GPIO_PIN_4
#define EEPROM_NWC_GPIO_Port GPIOA
#define APS_B_IN_Pin GPIO_PIN_5
#define APS_B_IN_GPIO_Port GPIOA
#define APS_A_IN_Pin GPIO_PIN_6
#define APS_A_IN_GPIO_Port GPIOA
#define DISP_1B_Pin GPIO_PIN_5
#define DISP_1B_GPIO_Port GPIOC
#define DISP_1A_Pin GPIO_PIN_0
#define DISP_1A_GPIO_Port GPIOB
#define DISP_1G_Pin GPIO_PIN_1
#define DISP_1G_GPIO_Port GPIOB
#define DISP_1F_Pin GPIO_PIN_2
#define DISP_1F_GPIO_Port GPIOB
#define DISP_2F_Pin GPIO_PIN_13
#define DISP_2F_GPIO_Port GPIOB
#define DISP_2A_Pin GPIO_PIN_14
#define DISP_2A_GPIO_Port GPIOB
#define DISP_2B_Pin GPIO_PIN_15
#define DISP_2B_GPIO_Port GPIOB
#define DISP_2C_Pin GPIO_PIN_6
#define DISP_2C_GPIO_Port GPIOC
#define DISP_2G_Pin GPIO_PIN_7
#define DISP_2G_GPIO_Port GPIOC
#define DISP_2D_Pin GPIO_PIN_8
#define DISP_2D_GPIO_Port GPIOC
#define DISP_2E_Pin GPIO_PIN_9
#define DISP_2E_GPIO_Port GPIOC
#define DISP_1E_Pin GPIO_PIN_8
#define DISP_1E_GPIO_Port GPIOA
#define DISP_1D_Pin GPIO_PIN_9
#define DISP_1D_GPIO_Port GPIOA
#define DISP_1C_Pin GPIO_PIN_10
#define DISP_1C_GPIO_Port GPIOA
#define E_STOP_Pin GPIO_PIN_10
#define E_STOP_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
