/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define PT100_Tank_Pin GPIO_PIN_0
#define PT100_Tank_GPIO_Port GPIOC
#define PT100_Water_Pin GPIO_PIN_1
#define PT100_Water_GPIO_Port GPIOC
#define pH_Sensor_Pin GPIO_PIN_2
#define pH_Sensor_GPIO_Port GPIOC
#define HPT_Pin GPIO_PIN_3
#define HPT_GPIO_Port GPIOC
#define AV6_Pin GPIO_PIN_0
#define AV6_GPIO_Port GPIOB
#define Pump_P5_Pin GPIO_PIN_1
#define Pump_P5_GPIO_Port GPIOB
#define Pump_P6_Pin GPIO_PIN_2
#define Pump_P6_GPIO_Port GPIOB
#define AV7_Pin GPIO_PIN_12
#define AV7_GPIO_Port GPIOB
#define Resistor_Pin GPIO_PIN_13
#define Resistor_GPIO_Port GPIOB
#define potPresence_Pin GPIO_PIN_14
#define potPresence_GPIO_Port GPIOB
#define potMin_Pin GPIO_PIN_15
#define potMin_GPIO_Port GPIOB
#define waterMax_Pin GPIO_PIN_8
#define waterMax_GPIO_Port GPIOD
#define waterMin_Pin GPIO_PIN_9
#define waterMin_GPIO_Port GPIOD
#define processWaterMax_Pin GPIO_PIN_10
#define processWaterMax_GPIO_Port GPIOD
#define processWaterMin_Pin GPIO_PIN_11
#define processWaterMin_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
