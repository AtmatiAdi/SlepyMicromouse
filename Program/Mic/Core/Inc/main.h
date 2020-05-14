/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTOR_PRESCALLER 72-1
#define MOTOR_COUNTER 1024-1
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define BAT_3V7_Pin GPIO_PIN_3
#define BAT_3V7_GPIO_Port GPIOA
#define BAT_7V6_Pin GPIO_PIN_4
#define BAT_7V6_GPIO_Port GPIOA
#define CSN_Pin GPIO_PIN_0
#define CSN_GPIO_Port GPIOB
#define CE_Pin GPIO_PIN_1
#define CE_GPIO_Port GPIOB
#define MOTOR_1A_Pin GPIO_PIN_6
#define MOTOR_1A_GPIO_Port GPIOB
#define MOTOR_1B_Pin GPIO_PIN_7
#define MOTOR_1B_GPIO_Port GPIOB
#define MOTOR_2A_Pin GPIO_PIN_8
#define MOTOR_2A_GPIO_Port GPIOB
#define MOTOR_2B_Pin GPIO_PIN_9
#define MOTOR_2B_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
