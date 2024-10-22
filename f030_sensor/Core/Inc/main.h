/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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
#define KEY1_Pin GPIO_PIN_3
#define KEY1_GPIO_Port GPIOA
#define KEY2_Pin GPIO_PIN_4
#define KEY2_GPIO_Port GPIOA
#define KEY3_Pin GPIO_PIN_5
#define KEY3_GPIO_Port GPIOA
#define LED_R_Pin GPIO_PIN_11
#define LED_R_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_12
#define LED_G_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_13
#define LED_B_GPIO_Port GPIOB
#define BEEP2_Pin GPIO_PIN_14
#define BEEP2_GPIO_Port GPIOB
#define BEEP1_Pin GPIO_PIN_15
#define BEEP1_GPIO_Port GPIOB
#define RS485_Ctrl_Pin GPIO_PIN_8
#define RS485_Ctrl_GPIO_Port GPIOA
#define Relay_K2_Pin GPIO_PIN_4
#define Relay_K2_GPIO_Port GPIOB
#define Relay_K1_Pin GPIO_PIN_5
#define Relay_K1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
