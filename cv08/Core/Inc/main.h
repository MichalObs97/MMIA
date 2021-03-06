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
#define Col1_Pin GPIO_PIN_3
#define Col1_GPIO_Port GPIOE
#define Col4_Pin GPIO_PIN_4
#define Col4_GPIO_Port GPIOE
#define Col3_Pin GPIO_PIN_5
#define Col3_GPIO_Port GPIOE
#define Col2_Pin GPIO_PIN_6
#define Col2_GPIO_Port GPIOE
#define Row3_Pin GPIO_PIN_7
#define Row3_GPIO_Port GPIOF
#define Row4_Pin GPIO_PIN_8
#define Row4_GPIO_Port GPIOF
#define Row2_Pin GPIO_PIN_9
#define Row2_GPIO_Port GPIOF
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOB
#define Row1_Pin GPIO_PIN_1
#define Row1_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
