/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define PA5_SCK_Pin GPIO_PIN_5
#define PA5_SCK_GPIO_Port GPIOA
#define PA6_MISO_Pin GPIO_PIN_6
#define PA6_MISO_GPIO_Port GPIOA
#define PA7_MOSI_Pin GPIO_PIN_7
#define PA7_MOSI_GPIO_Port GPIOA
#define PB0_CH3_Pin GPIO_PIN_0
#define PB0_CH3_GPIO_Port GPIOB
#define PB1_CH4_Pin GPIO_PIN_1
#define PB1_CH4_GPIO_Port GPIOB
#define PB4_CH1_Pin GPIO_PIN_4
#define PB4_CH1_GPIO_Port GPIOB
#define PB5_CH2_Pin GPIO_PIN_5
#define PB5_CH2_GPIO_Port GPIOB
#define PB6_SCL_Pin GPIO_PIN_6
#define PB6_SCL_GPIO_Port GPIOB
#define PB7_SDA_Pin GPIO_PIN_7
#define PB7_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
