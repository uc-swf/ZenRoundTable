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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ffconf.h"
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
void userMessage(const char *message, ...);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_ERROR_Pin GPIO_PIN_13
#define LED_ERROR_GPIO_Port GPIOC
#define _2IN_A_Pin GPIO_PIN_0
#define _2IN_A_GPIO_Port GPIOC
#define _2IN_B_Pin GPIO_PIN_1
#define _2IN_B_GPIO_Port GPIOC
#define Ref_S_1_Pin GPIO_PIN_2
#define Ref_S_1_GPIO_Port GPIOC
#define Ref_S_2_Pin GPIO_PIN_3
#define Ref_S_2_GPIO_Port GPIOC
#define Coder_A_2_Pin GPIO_PIN_0
#define Coder_A_2_GPIO_Port GPIOA
#define Coder_B_2_Pin GPIO_PIN_1
#define Coder_B_2_GPIO_Port GPIOA
#define Coder_A_1_Pin GPIO_PIN_6
#define Coder_A_1_GPIO_Port GPIOA
#define Coder_B_1_Pin GPIO_PIN_7
#define Coder_B_1_GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_4
#define SD_CS_GPIO_Port GPIOC
#define M1_CURRENT_Pin GPIO_PIN_0
#define M1_CURRENT_GPIO_Port GPIOB
#define M2_CURRENT_Pin GPIO_PIN_1
#define M2_CURRENT_GPIO_Port GPIOB
#define TEST_OUTPUT_Pin GPIO_PIN_7
#define TEST_OUTPUT_GPIO_Port GPIOC
#define _1IN_B_Pin GPIO_PIN_8
#define _1IN_B_GPIO_Port GPIOC
#define _1IN_A_Pin GPIO_PIN_9
#define _1IN_A_GPIO_Port GPIOC
#define LED_STATE_Pin GPIO_PIN_12
#define LED_STATE_GPIO_Port GPIOC
#define BT_PWR_Pin GPIO_PIN_3
#define BT_PWR_GPIO_Port GPIOB
#define BT_KEY_Pin GPIO_PIN_4
#define BT_KEY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define SD_SPI_HANDLE hspi2			// SPI-Driver by kiwih
// Debugging prints, can be s disabled by config.bits
#define debugPrint(level, ...) ((level) ? userMessage(__VA_ARGS__) : (void)0)
#define DEBUG_MAIN 0	// config-bit for Debug-Messages from the main- Function
#define DEBUG_THR 0		// config-bit for Debug-Messages from processing THR-Files
#define DEBUG_ROBOT 0	// config-bit for Debug-Messages from SCARA-Robot and Motors
#define DEBUG_PID 0		// config-bit for Debug-Messages from the PID-Controller
#define DEBUG_MANUAL 1	// config-bit for Debug-Messages from Manual-Mode
#define DEBUG_COM 0		// config-bit for Debug-Messages from Communication
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
