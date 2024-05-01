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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define DIGIT_B3_Pin GPIO_PIN_1
#define DIGIT_B3_GPIO_Port GPIOC
#define RPM_Tick_Pin GPIO_PIN_3
#define RPM_Tick_GPIO_Port GPIOC
#define RPM_Tick_EXTI_IRQn EXTI3_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define BLUE_Pin GPIO_PIN_2
#define BLUE_GPIO_Port GPIOB
#define RED_Pin GPIO_PIN_9
#define RED_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define DIGIT_B0_Pin GPIO_PIN_10
#define DIGIT_B0_GPIO_Port GPIOC
#define DIGIT_B1_Pin GPIO_PIN_11
#define DIGIT_B1_GPIO_Port GPIOC
#define DIGIT_B2_Pin GPIO_PIN_12
#define DIGIT_B2_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define DIGIT_A0_Pin GPIO_PIN_5
#define DIGIT_A0_GPIO_Port GPIOB
#define GREEN_Pin GPIO_PIN_6
#define GREEN_GPIO_Port GPIOB
#define DIGIT_A1_Pin GPIO_PIN_7
#define DIGIT_A1_GPIO_Port GPIOB
#define DIGIT_A2_Pin GPIO_PIN_8
#define DIGIT_A2_GPIO_Port GPIOB
#define DIGIT_A3_Pin GPIO_PIN_9
#define DIGIT_A3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
