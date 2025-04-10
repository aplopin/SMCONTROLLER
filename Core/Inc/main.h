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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ZERO_POS3_Pin GPIO_PIN_2
#define ZERO_POS3_GPIO_Port GPIOE
#define ZERO_POS2_Pin GPIO_PIN_3
#define ZERO_POS2_GPIO_Port GPIOE
#define EXTI_4_LS3_P_Pin GPIO_PIN_4
#define EXTI_4_LS3_P_GPIO_Port GPIOE
#define EXTI_5_LS3_N_Pin GPIO_PIN_5
#define EXTI_5_LS3_N_GPIO_Port GPIOE
#define ZERO_POS1_Pin GPIO_PIN_6
#define ZERO_POS1_GPIO_Port GPIOE
#define ACK_LINE0_Pin GPIO_PIN_13
#define ACK_LINE0_GPIO_Port GPIOC
#define ACK_LINE1_Pin GPIO_PIN_14
#define ACK_LINE1_GPIO_Port GPIOC
#define ACK_LINE2_Pin GPIO_PIN_15
#define ACK_LINE2_GPIO_Port GPIOC
#define EXTI_0_LS1_P_Pin GPIO_PIN_0
#define EXTI_0_LS1_P_GPIO_Port GPIOC
#define EXTI_2_LS2_P_Pin GPIO_PIN_2
#define EXTI_2_LS2_P_GPIO_Port GPIOC
#define EXTI_3_LS2_N_Pin GPIO_PIN_3
#define EXTI_3_LS2_N_GPIO_Port GPIOC
#define GATE_LASER_TIM13_CH1_Pin GPIO_PIN_6
#define GATE_LASER_TIM13_CH1_GPIO_Port GPIOA
#define STEP7_Pin GPIO_PIN_0
#define STEP7_GPIO_Port GPIOB
#define STEP8_Pin GPIO_PIN_1
#define STEP8_GPIO_Port GPIOB
#define DIR1_Pin GPIO_PIN_7
#define DIR1_GPIO_Port GPIOE
#define DIR2_Pin GPIO_PIN_8
#define DIR2_GPIO_Port GPIOE
#define STEP1_Pin GPIO_PIN_9
#define STEP1_GPIO_Port GPIOE
#define DIR3_Pin GPIO_PIN_10
#define DIR3_GPIO_Port GPIOE
#define STEP2_Pin GPIO_PIN_11
#define STEP2_GPIO_Port GPIOE
#define DIR4_Pin GPIO_PIN_12
#define DIR4_GPIO_Port GPIOE
#define STEP3_Pin GPIO_PIN_13
#define STEP3_GPIO_Port GPIOE
#define STEP4_Pin GPIO_PIN_14
#define STEP4_GPIO_Port GPIOE
#define DIR5_Pin GPIO_PIN_15
#define DIR5_GPIO_Port GPIOE
#define DIR6_Pin GPIO_PIN_10
#define DIR6_GPIO_Port GPIOB
#define DIR7_Pin GPIO_PIN_14
#define DIR7_GPIO_Port GPIOB
#define DIR8_Pin GPIO_PIN_15
#define DIR8_GPIO_Port GPIOB
#define EXTI_10_LS6_P_Pin GPIO_PIN_10
#define EXTI_10_LS6_P_GPIO_Port GPIOD
#define EXTI_11_LS6_N_Pin GPIO_PIN_11
#define EXTI_11_LS6_N_GPIO_Port GPIOD
#define EXTI_12_LS7_P_Pin GPIO_PIN_12
#define EXTI_12_LS7_P_GPIO_Port GPIOD
#define EXTI_13_LS7_N_Pin GPIO_PIN_13
#define EXTI_13_LS7_N_GPIO_Port GPIOD
#define EXTI_14_LS8_P_Pin GPIO_PIN_14
#define EXTI_14_LS8_P_GPIO_Port GPIOD
#define EXTI_15_LS8_N_Pin GPIO_PIN_15
#define EXTI_15_LS8_N_GPIO_Port GPIOD
#define EXTI_6_LS4_P_Pin GPIO_PIN_6
#define EXTI_6_LS4_P_GPIO_Port GPIOC
#define STEP6_Pin GPIO_PIN_7
#define STEP6_GPIO_Port GPIOC
#define EXTI_8_LS5_P_Pin GPIO_PIN_8
#define EXTI_8_LS5_P_GPIO_Port GPIOC
#define EXTI_9_LS5_N_Pin GPIO_PIN_9
#define EXTI_9_LS5_N_GPIO_Port GPIOC
#define LED_STATE_Pin GPIO_PIN_8
#define LED_STATE_GPIO_Port GPIOA
#define SPI3_NSS2_Pin GPIO_PIN_15
#define SPI3_NSS2_GPIO_Port GPIOA
#define STARTUP_OPTION1_Pin GPIO_PIN_2
#define STARTUP_OPTION1_GPIO_Port GPIOD
#define READY_Pin GPIO_PIN_3
#define READY_GPIO_Port GPIOD
#define ZERO_POS8_Pin GPIO_PIN_4
#define ZERO_POS8_GPIO_Port GPIOD
#define ZERO_POS7_Pin GPIO_PIN_5
#define ZERO_POS7_GPIO_Port GPIOD
#define ZERO_POS6_Pin GPIO_PIN_6
#define ZERO_POS6_GPIO_Port GPIOD
#define EXTI_7_LS4_N_Pin GPIO_PIN_7
#define EXTI_7_LS4_N_GPIO_Port GPIOD
#define STEP5_Pin GPIO_PIN_4
#define STEP5_GPIO_Port GPIOB
#define ZERO_POS5_Pin GPIO_PIN_5
#define ZERO_POS5_GPIO_Port GPIOB
#define SPI3_NSS1_Pin GPIO_PIN_8
#define SPI3_NSS1_GPIO_Port GPIOB
#define SPI3_NSS0_Pin GPIO_PIN_9
#define SPI3_NSS0_GPIO_Port GPIOB
#define ZERO_POS4_Pin GPIO_PIN_0
#define ZERO_POS4_GPIO_Port GPIOE
#define EXTI_1_LS1_N_Pin GPIO_PIN_1
#define EXTI_1_LS1_N_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
