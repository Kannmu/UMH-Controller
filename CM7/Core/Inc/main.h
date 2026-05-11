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
#include "stm32h7xx_hal.h"

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
#define Transducer_Pin GPIO_PIN_6
#define Transducer_GPIO_Port GPIOI
#define TransducerI5_Pin GPIO_PIN_5
#define TransducerI5_GPIO_Port GPIOI
#define TransducerI4_Pin GPIO_PIN_4
#define TransducerI4_GPIO_Port GPIOI
#define TransducerB5_Pin GPIO_PIN_5
#define TransducerB5_GPIO_Port GPIOB
#define TransducerG9_Pin GPIO_PIN_9
#define TransducerG9_GPIO_Port GPIOG
#define TransducerD5_Pin GPIO_PIN_5
#define TransducerD5_GPIO_Port GPIOD
#define TransducerD4_Pin GPIO_PIN_4
#define TransducerD4_GPIO_Port GPIOD
#define TransducerC10_Pin GPIO_PIN_10
#define TransducerC10_GPIO_Port GPIOC
#define TransducerI1_Pin GPIO_PIN_1
#define TransducerI1_GPIO_Port GPIOI
#define TransducerI0_Pin GPIO_PIN_0
#define TransducerI0_GPIO_Port GPIOI
#define TransducerI7_Pin GPIO_PIN_7
#define TransducerI7_GPIO_Port GPIOI
#define TransducerE1_Pin GPIO_PIN_1
#define TransducerE1_GPIO_Port GPIOE
#define TransducerB6_Pin GPIO_PIN_6
#define TransducerB6_GPIO_Port GPIOB
#define TransducerG11_Pin GPIO_PIN_11
#define TransducerG11_GPIO_Port GPIOG
#define TransducerD6_Pin GPIO_PIN_6
#define TransducerD6_GPIO_Port GPIOD
#define TransducerD3_Pin GPIO_PIN_3
#define TransducerD3_GPIO_Port GPIOD
#define TransducerI2_Pin GPIO_PIN_2
#define TransducerI2_GPIO_Port GPIOI
#define TransducerE2_Pin GPIO_PIN_2
#define TransducerE2_GPIO_Port GPIOE
#define TransducerE0_Pin GPIO_PIN_0
#define TransducerE0_GPIO_Port GPIOE
#define TransducerB7_Pin GPIO_PIN_7
#define TransducerB7_GPIO_Port GPIOB
#define TransducerG12_Pin GPIO_PIN_12
#define TransducerG12_GPIO_Port GPIOG
#define TransducerD7_Pin GPIO_PIN_7
#define TransducerD7_GPIO_Port GPIOD
#define TransducerI3_Pin GPIO_PIN_3
#define TransducerI3_GPIO_Port GPIOI
#define TransducerE5_Pin GPIO_PIN_5
#define TransducerE5_GPIO_Port GPIOE
#define TransducerE4_Pin GPIO_PIN_4
#define TransducerE4_GPIO_Port GPIOE
#define TransducerE3_Pin GPIO_PIN_3
#define TransducerE3_GPIO_Port GPIOE
#define TransducerB9_Pin GPIO_PIN_9
#define TransducerB9_GPIO_Port GPIOB
#define TransducerB8_Pin GPIO_PIN_8
#define TransducerB8_GPIO_Port GPIOB
#define TransducerG15_Pin GPIO_PIN_15
#define TransducerG15_GPIO_Port GPIOG
#define TransducerG14_Pin GPIO_PIN_14
#define TransducerG14_GPIO_Port GPIOG
#define TransducerG13_Pin GPIO_PIN_13
#define TransducerG13_GPIO_Port GPIOG
#define TransducerJ14_Pin GPIO_PIN_14
#define TransducerJ14_GPIO_Port GPIOJ
#define TransducerJ12_Pin GPIO_PIN_12
#define TransducerJ12_GPIO_Port GPIOJ
#define TransducerD2_Pin GPIO_PIN_2
#define TransducerD2_GPIO_Port GPIOD
#define TransducerD0_Pin GPIO_PIN_0
#define TransducerD0_GPIO_Port GPIOD
#define TransducerI9_Pin GPIO_PIN_9
#define TransducerI9_GPIO_Port GPIOI
#define TransducerI8_Pin GPIO_PIN_8
#define TransducerI8_GPIO_Port GPIOI
#define TransducerE6_Pin GPIO_PIN_6
#define TransducerE6_GPIO_Port GPIOE
#define TransducerJ13_Pin GPIO_PIN_13
#define TransducerJ13_GPIO_Port GPIOJ
#define TransducerD1_Pin GPIO_PIN_1
#define TransducerD1_GPIO_Port GPIOD
#define TransducerC8_Pin GPIO_PIN_8
#define TransducerC8_GPIO_Port GPIOC
#define TransducerC9_Pin GPIO_PIN_9
#define TransducerC9_GPIO_Port GPIOC
#define TransducerI10_Pin GPIO_PIN_10
#define TransducerI10_GPIO_Port GPIOI
#define TransducerI11_Pin GPIO_PIN_11
#define TransducerI11_GPIO_Port GPIOI
#define TransducerC7_Pin GPIO_PIN_7
#define TransducerC7_GPIO_Port GPIOC
#define TransducerC6_Pin GPIO_PIN_6
#define TransducerC6_GPIO_Port GPIOC
#define TransducerG8_Pin GPIO_PIN_8
#define TransducerG8_GPIO_Port GPIOG
#define TransducerG7_Pin GPIO_PIN_7
#define TransducerG7_GPIO_Port GPIOG
#define TransducerF2_Pin GPIO_PIN_2
#define TransducerF2_GPIO_Port GPIOF
#define TransducerF1_Pin GPIO_PIN_1
#define TransducerF1_GPIO_Port GPIOF
#define TransducerF0_Pin GPIO_PIN_0
#define TransducerF0_GPIO_Port GPIOF
#define TransducerG5_Pin GPIO_PIN_5
#define TransducerG5_GPIO_Port GPIOG
#define TransducerG6_Pin GPIO_PIN_6
#define TransducerG6_GPIO_Port GPIOG
#define TransducerI12_Pin GPIO_PIN_12
#define TransducerI12_GPIO_Port GPIOI
#define TransducerI13_Pin GPIO_PIN_13
#define TransducerI13_GPIO_Port GPIOI
#define TransducerI14_Pin GPIO_PIN_14
#define TransducerI14_GPIO_Port GPIOI
#define TransducerF3_Pin GPIO_PIN_3
#define TransducerF3_GPIO_Port GPIOF
#define TransducerG4_Pin GPIO_PIN_4
#define TransducerG4_GPIO_Port GPIOG
#define TransducerG3_Pin GPIO_PIN_3
#define TransducerG3_GPIO_Port GPIOG
#define TransducerG2_Pin GPIO_PIN_2
#define TransducerG2_GPIO_Port GPIOG
#define TransducerF5_Pin GPIO_PIN_5
#define TransducerF5_GPIO_Port GPIOF
#define TransducerF4_Pin GPIO_PIN_4
#define TransducerF4_GPIO_Port GPIOF
#define TransducerF6_Pin GPIO_PIN_6
#define TransducerF6_GPIO_Port GPIOF
#define TransducerF7_Pin GPIO_PIN_7
#define TransducerF7_GPIO_Port GPIOF
#define TransducerF8_Pin GPIO_PIN_8
#define TransducerF8_GPIO_Port GPIOF
#define TransducerJ11_Pin GPIO_PIN_11
#define TransducerJ11_GPIO_Port GPIOJ
#define TransducerC0_Pin GPIO_PIN_0
#define TransducerC0_GPIO_Port GPIOC
#define TransducerF10_Pin GPIO_PIN_10
#define TransducerF10_GPIO_Port GPIOF
#define TransducerF9_Pin GPIO_PIN_9
#define TransducerF9_GPIO_Port GPIOF
#define TransducerJ10_Pin GPIO_PIN_10
#define TransducerJ10_GPIO_Port GPIOJ
#define TransducerC1_Pin GPIO_PIN_1
#define TransducerC1_GPIO_Port GPIOC
#define TransducerC2_Pin GPIO_PIN_2
#define TransducerC2_GPIO_Port GPIOC
#define TransducerJ9_Pin GPIO_PIN_9
#define TransducerJ9_GPIO_Port GPIOJ
#define TransducerJ0_Pin GPIO_PIN_0
#define TransducerJ0_GPIO_Port GPIOJ
#define TransducerJ8_Pin GPIO_PIN_8
#define TransducerJ8_GPIO_Port GPIOJ
#define TransducerJ7_Pin GPIO_PIN_7
#define TransducerJ7_GPIO_Port GPIOJ
#define TransducerJ6_Pin GPIO_PIN_6
#define TransducerJ6_GPIO_Port GPIOJ
#define TransducerI15_Pin GPIO_PIN_15
#define TransducerI15_GPIO_Port GPIOI
#define TransducerJ1_Pin GPIO_PIN_1
#define TransducerJ1_GPIO_Port GPIOJ
#define TransducerF13_Pin GPIO_PIN_13
#define TransducerF13_GPIO_Port GPIOF
#define TransducerF14_Pin GPIO_PIN_14
#define TransducerF14_GPIO_Port GPIOF
#define TransducerD15_Pin GPIO_PIN_15
#define TransducerD15_GPIO_Port GPIOD
#define TransducerD14_Pin GPIO_PIN_14
#define TransducerD14_GPIO_Port GPIOD
#define TransducerB2_Pin GPIO_PIN_2
#define TransducerB2_GPIO_Port GPIOB
#define TransducerF12_Pin GPIO_PIN_12
#define TransducerF12_GPIO_Port GPIOF
#define TransducerF15_Pin GPIO_PIN_15
#define TransducerF15_GPIO_Port GPIOF
#define TransducerJ5_Pin GPIO_PIN_5
#define TransducerJ5_GPIO_Port GPIOJ
#define TransducerD11_Pin GPIO_PIN_11
#define TransducerD11_GPIO_Port GPIOD
#define TransducerD12_Pin GPIO_PIN_12
#define TransducerD12_GPIO_Port GPIOD
#define TransducerD13_Pin GPIO_PIN_13
#define TransducerD13_GPIO_Port GPIOD
#define TransducerC4_Pin GPIO_PIN_4
#define TransducerC4_GPIO_Port GPIOC
#define TransducerB1_Pin GPIO_PIN_1
#define TransducerB1_GPIO_Port GPIOB
#define TransducerJ2_Pin GPIO_PIN_2
#define TransducerJ2_GPIO_Port GPIOJ
#define TransducerF11_Pin GPIO_PIN_11
#define TransducerF11_GPIO_Port GPIOF
#define TransducerB12_Pin GPIO_PIN_12
#define TransducerB12_GPIO_Port GPIOB
#define TransducerB15_Pin GPIO_PIN_15
#define TransducerB15_GPIO_Port GPIOB
#define TransducerD10_Pin GPIO_PIN_10
#define TransducerD10_GPIO_Port GPIOD
#define TransducerD9_Pin GPIO_PIN_9
#define TransducerD9_GPIO_Port GPIOD
#define TransducerC5_Pin GPIO_PIN_5
#define TransducerC5_GPIO_Port GPIOC
#define TransducerB0_Pin GPIO_PIN_0
#define TransducerB0_GPIO_Port GPIOB
#define TransducerJ3_Pin GPIO_PIN_3
#define TransducerJ3_GPIO_Port GPIOJ
#define TransducerJ4_Pin GPIO_PIN_4
#define TransducerJ4_GPIO_Port GPIOJ
#define TransducerE7_Pin GPIO_PIN_7
#define TransducerE7_GPIO_Port GPIOE
#define TransducerB13_Pin GPIO_PIN_13
#define TransducerB13_GPIO_Port GPIOB
#define TransducerB14_Pin GPIO_PIN_14
#define TransducerB14_GPIO_Port GPIOB
#define TransducerD8_Pin GPIO_PIN_8
#define TransducerD8_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
