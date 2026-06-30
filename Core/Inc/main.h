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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define _USE_MATH_DEFINES
// 标准库
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

extern DMA_HandleTypeDef hdma_memtomem_dma1_stream0;
extern DMA_HandleTypeDef hdma_memtomem_dma1_stream1;
extern DMA_HandleTypeDef hdma_memtomem_dma1_stream2;
extern DMA_HandleTypeDef hdma_memtomem_dma2_stream0;
extern DMA_HandleTypeDef hdma_memtomem_dma2_stream1;
extern I2C_HandleTypeDef hi2c3;
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
#define TRANSDUCER_Pin GPIO_PIN_2
#define TRANSDUCER_GPIO_Port GPIOE
#define TRANSDUCERE3_Pin GPIO_PIN_3
#define TRANSDUCERE3_GPIO_Port GPIOE
#define TRANSDUCERE4_Pin GPIO_PIN_4
#define TRANSDUCERE4_GPIO_Port GPIOE
#define TRANSDUCERE5_Pin GPIO_PIN_5
#define TRANSDUCERE5_GPIO_Port GPIOE
#define TRANSDUCERE6_Pin GPIO_PIN_6
#define TRANSDUCERE6_GPIO_Port GPIOE
#define VIRTUALTRANSDUCER_Pin GPIO_PIN_13
#define VIRTUALTRANSDUCER_GPIO_Port GPIOC
#define TRIGGER0_Pin GPIO_PIN_14
#define TRIGGER0_GPIO_Port GPIOC
#define TRIGGER1_Pin GPIO_PIN_15
#define TRIGGER1_GPIO_Port GPIOC
#define TRANSDUCERC0_Pin GPIO_PIN_0
#define TRANSDUCERC0_GPIO_Port GPIOC
#define TRANSDUCERC1_Pin GPIO_PIN_1
#define TRANSDUCERC1_GPIO_Port GPIOC
#define TRANSDUCERC2_Pin GPIO_PIN_2
#define TRANSDUCERC2_GPIO_Port GPIOC
#define TRANSDUCERC3_Pin GPIO_PIN_3
#define TRANSDUCERC3_GPIO_Port GPIOC
#define KEY0_Pin GPIO_PIN_3
#define KEY0_GPIO_Port GPIOA
#define KEY1_Pin GPIO_PIN_4
#define KEY1_GPIO_Port GPIOA
#define KEY2_Pin GPIO_PIN_5
#define KEY2_GPIO_Port GPIOA
#define KEY3_Pin GPIO_PIN_6
#define KEY3_GPIO_Port GPIOA
#define INPUT0_Pin GPIO_PIN_7
#define INPUT0_GPIO_Port GPIOA
#define TRANSDUCERC4_Pin GPIO_PIN_4
#define TRANSDUCERC4_GPIO_Port GPIOC
#define TRANSDUCERC5_Pin GPIO_PIN_5
#define TRANSDUCERC5_GPIO_Port GPIOC
#define TRANSDUCERB0_Pin GPIO_PIN_0
#define TRANSDUCERB0_GPIO_Port GPIOB
#define TRANSDUCERB1_Pin GPIO_PIN_1
#define TRANSDUCERB1_GPIO_Port GPIOB
#define TRANSDUCERB2_Pin GPIO_PIN_2
#define TRANSDUCERB2_GPIO_Port GPIOB
#define TRANSDUCERE7_Pin GPIO_PIN_7
#define TRANSDUCERE7_GPIO_Port GPIOE
#define TRANSDUCERE8_Pin GPIO_PIN_8
#define TRANSDUCERE8_GPIO_Port GPIOE
#define TRANSDUCERE9_Pin GPIO_PIN_9
#define TRANSDUCERE9_GPIO_Port GPIOE
#define TRANSDUCERE10_Pin GPIO_PIN_10
#define TRANSDUCERE10_GPIO_Port GPIOE
#define TRANSDUCERE11_Pin GPIO_PIN_11
#define TRANSDUCERE11_GPIO_Port GPIOE
#define TRANSDUCERE12_Pin GPIO_PIN_12
#define TRANSDUCERE12_GPIO_Port GPIOE
#define TRANSDUCERE13_Pin GPIO_PIN_13
#define TRANSDUCERE13_GPIO_Port GPIOE
#define TRANSDUCERE14_Pin GPIO_PIN_14
#define TRANSDUCERE14_GPIO_Port GPIOE
#define TRANSDUCERE15_Pin GPIO_PIN_15
#define TRANSDUCERE15_GPIO_Port GPIOE
#define TRANSDUCERB10_Pin GPIO_PIN_10
#define TRANSDUCERB10_GPIO_Port GPIOB
#define TRANSDUCERB11_Pin GPIO_PIN_11
#define TRANSDUCERB11_GPIO_Port GPIOB
#define TRANSDUCERB12_Pin GPIO_PIN_12
#define TRANSDUCERB12_GPIO_Port GPIOB
#define TRANSDUCERB13_Pin GPIO_PIN_13
#define TRANSDUCERB13_GPIO_Port GPIOB
#define TRANSDUCERB14_Pin GPIO_PIN_14
#define TRANSDUCERB14_GPIO_Port GPIOB
#define TRANSDUCERB15_Pin GPIO_PIN_15
#define TRANSDUCERB15_GPIO_Port GPIOB
#define TRANSDUCERD8_Pin GPIO_PIN_8
#define TRANSDUCERD8_GPIO_Port GPIOD
#define TRANSDUCERD9_Pin GPIO_PIN_9
#define TRANSDUCERD9_GPIO_Port GPIOD
#define TRANSDUCERD10_Pin GPIO_PIN_10
#define TRANSDUCERD10_GPIO_Port GPIOD
#define TRANSDUCERD11_Pin GPIO_PIN_11
#define TRANSDUCERD11_GPIO_Port GPIOD
#define TRANSDUCERD12_Pin GPIO_PIN_12
#define TRANSDUCERD12_GPIO_Port GPIOD
#define TRANSDUCERD13_Pin GPIO_PIN_13
#define TRANSDUCERD13_GPIO_Port GPIOD
#define TRANSDUCERD14_Pin GPIO_PIN_14
#define TRANSDUCERD14_GPIO_Port GPIOD
#define TRANSDUCERD15_Pin GPIO_PIN_15
#define TRANSDUCERD15_GPIO_Port GPIOD
#define TRANSDUCERC6_Pin GPIO_PIN_6
#define TRANSDUCERC6_GPIO_Port GPIOC
#define TRANSDUCERC7_Pin GPIO_PIN_7
#define TRANSDUCERC7_GPIO_Port GPIOC
#define TRANSDUCERC8_Pin GPIO_PIN_8
#define TRANSDUCERC8_GPIO_Port GPIOC
#define INPUT1_Pin GPIO_PIN_9
#define INPUT1_GPIO_Port GPIOA
#define INPUT2_Pin GPIO_PIN_10
#define INPUT2_GPIO_Port GPIOA
#define HEARTBEAT_Pin GPIO_PIN_15
#define HEARTBEAT_GPIO_Port GPIOA
#define TRANSDUCERC10_Pin GPIO_PIN_10
#define TRANSDUCERC10_GPIO_Port GPIOC
#define TRANSDUCERC11_Pin GPIO_PIN_11
#define TRANSDUCERC11_GPIO_Port GPIOC
#define TRANSDUCERC12_Pin GPIO_PIN_12
#define TRANSDUCERC12_GPIO_Port GPIOC
#define TRANSDUCERD0_Pin GPIO_PIN_0
#define TRANSDUCERD0_GPIO_Port GPIOD
#define TRANSDUCERD1_Pin GPIO_PIN_1
#define TRANSDUCERD1_GPIO_Port GPIOD
#define TRANSDUCERD2_Pin GPIO_PIN_2
#define TRANSDUCERD2_GPIO_Port GPIOD
#define TRANSDUCERD3_Pin GPIO_PIN_3
#define TRANSDUCERD3_GPIO_Port GPIOD
#define TRANSDUCERD4_Pin GPIO_PIN_4
#define TRANSDUCERD4_GPIO_Port GPIOD
#define TRANSDUCERD5_Pin GPIO_PIN_5
#define TRANSDUCERD5_GPIO_Port GPIOD
#define TRANSDUCERD6_Pin GPIO_PIN_6
#define TRANSDUCERD6_GPIO_Port GPIOD
#define TRANSDUCERD7_Pin GPIO_PIN_7
#define TRANSDUCERD7_GPIO_Port GPIOD
#define TRANSDUCERB3_Pin GPIO_PIN_3
#define TRANSDUCERB3_GPIO_Port GPIOB
#define TRANSDUCERB4_Pin GPIO_PIN_4
#define TRANSDUCERB4_GPIO_Port GPIOB
#define TRANSDUCERB5_Pin GPIO_PIN_5
#define TRANSDUCERB5_GPIO_Port GPIOB
#define TRANSDUCERB6_Pin GPIO_PIN_6
#define TRANSDUCERB6_GPIO_Port GPIOB
#define TRANSDUCERB7_Pin GPIO_PIN_7
#define TRANSDUCERB7_GPIO_Port GPIOB
#define TRANSDUCERB8_Pin GPIO_PIN_8
#define TRANSDUCERB8_GPIO_Port GPIOB
#define TRANSDUCERB9_Pin GPIO_PIN_9
#define TRANSDUCERB9_GPIO_Port GPIOB
#define TRANSDUCERE0_Pin GPIO_PIN_0
#define TRANSDUCERE0_GPIO_Port GPIOE
#define TRANSDUCERE1_Pin GPIO_PIN_1
#define TRANSDUCERE1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
# define VERSION 505U

# define target_loop_freq 3000
# define target_loop_period_us (1e6f/target_loop_freq)

# define LOOP_FREQ_CHECK_MS 1000U

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
