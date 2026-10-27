/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
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
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi1;

extern SPI_HandleTypeDef hspi3;

/* USER CODE BEGIN Private defines */

#ifndef UMH_SPI1_LINK_DIV
#define UMH_SPI1_LINK_DIV 4
#endif

#if UMH_SPI1_LINK_DIV == 4
#define UMH_SPI1_LINK_PRESCALER SPI_BAUDRATEPRESCALER_4
#elif UMH_SPI1_LINK_DIV == 8
#define UMH_SPI1_LINK_PRESCALER SPI_BAUDRATEPRESCALER_8
#else
#error "UMH_SPI1_LINK_DIV must be 4 or 8"
#endif

/* USER CODE END Private defines */

void MX_SPI1_Init(void);
void MX_SPI3_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

