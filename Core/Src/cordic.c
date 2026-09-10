/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    cordic.c
  * @brief   This file provides code for the configuration
  *          of the CORDIC instances.
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
/* Includes ------------------------------------------------------------------*/
#include "cordic.h"
#include <math.h>
#include <stdint.h>

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CORDIC_HandleTypeDef hcordic;
DMA_HandleTypeDef hdma_cordic_read;
DMA_HandleTypeDef hdma_cordic_write;

/* CORDIC init function */
void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

}

void HAL_CORDIC_MspInit(CORDIC_HandleTypeDef* cordicHandle)
{

  if(cordicHandle->Instance==CORDIC)
  {
  /* USER CODE BEGIN CORDIC_MspInit 0 */

  /* USER CODE END CORDIC_MspInit 0 */
    /* CORDIC clock enable */
    __HAL_RCC_CORDIC_CLK_ENABLE();

    /* CORDIC DMA Init */
    /* CORDIC_READ Init */
    hdma_cordic_read.Instance = DMA1_Channel3;
    hdma_cordic_read.Init.Request = DMA_REQUEST_CORDIC_READ;
    hdma_cordic_read.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_cordic_read.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_cordic_read.Init.MemInc = DMA_MINC_ENABLE;
    hdma_cordic_read.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_cordic_read.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_cordic_read.Init.Mode = DMA_NORMAL;
    hdma_cordic_read.Init.Priority = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(&hdma_cordic_read) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(cordicHandle,hdmaOut,hdma_cordic_read);

    /* CORDIC_WRITE Init */
    hdma_cordic_write.Instance = DMA1_Channel4;
    hdma_cordic_write.Init.Request = DMA_REQUEST_CORDIC_WRITE;
    hdma_cordic_write.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_cordic_write.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_cordic_write.Init.MemInc = DMA_MINC_ENABLE;
    hdma_cordic_write.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_cordic_write.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_cordic_write.Init.Mode = DMA_NORMAL;
    hdma_cordic_write.Init.Priority = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(&hdma_cordic_write) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(cordicHandle,hdmaIn,hdma_cordic_write);

  /* USER CODE BEGIN CORDIC_MspInit 1 */

  /* USER CODE END CORDIC_MspInit 1 */
  }
}

void HAL_CORDIC_MspDeInit(CORDIC_HandleTypeDef* cordicHandle)
{

  if(cordicHandle->Instance==CORDIC)
  {
  /* USER CODE BEGIN CORDIC_MspDeInit 0 */

  /* USER CODE END CORDIC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CORDIC_CLK_DISABLE();

    /* CORDIC DMA DeInit */
    HAL_DMA_DeInit(cordicHandle->hdmaOut);
    HAL_DMA_DeInit(cordicHandle->hdmaIn);
  /* USER CODE BEGIN CORDIC_MspDeInit 1 */

  /* USER CODE END CORDIC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

int umh_cordic_phase8(float real, float imag, uint8_t *phase)
{
  CORDIC_ConfigTypeDef config;
  int32_t input[2];
  int32_t output;
  float angle;
  int32_t code;
  if (phase == NULL || (real == 0.0f && imag == 0.0f)) return -1;
  if (real > 1.0f) real = 1.0f;
  if (real < -1.0f) real = -1.0f;
  if (imag > 1.0f) imag = 1.0f;
  if (imag < -1.0f) imag = -1.0f;
  input[0] = (int32_t)(real * 2147483647.0f);
  input[1] = (int32_t)(imag * 2147483647.0f);
  config.Function = CORDIC_FUNCTION_PHASE;
  config.Scale = CORDIC_SCALE_0;
  config.InSize = CORDIC_INSIZE_32BITS;
  config.OutSize = CORDIC_OUTSIZE_32BITS;
  config.NbWrite = CORDIC_NBWRITE_2;
  config.NbRead = CORDIC_NBREAD_1;
  config.Precision = CORDIC_PRECISION_6CYCLES;
  if (HAL_CORDIC_Configure(&hcordic, &config) != HAL_OK ||
      HAL_CORDIC_Calculate(&hcordic, input, &output, 1u, 2u) != HAL_OK) return -1;
  /* The phase result is Q3.29 radians on STM32G4 CORDIC. */
  angle = (float)output / 536870912.0f;
  if (angle < 0.0f) angle += 6.28318530717958647692f;
  code = (int32_t)(angle * (256.0f / 6.28318530717958647692f) + 0.5f);
  *phase = (uint8_t)(code & 0xFF);
  return 0;
}

/* USER CODE END 1 */

