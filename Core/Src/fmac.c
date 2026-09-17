/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fmac.c
  * @brief   This file provides code for the configuration
  *          of the FMAC instances.
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
#include "fmac.h"
#include <string.h>

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

FMAC_HandleTypeDef hfmac;
DMA_HandleTypeDef hdma_fmac_preload;
DMA_HandleTypeDef hdma_fmac_read;
DMA_HandleTypeDef hdma_fmac_write;

/* FMAC init function */
void MX_FMAC_Init(void)
{

  /* USER CODE BEGIN FMAC_Init 0 */

  /* USER CODE END FMAC_Init 0 */

  /* USER CODE BEGIN FMAC_Init 1 */

  /* USER CODE END FMAC_Init 1 */
  hfmac.Instance = FMAC;
  if (HAL_FMAC_Init(&hfmac) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FMAC_Init 2 */

  /* USER CODE END FMAC_Init 2 */

}

void HAL_FMAC_MspInit(FMAC_HandleTypeDef* fmacHandle)
{

  if(fmacHandle->Instance==FMAC)
  {
  /* USER CODE BEGIN FMAC_MspInit 0 */

  /* USER CODE END FMAC_MspInit 0 */
    /* FMAC clock enable */
    __HAL_RCC_FMAC_CLK_ENABLE();

    /* FMAC DMA Init */
    /* FMAC_PRELOAD Init */
    hdma_fmac_preload.Instance = DMA1_Channel7;
    hdma_fmac_preload.Init.Request = DMA_REQUEST_MEM2MEM;
    hdma_fmac_preload.Init.Direction = DMA_MEMORY_TO_MEMORY;
    hdma_fmac_preload.Init.PeriphInc = DMA_PINC_ENABLE;
    hdma_fmac_preload.Init.MemInc = DMA_MINC_DISABLE;
    hdma_fmac_preload.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_fmac_preload.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_fmac_preload.Init.Mode = DMA_CIRCULAR;
    hdma_fmac_preload.Init.Priority = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(&hdma_fmac_preload) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(fmacHandle,hdmaPreload,hdma_fmac_preload);

    /* FMAC_READ Init */
    hdma_fmac_read.Instance = DMA1_Channel8;
    hdma_fmac_read.Init.Request = DMA_REQUEST_FMAC_READ;
    hdma_fmac_read.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_fmac_read.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_fmac_read.Init.MemInc = DMA_MINC_ENABLE;
    hdma_fmac_read.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_fmac_read.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_fmac_read.Init.Mode = DMA_CIRCULAR;
    hdma_fmac_read.Init.Priority = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(&hdma_fmac_read) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(fmacHandle,hdmaOut,hdma_fmac_read);

    /* FMAC_WRITE Init */
    hdma_fmac_write.Instance = DMA2_Channel1;
    hdma_fmac_write.Init.Request = DMA_REQUEST_FMAC_WRITE;
    hdma_fmac_write.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_fmac_write.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_fmac_write.Init.MemInc = DMA_MINC_ENABLE;
    hdma_fmac_write.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_fmac_write.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_fmac_write.Init.Mode = DMA_CIRCULAR;
    hdma_fmac_write.Init.Priority = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(&hdma_fmac_write) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(fmacHandle,hdmaIn,hdma_fmac_write);

  /* USER CODE BEGIN FMAC_MspInit 1 */

  /* USER CODE END FMAC_MspInit 1 */
  }
}

void HAL_FMAC_MspDeInit(FMAC_HandleTypeDef* fmacHandle)
{

  if(fmacHandle->Instance==FMAC)
  {
  /* USER CODE BEGIN FMAC_MspDeInit 0 */

  /* USER CODE END FMAC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FMAC_CLK_DISABLE();

    /* FMAC DMA DeInit */
    HAL_DMA_DeInit(fmacHandle->hdmaPreload);
    HAL_DMA_DeInit(fmacHandle->hdmaOut);
    HAL_DMA_DeInit(fmacHandle->hdmaIn);
  /* USER CODE BEGIN FMAC_MspDeInit 1 */

  /* USER CODE END FMAC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

volatile uint16_t umh_fmac_debug_output_size;
volatile int16_t umh_fmac_debug_last_output;

/* One-shot Q1.15 FIR pass through the FMAC filter unit.  Coefficients are
 * loaded into the coefficient buffer, the samples are streamed through
 * polling mode and the complete causal output vector is presented to the
 * caller.  Constraints: 1 <= taps <= 32, taps <= length <= 255.
 *
 * FMAC emits only N-P+1 results (y[P-1]..y[N-1]); this helper shifts those
 * results to the tail and zero-fills the first P-1 samples, matching the
 * CPU fallback used by cal_fmac_refine(). */
int umh_fmac_fir_q15(const int16_t *coeff, uint8_t taps,
                     const int16_t *input, uint16_t length,
                     int16_t *output)
{
  FMAC_FilterConfigTypeDef config;
  uint16_t input_size;
  uint16_t output_size;
  uint16_t produced;
  HAL_StatusTypeDef status;

  if (coeff == NULL || input == NULL || output == NULL) return -1;
  if (taps == 0u || taps > 32u || length == 0u || length > 255u) return -1;
  if (length < taps) return -1;
  produced = (uint16_t)(length - taps + 1u);

  memset(&config, 0, sizeof(config));
  /* Internal FMAC memory is 256 16-bit words.  Layout: X1[0..63],
   * X2[64..95], Y[96..159].  Only polling mode is used, so the DMA
   * channels configured by MspInit stay idle. */
  config.InputBaseAddress = 0u;
  config.InputBufferSize = 64u;
  config.InputThreshold = FMAC_THRESHOLD_NO_VALUE;
  config.CoeffBaseAddress = 64u;
  config.CoeffBufferSize = 32u;
  config.OutputBaseAddress = 96u;
  config.OutputBufferSize = 64u;
  config.OutputThreshold = FMAC_THRESHOLD_NO_VALUE;
  config.InputAccess = FMAC_BUFFER_ACCESS_POLLING;
  config.OutputAccess = FMAC_BUFFER_ACCESS_POLLING;
  config.Clip = FMAC_CLIP_DISABLED;
  config.Filter = FMAC_FUNC_CONVO_FIR;
  config.P = taps;
  config.Q = 0u;
  config.R = 0u;
  config.pCoeffA = NULL;
  config.CoeffASize = 0u;
  config.pCoeffB = (int16_t *)coeff;
  config.CoeffBSize = taps;

  status = HAL_FMAC_FilterConfig(&hfmac, &config);
  if (status != HAL_OK) return -2;

  input_size = length;
  output_size = length;
  status = HAL_FMAC_FilterStart(&hfmac, output, &output_size);
  if (status != HAL_OK) {
    (void)HAL_FMAC_FilterStop(&hfmac);
    return -3;
  }
  status = HAL_FMAC_AppendFilterData(&hfmac, (int16_t *)input, &input_size);
  if (status != HAL_OK) {
    (void)HAL_FMAC_FilterStop(&hfmac);
    return -4;
  }
  status = HAL_FMAC_PollFilterData(&hfmac, 50u);
  umh_fmac_debug_output_size = output_size;
  umh_fmac_debug_last_output = (output_size != 0u) ? output[0] : (int16_t)0;
  (void)HAL_FMAC_FilterStop(&hfmac);
  if (status != HAL_OK) return -5;
  if (output_size != produced) return -6;

  /* Present the same full-length causal FIR vector as the CPU fallback. */
  if (taps > 1u) {
    memmove(&output[taps - 1u], output, (size_t)produced * sizeof(int16_t));
    memset(output, 0, (size_t)(taps - 1u) * sizeof(int16_t));
  }
  return 0;
}