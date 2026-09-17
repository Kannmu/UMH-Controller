/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fmac.h
  * @brief   This file contains all the function prototypes for
  *          the fmac.c file
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
#ifndef __FMAC_H__
#define __FMAC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern FMAC_HandleTypeDef hfmac;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_FMAC_Init(void);

/* One-shot Q1.15 FIR pass through the FMAC filter unit.  Coefficients are
 * loaded into the FMAC coefficient buffer, the samples are streamed through
 * the input buffer in polling mode and the complete result vector is read
 * back.  Calibration uses it for the echo leading-edge matched filter and
 * falls back to a CPU FIR when the peripheral reports an error.
 * Constraints: 1 <= taps <= 32 and taps <= length <= 255.  The helper
 * returns a full-length causal output vector (first taps-1 samples are zero)
 * so callers can share the same indexing with the CPU fallback. */
int umh_fmac_fir_q15(const int16_t *coeff, uint8_t taps,
                     const int16_t *input, uint16_t length,
                     int16_t *output);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FMAC_H__ */

