/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    cordic.h
  * @brief   This file contains all the function prototypes for
  *          the cordic.c file
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
#ifndef __CORDIC_H__
#define __CORDIC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CORDIC_HandleTypeDef hcordic;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_CORDIC_Init(void);
int umh_cordic_phase8_batch(const float *real, const float *imag,
                             uint8_t *phase_codes, uint32_t count);

/* Batch hardware trigonometry helpers used by the ultrasound calibration
 * solver.  They return 0 on success and a negative value when the CORDIC
 * unit is unavailable, so callers keep a scalar libm fallback.  Arbitrary
 * counts are internally chunked through a small static Q1.31 staging
 * buffer. */
int umh_cordic_sincos_batch(const float *angles, float *sin_out, float *cos_out,
                            uint32_t count);
int umh_cordic_phase_batch(const float *real, const float *imag, float *phase_rad,
                           uint32_t count);
int umh_cordic_phase(float real, float imag, float *phase_rad);

/* Streaming atan2 for per-channel render loops.  begin() runs the phase
 * self-test once and leaves the unit in PHASE mode; it returns non-zero when
 * the caller must use atan2f instead.  Each call then costs one register
 * round trip instead of a ~300-cycle newlib atan2f.  x/y are in the caller's
 * unit and only their ratio matters; |x|,|y| <= 262 mm when given in um.
 * Nothing else may use the CORDIC between begin() and the last call. */
int umh_cordic_phase_stream_begin(void);

static inline int32_t umh_cordic_phase_stream_q31(float x_um, float y_um)
{
  /* um * 8192 = um / 2^18 in Q1.31, so +-262 mm spans the full input range. */
  float xs = x_um * 8192.0f;
  float ys = y_um * 8192.0f;
  if (xs > 2147483520.0f) xs = 2147483520.0f;
  if (xs < -2147483520.0f) xs = -2147483520.0f;
  if (ys > 2147483520.0f) ys = 2147483520.0f;
  if (ys < -2147483520.0f) ys = -2147483520.0f;
  CORDIC->WDATA = (uint32_t)(int32_t)xs;
  CORDIC->WDATA = (uint32_t)(int32_t)ys;
  while ((CORDIC->CSR & CORDIC_CSR_RRDY) == 0u) {
  }
  return (int32_t)CORDIC->RDATA;
}

/* Phase result is angle/pi in signed Q1.31; one 1/1024 turn is 2^22. */
static inline int32_t umh_cordic_phase_stream_q10(float x_um, float y_um)
{
  return umh_cordic_phase_stream_q31(x_um, y_um) >> 22;
}

/* Phase as a signed fractional turn (-0.5 .. +0.5).  Keeps the CORDIC
 * round-trip without quantising to the 10-bit lookup-table grid first, which
 * lets the vortex emitter preserve its original lroundf() arithmetic. */
static inline float umh_cordic_phase_stream_turns(float x_um, float y_um)
{
  return (float)umh_cordic_phase_stream_q31(x_um, y_um) *
         (1.0f / 4294967296.0f);
}

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CORDIC_H__ */


