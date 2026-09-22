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
#include <string.h>

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

/* --------------------------------------------------------------------------
 * Batch CORDIC helpers.
 *
 * STM32G4 CORDIC conventions used here:
 *   - phase (atan2) inputs are Q1.31 signed values; the measured STM32G4
 *     result is angle/pi in signed Q1.31, i.e. multiply by pi/2^31 to get
 *     radians.  The old firmware assumed Q3.29, which made atan2 four/pi
 *     too large and forced every calibration fallback to libm.
 *   - sine/cosine use an angle normalised by pi and expressed in Q1.31,
 *     the result is Q1.31;
 *   - square root takes a Q1.31 value and returns a Q1.31 result.
 * A one-time self-test protects the calibration solver: if a future silicon
 * revision or a wrong configuration is detected, the helpers report failure
 * and us_calibration.c automatically falls back to the scalar libm path.
 * -------------------------------------------------------------------------- */
#define UMH_CORDIC_CHUNK 96u
#define UMH_CORDIC_PI 3.14159265358979323846f
#define UMH_CORDIC_TWO_PI 6.28318530717958647692f
#define UMH_CORDIC_Q31_SCALE 2147483648.0f
#define UMH_CORDIC_PHASE_SCALE (2147483648.0f / UMH_CORDIC_PI)

static int32_t umh_cordic_in[UMH_CORDIC_CHUNK * 2u];
static int32_t umh_cordic_out[UMH_CORDIC_CHUNK];
static int8_t umh_cordic_trig_state;
static int8_t umh_cordic_phase_state;
static int8_t umh_cordic_sqrt_state;
volatile uint32_t umh_cordic_cfg_errs;
volatile uint32_t umh_cordic_calc_errs;
volatile uint32_t umh_cordic_selftest_errs;

static HAL_StatusTypeDef umh_cordic_config(uint32_t function, uint32_t nb_write, uint32_t nb_read)
{
  CORDIC_ConfigTypeDef config;
  memset(&config, 0, sizeof(config));
  config.Function = function;
  config.Scale = CORDIC_SCALE_0;
  config.InSize = CORDIC_INSIZE_32BITS;
  config.OutSize = CORDIC_OUTSIZE_32BITS;
  config.NbWrite = nb_write;
  config.NbRead = nb_read;
  config.Precision = CORDIC_PRECISION_6CYCLES;
  {
    HAL_StatusTypeDef status = HAL_CORDIC_Configure(&hcordic, &config);
    if (status != HAL_OK) ++umh_cordic_cfg_errs;
    return status;
  }
}

static int umh_cordic_calculate(const int32_t *in, int32_t *out, uint32_t count)
{
  if (count == 0u) return 0;
  if (HAL_CORDIC_Calculate(&hcordic, in, out, count, 20u) != HAL_OK) {
    ++umh_cordic_calc_errs;
    return -1;
  }
  return 0;
}

static float umh_cordic_wrap_pi(float value)
{
  if (value > UMH_CORDIC_PI || value < -UMH_CORDIC_PI) {
    float k = value * (1.0f / UMH_CORDIC_TWO_PI);
    int32_t n = (k >= 0.0f) ? (int32_t)(k + 0.5f) : (int32_t)(k - 0.5f);
    value -= (float)n * UMH_CORDIC_TWO_PI;
  }
  return value;
}

static int32_t umh_cordic_angle_q31(float angle)
{
  float scaled;
  angle = umh_cordic_wrap_pi(angle);
  scaled = angle * (UMH_CORDIC_Q31_SCALE / UMH_CORDIC_PI);
  if (scaled >= 2147483647.0f) return 2147483647;
  if (scaled <= -2147483648.0f) return (int32_t)0x80000000;
  return (int32_t)scaled;
}

/* Raw helpers without self-test: used by the self-test itself. */
static int umh_cordic_trig_raw(uint32_t function, const float *angles, float *out, uint32_t count)
{
  uint32_t i, j;
  if (count == 0u) return 0;
  if (umh_cordic_config(function, CORDIC_NBWRITE_1, CORDIC_NBREAD_1) != HAL_OK) return -1;
  for (i = 0u; i < count; i += UMH_CORDIC_CHUNK) {
    uint32_t n = count - i;
    if (n > UMH_CORDIC_CHUNK) n = UMH_CORDIC_CHUNK;
    for (j = 0u; j < n; ++j) umh_cordic_in[j] = umh_cordic_angle_q31(angles[i + j]);
    if (umh_cordic_calculate(umh_cordic_in, umh_cordic_out, n) != 0) return -2;
    for (j = 0u; j < n; ++j)
      out[i + j] = (float)umh_cordic_out[j] * (1.0f / UMH_CORDIC_Q31_SCALE);
  }
  return 0;
}

static int umh_cordic_phase_raw(const float *real, const float *imag, float *phase, uint32_t count)
{
  uint32_t i, j;
  if (count == 0u) return 0;
  if (umh_cordic_config(CORDIC_FUNCTION_PHASE, CORDIC_NBWRITE_2, CORDIC_NBREAD_1) != HAL_OK) return -1;
  for (i = 0u; i < count; i += UMH_CORDIC_CHUNK) {
    uint32_t n = count - i;
    if (n > UMH_CORDIC_CHUNK) n = UMH_CORDIC_CHUNK;
    for (j = 0u; j < n; ++j) {
      float re = real[i + j];
      float im = imag[i + j];
      float scale = fmaxf(fabsf(re), fabsf(im));
      if (scale > 1.0e-12f) { re /= scale; im /= scale; }
      else { re = 0.0f; im = 0.0f; }
      if (re > 1.0f) re = 1.0f;
      if (re < -1.0f) re = -1.0f;
      if (im > 1.0f) im = 1.0f;
      if (im < -1.0f) im = -1.0f;
      umh_cordic_in[2u * j] = (int32_t)(re * 2147483647.0f);
      umh_cordic_in[2u * j + 1u] = (int32_t)(im * 2147483647.0f);
    }
    if (umh_cordic_calculate(umh_cordic_in, umh_cordic_out, n) != 0) return -2;
    for (j = 0u; j < n; ++j)
      phase[i + j] = umh_cordic_wrap_pi((float)umh_cordic_out[j] * (1.0f / UMH_CORDIC_PHASE_SCALE));
  }
  return 0;
}

static int umh_cordic_sqrt_raw(const float *values, float *roots, uint32_t count)
{
  uint32_t i, j;
  if (count == 0u) return 0;
  if (umh_cordic_config(CORDIC_FUNCTION_SQUAREROOT, CORDIC_NBWRITE_1, CORDIC_NBREAD_1) != HAL_OK) return -1;
  for (i = 0u; i < count; i += UMH_CORDIC_CHUNK) {
    uint32_t n = count - i;
    uint8_t shifts[UMH_CORDIC_CHUNK];
    if (n > UMH_CORDIC_CHUNK) n = UMH_CORDIC_CHUNK;
    for (j = 0u; j < n; ++j) {
      float value = values[i + j];
      uint8_t shift = 0u;
      if (value <= 0.0f) {
        shifts[j] = 0xFFu;
        umh_cordic_in[j] = 0;
        continue;
      }
      while (value >= 1.0f && shift < 15u) { value *= 0.25f; ++shift; }
      {
        float scaled = value * UMH_CORDIC_Q31_SCALE;
        if (scaled > 2147483647.0f) scaled = 2147483647.0f;
        umh_cordic_in[j] = (int32_t)scaled;
      }
      shifts[j] = shift;
    }
    if (umh_cordic_calculate(umh_cordic_in, umh_cordic_out, n) != 0) return -2;
    for (j = 0u; j < n; ++j) {
      if (shifts[j] == 0xFFu) { roots[i + j] = 0.0f; continue; }
      roots[i + j] = (float)umh_cordic_out[j] * (1.0f / UMH_CORDIC_Q31_SCALE)
                     * (float)(1u << shifts[j]);
    }
  }
  return 0;
}

static int8_t umh_cordic_selftest_trig(void)
{
  float angles[2];
  float sn[2], cs[2];
  angles[0] = UMH_CORDIC_PI * 0.5f;
  angles[1] = UMH_CORDIC_PI * 0.25f;
  if (umh_cordic_trig_raw(CORDIC_FUNCTION_SINE, angles, sn, 2u) != 0) return -1;
  if (umh_cordic_trig_raw(CORDIC_FUNCTION_COSINE, angles, cs, 2u) != 0) return -1;
  if (fabsf(sn[0] - 1.0f) > 2.0e-2f || fabsf(cs[0]) > 2.0e-2f) { ++umh_cordic_selftest_errs; return -1; }
  if (fabsf(sn[1] - 0.70710678f) > 2.0e-2f || fabsf(cs[1] - 0.70710678f) > 2.0e-2f) { ++umh_cordic_selftest_errs; return -1; }
  return 1;
}

static int8_t umh_cordic_selftest_phase(void)
{
  float re[2], im[2], ph[2];
  re[0] = 1.0f; im[0] = 0.0f;
  re[1] = 0.0f; im[1] = 1.0f;
  if (umh_cordic_phase_raw(re, im, ph, 2u) != 0) return -1;
  if (fabsf(ph[0]) > 2.0e-2f) { ++umh_cordic_selftest_errs; return -1; }
  if (fabsf(ph[1] - UMH_CORDIC_PI * 0.5f) > 2.0e-2f) { ++umh_cordic_selftest_errs; return -1; }
  return 1;
}

static int8_t umh_cordic_selftest_sqrt(void)
{
  float in[2], out[2];
  in[0] = 0.25f; in[1] = 4.0f;
  if (umh_cordic_sqrt_raw(in, out, 2u) != 0) return -1;
  if (fabsf(out[0] - 0.5f) > 2.0e-2f) { ++umh_cordic_selftest_errs; return -1; }
  if (fabsf(out[1] - 2.0f) > 2.0e-2f) { ++umh_cordic_selftest_errs; return -1; }
  return 1;
}

int umh_cordic_sincos_batch(const float *angles, float *sin_out, float *cos_out, uint32_t count)
{
  if (angles == NULL || sin_out == NULL || cos_out == NULL) return -1;
  if (umh_cordic_trig_state == 0) umh_cordic_trig_state = umh_cordic_selftest_trig();
  if (umh_cordic_trig_state < 0) return -2;
  if (umh_cordic_trig_raw(CORDIC_FUNCTION_SINE, angles, sin_out, count) != 0) return -3;
  if (umh_cordic_trig_raw(CORDIC_FUNCTION_COSINE, angles, cos_out, count) != 0) return -4;
  return 0;
}

int umh_cordic_phase_batch(const float *real, const float *imag, float *phase_rad, uint32_t count)
{
  if (real == NULL || imag == NULL || phase_rad == NULL) return -1;
  if (umh_cordic_phase_state == 0) umh_cordic_phase_state = umh_cordic_selftest_phase();
  if (umh_cordic_phase_state < 0) return -2;
  return umh_cordic_phase_raw(real, imag, phase_rad, count) == 0 ? 0 : -3;
}

int umh_cordic_sqrt_batch(const float *values, float *roots, uint32_t count)
{
  if (values == NULL || roots == NULL) return -1;
  if (umh_cordic_sqrt_state == 0) umh_cordic_sqrt_state = umh_cordic_selftest_sqrt();
  if (umh_cordic_sqrt_state < 0) return -2;
  return umh_cordic_sqrt_raw(values, roots, count) == 0 ? 0 : -3;
}

int umh_cordic_sincos(float angle, float *sin_out, float *cos_out)
{
  return umh_cordic_sincos_batch(&angle, sin_out, cos_out, 1u);
}

int umh_cordic_phase(float real, float imag, float *phase_rad)
{
  return umh_cordic_phase_batch(&real, &imag, phase_rad, 1u);
}

int umh_cordic_sqrt(float value, float *root)
{
  return umh_cordic_sqrt_batch(&value, root, 1u);
}

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
  /* Phase result is angle/pi in signed Q1.31 on this STM32G4. */
  angle = (float)output * (UMH_CORDIC_PI / 2147483648.0f);
  if (angle < 0.0f) angle += 6.28318530717958647692f;
  code = (int32_t)(angle * (256.0f / 6.28318530717958647692f) + 0.5f);
  *phase = (uint8_t)(code & 0xFF);
  return 0;
}

int umh_cordic_phase8_batch(const float *real, const float *imag,
                            uint8_t *phase_codes, uint32_t count)
{
  uint32_t i;
  if (real == NULL || imag == NULL || phase_codes == NULL) return -1;
  if (count == 0u) return 0;
  if (umh_cordic_phase_state == 0) umh_cordic_phase_state = umh_cordic_selftest_phase();
  if (umh_cordic_phase_state < 0) return -2;
  /* Direct CSR/WDATA/RDATA access.  The HAL polling helper re-arms its own
   * timeout bookkeeping and calls HAL_GetTick() for every word; the 84-channel
   * finalize runs at up to 2 kHz, so the register path keeps the CORDIC
   * pipeline busy instead of spending most of its time in the wrapper. */
  CORDIC->CSR = (uint32_t)(CORDIC_FUNCTION_PHASE | CORDIC_PRECISION_6CYCLES |
                           CORDIC_NBWRITE_2);
  for (i = 0u; i < count; ++i) {
    float re = real[i];
    float im = imag[i];
    int32_t out_q31;
    if (re > 1.0f) re = 1.0f;
    if (re < -1.0f) re = -1.0f;
    if (im > 1.0f) im = 1.0f;
    if (im < -1.0f) im = -1.0f;
    CORDIC->WDATA = (int32_t)(re * 2147483647.0f);
    CORDIC->WDATA = (int32_t)(im * 2147483647.0f);
    while ((CORDIC->CSR & CORDIC_CSR_RRDY) == 0u) {
    }
    out_q31 = (int32_t)CORDIC->RDATA;
    /* Phase result is angle/pi in Q1.31; the 8-bit wire code is
     * angle/(2*pi)*256 = out_q31/2^24, rounded to nearest. */
    phase_codes[i] = (uint8_t)((uint32_t)((out_q31 + (1 << 23)) >> 24) & 0xFFu);
  }
  return 0;
}

/* USER CODE END 1 */



