/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    crc.c
  * @brief   This file provides code for the configuration
  *          of the CRC instances.
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
#include "crc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CRC_HandleTypeDef hcrc;

/* CRC init function */
void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  /* The firmware CRC32 contract is standard CRC-32/ISO-HDLC (reflected
   * 0xEDB88320, init/xorout 0xFFFFFFFF).  With the default polynomial and
   * byte input inversion plus output reversal, the peripheral computes the
   * bit-reflected raw value; umh_crc32() applies the final complement. */
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

void HAL_CRC_MspInit(CRC_HandleTypeDef* crcHandle)
{

  if(crcHandle->Instance==CRC)
  {
  /* USER CODE BEGIN CRC_MspInit 0 */

  /* USER CODE END CRC_MspInit 0 */
    /* CRC clock enable */
    __HAL_RCC_CRC_CLK_ENABLE();
  /* USER CODE BEGIN CRC_MspInit 1 */

  /* USER CODE END CRC_MspInit 1 */
  }
}

void HAL_CRC_MspDeInit(CRC_HandleTypeDef* crcHandle)
{

  if(crcHandle->Instance==CRC)
  {
  /* USER CODE BEGIN CRC_MspDeInit 0 */

  /* USER CODE END CRC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CRC_CLK_DISABLE();
  /* USER CODE BEGIN CRC_MspDeInit 1 */

  /* USER CODE END CRC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* Standard CRC-32/ISO-HDLC software core.  Exposed as an incremental update
 * so the flash/EEPROM layer can verify objects larger than its staging
 * buffer without a second copy of the algorithm. */
uint32_t umh_crc32_update(uint32_t state, const void *data, uint32_t length)
{
  const uint8_t *bytes = (const uint8_t *)data;
  uint32_t i;
  uint8_t bit;
  if (bytes == NULL) return state;
  for (i = 0u; i < length; ++i) {
    state ^= bytes[i];
    for (bit = 0u; bit < 8u; ++bit)
      state = (state >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(state & 1u));
  }
  return state;
}

uint32_t umh_crc32_finish(uint32_t state)
{
  return ~state;
}

/* Hardware fast path.  A one-time known-answer test (CRC32("123456789") =
 * 0xCBF43926) guards the EEPROM/NOR data contract: if a future CubeMX
 * regeneration changes the peripheral configuration, umh_crc32 silently
 * stays on the bit-exact software path instead of rejecting stored records. */
uint32_t umh_crc32(const void *data, uint32_t length)
{
  static const uint8_t check_vector[9] = {'1','2','3','4','5','6','7','8','9'};
  static uint8_t hardware_ok;
  uint32_t primask;
  uint32_t value;
  if (data == NULL) return 0u;

  if (hardware_ok == 0u) {
    primask = __get_PRIMASK();
    __disable_irq();
    value = ~HAL_CRC_Calculate(&hcrc, (const uint32_t *)(const void *)check_vector,
                               (uint32_t)sizeof(check_vector));
    __set_PRIMASK(primask);
    hardware_ok = (value == 0xCBF43926u) ? 1u : 2u;
  }
  if (hardware_ok == 1u) {
    primask = __get_PRIMASK();
    __disable_irq();
    value = ~HAL_CRC_Calculate(&hcrc, (const uint32_t *)(const void *)data, length);
    __set_PRIMASK(primask);
    return value;
  }
  return umh_crc32_finish(umh_crc32_update(0xFFFFFFFFu, data, length));
}

/* USER CODE END 1 */

