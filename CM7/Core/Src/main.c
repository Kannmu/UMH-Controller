/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DFSDM_Channel_HandleTypeDef hdfsdm1_channel3;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel4;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel5;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DFSDM1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_FDCAN;
  PeriphClkInitStruct.PLL2.PLL2M = 5;
  PeriphClkInitStruct.PLL2.PLL2N = 40;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel3.Instance = DFSDM1_Channel3;
  hdfsdm1_channel3.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel3.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel3.Init.OutputClock.Divider = 200;
  hdfsdm1_channel3.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel3.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel3.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel3.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_FALLING;
  hdfsdm1_channel3.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel3.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel3.Init.Awd.Oversampling = 1;
  hdfsdm1_channel3.Init.Offset = 0;
  hdfsdm1_channel3.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel3) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel4.Instance = DFSDM1_Channel4;
  hdfsdm1_channel4.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel4.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel4.Init.OutputClock.Divider = 200;
  hdfsdm1_channel4.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel4.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel4.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel4.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel4.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel4.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel4.Init.Awd.Oversampling = 1;
  hdfsdm1_channel4.Init.Offset = 0;
  hdfsdm1_channel4.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel4) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel5.Instance = DFSDM1_Channel5;
  hdfsdm1_channel5.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel5.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel5.Init.OutputClock.Divider = 200;
  hdfsdm1_channel5.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel5.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel5.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel5.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel5.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel5.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel5.Init.Awd.Oversampling = 1;
  hdfsdm1_channel5.Init.Offset = 0;
  hdfsdm1_channel5.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, Transducer_Pin|TransducerI5_Pin|TransducerI4_Pin|TransducerI1_Pin
                          |TransducerI0_Pin|TransducerI7_Pin|TransducerI2_Pin|TransducerI3_Pin
                          |TransducerI9_Pin|TransducerI8_Pin|TransducerI10_Pin|TransducerI11_Pin
                          |TransducerI12_Pin|TransducerI13_Pin|TransducerI14_Pin|TransducerI15_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TransducerB5_Pin|TransducerB6_Pin|TransducerB7_Pin|TransducerB9_Pin
                          |TransducerB8_Pin|TransducerB2_Pin|TransducerB1_Pin|TransducerB12_Pin
                          |TransducerB15_Pin|TransducerB0_Pin|TransducerB13_Pin|TransducerB14_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, TransducerG9_Pin|TransducerG11_Pin|TransducerG12_Pin|TransducerG15_Pin
                          |TransducerG14_Pin|TransducerG13_Pin|TransducerG8_Pin|TransducerG7_Pin
                          |TransducerG5_Pin|TransducerG6_Pin|TransducerG4_Pin|TransducerG3_Pin
                          |TransducerG2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, TransducerD5_Pin|TransducerD4_Pin|TransducerD6_Pin|TransducerD3_Pin
                          |TransducerD7_Pin|TransducerD2_Pin|TransducerD0_Pin|TransducerD1_Pin
                          |TransducerD15_Pin|TransducerD14_Pin|TransducerD11_Pin|TransducerD12_Pin
                          |TransducerD13_Pin|TransducerD10_Pin|TransducerD9_Pin|TransducerD8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, TransducerC10_Pin|TransducerC8_Pin|TransducerC9_Pin|TransducerC7_Pin
                          |TransducerC6_Pin|TransducerC0_Pin|TransducerC1_Pin|TransducerC2_Pin
                          |TransducerC4_Pin|TransducerC5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, TransducerE1_Pin|TransducerE2_Pin|TransducerE0_Pin|TransducerE5_Pin
                          |TransducerE4_Pin|TransducerE3_Pin|TransducerE6_Pin|TransducerE7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOJ, TransducerJ14_Pin|TransducerJ12_Pin|TransducerJ13_Pin|TransducerJ11_Pin
                          |TransducerJ10_Pin|TransducerJ9_Pin|TransducerJ0_Pin|TransducerJ8_Pin
                          |TransducerJ7_Pin|TransducerJ6_Pin|TransducerJ1_Pin|TransducerJ5_Pin
                          |TransducerJ2_Pin|TransducerJ3_Pin|TransducerJ4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, TransducerF2_Pin|TransducerF1_Pin|TransducerF0_Pin|TransducerF3_Pin
                          |TransducerF5_Pin|TransducerF4_Pin|TransducerF6_Pin|TransducerF7_Pin
                          |TransducerF8_Pin|TransducerF10_Pin|TransducerF9_Pin|TransducerF13_Pin
                          |TransducerF14_Pin|TransducerF12_Pin|TransducerF15_Pin|TransducerF11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Transducer_Pin TransducerI5_Pin TransducerI4_Pin TransducerI1_Pin
                           TransducerI0_Pin TransducerI7_Pin TransducerI2_Pin TransducerI3_Pin
                           TransducerI9_Pin TransducerI10_Pin TransducerI11_Pin TransducerI12_Pin
                           TransducerI13_Pin TransducerI14_Pin TransducerI15_Pin */
  GPIO_InitStruct.Pin = Transducer_Pin|TransducerI5_Pin|TransducerI4_Pin|TransducerI1_Pin
                          |TransducerI0_Pin|TransducerI7_Pin|TransducerI2_Pin|TransducerI3_Pin
                          |TransducerI9_Pin|TransducerI10_Pin|TransducerI11_Pin|TransducerI12_Pin
                          |TransducerI13_Pin|TransducerI14_Pin|TransducerI15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : TransducerB5_Pin TransducerB6_Pin TransducerB7_Pin TransducerB9_Pin
                           TransducerB8_Pin TransducerB2_Pin TransducerB1_Pin TransducerB12_Pin
                           TransducerB15_Pin TransducerB0_Pin TransducerB13_Pin TransducerB14_Pin */
  GPIO_InitStruct.Pin = TransducerB5_Pin|TransducerB6_Pin|TransducerB7_Pin|TransducerB9_Pin
                          |TransducerB8_Pin|TransducerB2_Pin|TransducerB1_Pin|TransducerB12_Pin
                          |TransducerB15_Pin|TransducerB0_Pin|TransducerB13_Pin|TransducerB14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TransducerG9_Pin TransducerG11_Pin TransducerG12_Pin TransducerG15_Pin
                           TransducerG14_Pin TransducerG13_Pin TransducerG8_Pin TransducerG7_Pin
                           TransducerG5_Pin TransducerG6_Pin TransducerG4_Pin TransducerG3_Pin
                           TransducerG2_Pin */
  GPIO_InitStruct.Pin = TransducerG9_Pin|TransducerG11_Pin|TransducerG12_Pin|TransducerG15_Pin
                          |TransducerG14_Pin|TransducerG13_Pin|TransducerG8_Pin|TransducerG7_Pin
                          |TransducerG5_Pin|TransducerG6_Pin|TransducerG4_Pin|TransducerG3_Pin
                          |TransducerG2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : TransducerD5_Pin TransducerD4_Pin TransducerD6_Pin TransducerD3_Pin
                           TransducerD7_Pin TransducerD2_Pin TransducerD0_Pin TransducerD1_Pin
                           TransducerD15_Pin TransducerD14_Pin TransducerD11_Pin TransducerD12_Pin
                           TransducerD13_Pin TransducerD10_Pin TransducerD9_Pin TransducerD8_Pin */
  GPIO_InitStruct.Pin = TransducerD5_Pin|TransducerD4_Pin|TransducerD6_Pin|TransducerD3_Pin
                          |TransducerD7_Pin|TransducerD2_Pin|TransducerD0_Pin|TransducerD1_Pin
                          |TransducerD15_Pin|TransducerD14_Pin|TransducerD11_Pin|TransducerD12_Pin
                          |TransducerD13_Pin|TransducerD10_Pin|TransducerD9_Pin|TransducerD8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : TransducerC10_Pin TransducerC8_Pin TransducerC9_Pin TransducerC7_Pin
                           TransducerC6_Pin TransducerC0_Pin TransducerC1_Pin TransducerC2_Pin
                           TransducerC4_Pin TransducerC5_Pin */
  GPIO_InitStruct.Pin = TransducerC10_Pin|TransducerC8_Pin|TransducerC9_Pin|TransducerC7_Pin
                          |TransducerC6_Pin|TransducerC0_Pin|TransducerC1_Pin|TransducerC2_Pin
                          |TransducerC4_Pin|TransducerC5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TransducerE1_Pin TransducerE2_Pin TransducerE0_Pin TransducerE5_Pin
                           TransducerE4_Pin TransducerE3_Pin TransducerE6_Pin TransducerE7_Pin */
  GPIO_InitStruct.Pin = TransducerE1_Pin|TransducerE2_Pin|TransducerE0_Pin|TransducerE5_Pin
                          |TransducerE4_Pin|TransducerE3_Pin|TransducerE6_Pin|TransducerE7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : TransducerJ14_Pin TransducerJ12_Pin TransducerJ13_Pin TransducerJ11_Pin
                           TransducerJ10_Pin TransducerJ9_Pin TransducerJ0_Pin TransducerJ8_Pin
                           TransducerJ7_Pin TransducerJ6_Pin TransducerJ1_Pin TransducerJ5_Pin
                           TransducerJ2_Pin TransducerJ3_Pin TransducerJ4_Pin */
  GPIO_InitStruct.Pin = TransducerJ14_Pin|TransducerJ12_Pin|TransducerJ13_Pin|TransducerJ11_Pin
                          |TransducerJ10_Pin|TransducerJ9_Pin|TransducerJ0_Pin|TransducerJ8_Pin
                          |TransducerJ7_Pin|TransducerJ6_Pin|TransducerJ1_Pin|TransducerJ5_Pin
                          |TransducerJ2_Pin|TransducerJ3_Pin|TransducerJ4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*Configure GPIO pin : TransducerI8_Pin */
  GPIO_InitStruct.Pin = TransducerI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TransducerI8_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TransducerF2_Pin TransducerF1_Pin TransducerF0_Pin TransducerF3_Pin
                           TransducerF5_Pin TransducerF4_Pin TransducerF6_Pin TransducerF7_Pin
                           TransducerF8_Pin TransducerF10_Pin TransducerF9_Pin TransducerF13_Pin
                           TransducerF14_Pin TransducerF12_Pin TransducerF15_Pin TransducerF11_Pin */
  GPIO_InitStruct.Pin = TransducerF2_Pin|TransducerF1_Pin|TransducerF0_Pin|TransducerF3_Pin
                          |TransducerF5_Pin|TransducerF4_Pin|TransducerF6_Pin|TransducerF7_Pin
                          |TransducerF8_Pin|TransducerF10_Pin|TransducerF9_Pin|TransducerF13_Pin
                          |TransducerF14_Pin|TransducerF12_Pin|TransducerF15_Pin|TransducerF11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
