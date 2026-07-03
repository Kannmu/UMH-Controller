#define _USE_MATH_DEFINES
#include "calib_adc.h"
#include "transducer.h"
#include <math.h>

TIM_HandleTypeDef htim6;
DMA_HandleTypeDef hdma_adc1_calib;
volatile uint8_t adc_capture_done = 0;

__ALIGNED(32)
uint16_t adc_buffer[CALIB_ADC_BUFFER_SIZE] __attribute__((section(".storage_buffer")));

/* 10-sample IQ LUT: cos/sin at 400kHz sample rate for 40kHz target.
 * cos(2*PI*k/10), sin(2*PI*k/10) for k = 0..9 */
static const float cos_lut[10] = {
     1.000000000f,  0.809016994f,  0.309016994f, -0.309016994f, -0.809016994f,
    -1.000000000f, -0.809016994f, -0.309016994f,  0.309016994f,  0.809016994f
};
static const float sin_lut[10] = {
     0.000000000f,  0.587785252f,  0.951056516f,  0.951056516f,  0.587785252f,
     0.000000000f, -0.587785252f, -0.951056516f, -0.951056516f, -0.587785252f
};

static void Calib_ADC_DMA_Complete(DMA_HandleTypeDef *hdma);

/* ---- TIM6: 400kHz TRGO=UPDATE, triggers ADC1 ---- */
void MX_TIM6_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* APB1 timer clock: PCLK1 when D2PPRE1==0; 2×PCLK1 otherwise.
     * Our config: RCC_APB1_DIV2 → D2PPRE1 != 0 → TIM6_CLK = 2×PCLK1 = 200MHz.
     * Prescaler=0, Period=499 → 200MHz/500 = 400kHz. */
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t tim6_clk = pclk1;
    if ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) != 0U)
        tim6_clk = pclk1 * 2U;

    htim6.Instance = TIM6;
    htim6.Init.Prescaler         = 0;
    htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim6.Init.Period            = (tim6_clk / CALIB_ADC_SAMPLING_FREQ) - 1U;  /* 400kHz */
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
        Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
        Error_Handler();
}

/* ---- ADC1 reconfig for DMA IQ capture ---- */
void Calib_ADC_Configure(void)
{
    extern ADC_HandleTypeDef hadc1;

    HAL_ADC_DeInit(&hadc1);

    /* Enable DMA1 clock (in case not already) */
    __HAL_RCC_DMA1_CLK_ENABLE();

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;
    hadc1.Init.Resolution            = ADC_RESOLUTION_16B;
    hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait      = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.NbrOfConversion       = 2;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIG_T6_TRGO;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
    hadc1.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
    hadc1.Init.LeftBitShift          = ADC_LEFTBITSHIFT_NONE;
    hadc1.Init.OversamplingMode      = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
        Error_Handler();

    /* Channel config: ch16=PA0 rank1, ch17=PA1 rank2 */
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = ADC_CHANNEL_16;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;
    sConfig.OffsetSignedSaturation = DISABLE;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        Error_Handler();

    sConfig.Channel = ADC_CHANNEL_17;
    sConfig.Rank    = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        Error_Handler();

    /* DMA1_Stream3: peripheral-to-memory, halfword, normal mode, NDTR=CALIB_ADC_BUFFER_SIZE */
    hdma_adc1_calib.Instance                 = DMA1_Stream3;
    hdma_adc1_calib.Init.Request             = DMA_REQUEST_ADC1;
    hdma_adc1_calib.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc1_calib.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc1_calib.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc1_calib.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1_calib.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1_calib.Init.Mode                = DMA_NORMAL;
    hdma_adc1_calib.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_adc1_calib.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    hdma_adc1_calib.Init.MemBurst            = DMA_MBURST_SINGLE;
    hdma_adc1_calib.Init.PeriphBurst         = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_adc1_calib) != HAL_OK)
        Error_Handler();

    HAL_DMA_RegisterCallback(&hdma_adc1_calib, HAL_DMA_XFER_CPLT_CB_ID, Calib_ADC_DMA_Complete);

    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1_calib);
}

void Calib_ADC_Deinit(void)
{
    extern ADC_HandleTypeDef hadc1;

    HAL_ADC_Stop_DMA(&hadc1);
    HAL_DMA_Abort(&hdma_adc1_calib);
    HAL_ADC_DeInit(&hadc1);

    /* Restore ADC1 to polling mode (software trigger, 1 channel, 810.5 cycles).
     * Replicates MX_ADC1_Init from main.c (single rank on ch17). */
    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;
    hadc1.Init.Resolution            = ADC_RESOLUTION_16B;
    hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait      = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    hadc1.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.LeftBitShift          = ADC_LEFTBITSHIFT_NONE;
    hadc1.Init.OversamplingMode      = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
        Error_Handler();

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = ADC_CHANNEL_17;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;
    sConfig.OffsetSignedSaturation = DISABLE;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        Error_Handler();
}

static void Calib_ADC_DMA_Complete(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    adc_capture_done = 1;
}

/* ---- IQ Demodulation ---- */
float Calib_IQ_Demodulate(uint16_t *buf, uint32_t sample_count, float *amplitude_out)
{
    float I = 0.0f, Q = 0.0f;
    uint32_t pair_count = sample_count / 2U;

    for (uint32_t k = 0; k < pair_count; k++) {
        int32_t v0 = (int32_t)buf[k * 2U];
        int32_t v1 = (int32_t)buf[k * 2U + 1U];
        float diff = (float)(v0 - v1);
        uint32_t idx = k % 10U;
        I += diff * cos_lut[idx];
        Q += diff * sin_lut[idx];
    }

    float n = (float)pair_count;
    I /= n;
    Q /= n;

    float mag = sqrtf(I * I + Q * Q);
    if (amplitude_out) *amplitude_out = mag;

    float measured = atan2f(Q, I);
    float calib_phase = fmodf(-measured + 2.0f * (float)M_PI, 2.0f * (float)M_PI);
    return calib_phase;
}

float Calib_PhaseToMicroseconds(float phase_rad)
{
    return phase_rad / (2.0f * (float)M_PI) * TRANSDUCER_PERIOD_US;
}
