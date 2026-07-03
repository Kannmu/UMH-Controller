#pragma once
#include "main.h"

/* adc_buffer must reside in the non-cacheable MPU region (RAM_D2,
 * 0x30000000-0x30080000, see SystemClock_Config MPU setup in main.c).
 * Calib_semi.c calls SCB_InvalidateDCache_by_Addr before reading it as a
 * defensive measure in case the MPU attribute is ever changed. */
#define CALIB_ADC_BUFFER_SIZE   8000U   /* ADC @ 400kHz, 2 conv/trigger (ch16+ch17) -> 4000 triggers -> 10ms capture */
#define CALIB_ADC_SAMPLING_FREQ 400000U

extern TIM_HandleTypeDef htim6;
extern DMA_HandleTypeDef hdma_adc1_calib;
extern volatile uint8_t adc_capture_done;
extern uint16_t adc_buffer[CALIB_ADC_BUFFER_SIZE];

void MX_TIM6_Init(void);
void Calib_ADC_Configure(void);
void Calib_ADC_Deinit(void);
float Calib_IQ_Demodulate(uint16_t *buf, uint32_t sample_count, float *amplitude_out);
float Calib_PhaseToMicroseconds(float phase_rad);
