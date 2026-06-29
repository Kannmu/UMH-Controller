#pragma once
#include "main.h"

extern TIM_HandleTypeDef htim6;
extern DMA_HandleTypeDef hdma_adc1_calib;
extern volatile uint8_t adc_capture_done;
extern uint16_t adc_buffer[8000];

void MX_TIM6_Init(void);
void Calib_ADC_Configure(void);
void Calib_ADC_Deinit(void);
float Calib_IQ_Demodulate(uint16_t *buf, uint32_t sample_count, float *amplitude_out);
float Calib_PhaseToMicroseconds(float phase_rad);
