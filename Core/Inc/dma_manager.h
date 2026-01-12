# pragma once
# include "main.h"
# include "transducer.h"

# define DMA_CHANNELS 5
# define _USE_MATH_DEFINES

// DMA Sampling Frequency
# define DMA_SAMPLING_FREQ 8000000UL

# define WAVEFORM_BUFFER_SIZE ((uint32_t)(DMA_SAMPLING_FREQ/TRANSDUCER_BASE_FREQ))
# define MAIN_WAVE_LENGTH_IN_BUFFER (WAVEFORM_BUFFER_SIZE)

# define TIME_GAP_PER_DMA_BUFFER_BIT ((long double)(1.0/(DMA_SAMPLING_FREQ)))

// dma_manager.h
extern const float GPIO_Group_Output_Offset[DMA_CHANNELS];

extern DMA_HandleTypeDef* DMA_Stream_Handles[DMA_CHANNELS];

__ALIGNED(32) extern uint16_t Waveform_Buffer[DMA_CHANNELS][NUM_STIMULATION_SAMPLES][WAVEFORM_BUFFER_SIZE] __attribute__((section(".dma")));

// extern long double TimeGapPerDMABufferBit;
extern const uint16_t BufferGapPerMicroseconds;

void DMA_Init();
void Start_DMAs();
void Update_Full_Waveform_Buffer(void);
void Clean_DMABuffer();
