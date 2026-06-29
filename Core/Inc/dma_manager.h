# pragma once
# include "main.h"
# include "transducer.h"

# define DMA_CHANNELS 4   /* B / C / D / E — no transducers on GPIOA */

# define DMA_SAMPLING_FREQ 4000000UL
# define WAVEFORM_BUFFER_SIZE ((uint32_t)(DMA_SAMPLING_FREQ/TRANSDUCER_BASE_FREQ))
# define MAIN_WAVE_LENGTH_IN_BUFFER (WAVEFORM_BUFFER_SIZE)

# define TIME_GAP_PER_DMA_BUFFER_BIT ((long double)(1.0/(DMA_SAMPLING_FREQ)))

#define DMA_STRENGTH_MIN 0.0f
#define DMA_STRENGTH_MAX 100.0f
#define DMA_DUTY_CYCLE_MIN 0.0f
#define DMA_DUTY_CYCLE_MAX 0.5f

extern DMA_HandleTypeDef* DMA_Stream_Handles[DMA_CHANNELS];

__ALIGNED(32) extern uint16_t Waveform_Storage[DMA_CHANNELS][NUM_STIMULATION_SAMPLES][WAVEFORM_BUFFER_SIZE] __attribute__((section(".storage_buffer")));

extern const uint16_t BufferGapPerMicroseconds;

void DMA_Init();
void Start_DMAs();
void Update_Full_Waveform_Buffer(void);
void Clean_DMABuffer();
float DMA_Clamp_Stimulation_Strength(float strength);
uint16_t DMA_Convert_Strength_To_On_Ticks(float strength);

void Configure_Trigger0(uint8_t enable, uint32_t pulse_us);
void Configure_Trigger1(uint8_t enable);
