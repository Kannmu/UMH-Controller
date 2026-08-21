# pragma once
# include "main.h"
# include "transducer.h"

# define DMA_CHANNELS 5
# define _USE_MATH_DEFINES

// DMA Sampling Frequency
# define DMA_SAMPLING_FREQ 4000000UL

# define WAVEFORM_BUFFER_SIZE ((uint32_t)(DMA_SAMPLING_FREQ/TRANSDUCER_BASE_FREQ))
# define MAIN_WAVE_LENGTH_IN_BUFFER (WAVEFORM_BUFFER_SIZE)

# define TIME_GAP_PER_DMA_BUFFER_BIT ((long double)(1.0/(DMA_SAMPLING_FREQ)))

#define DMA_STRENGTH_MIN 0.0f
#define DMA_STRENGTH_MAX 100.0f
#define DMA_DUTY_CYCLE_MIN 0.0f
#define DMA_DUTY_CYCLE_MAX 0.5f

// dma_manager.h
extern const float GPIO_Group_Output_Offset[DMA_CHANNELS];

extern DMA_HandleTypeDef* DMA_Stream_Handles[DMA_CHANNELS];

typedef uint16_t DMA_WaveformBlock[DMA_CHANNELS][NUM_STIMULATION_SAMPLES][WAVEFORM_BUFFER_SIZE];

__ALIGNED(32) extern DMA_WaveformBlock Waveform_Storage __attribute__((section(".storage_buffer")));

extern const uint16_t BufferGapPerMicroseconds;

void DMA_Init();
void Start_DMAs();
void Update_Full_Waveform_Buffer(void);
void Clean_DMABuffer();
void DMA_Update_LED_State(uint16_t led_mask);
float DMA_Clamp_Stimulation_Strength(float strength);
uint16_t DMA_Convert_Strength_To_On_Ticks(float strength);

DMA_WaveformBlock *DMA_Sequence_Get_Block(uint8_t block);
void DMA_Sequence_Clean_Block(uint8_t block);
int DMA_Sequence_Start(void);
void DMA_Sequence_Stop(void);
int DMA_Sequence_Take_Free_Block(uint8_t *block);
void DMA_Sequence_Release_Block(uint8_t block);
uint32_t DMA_Sequence_Take_Deadline_Misses(void);
int DMA_Is_Sequence_Active(void);
