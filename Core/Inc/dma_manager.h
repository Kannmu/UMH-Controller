# pragma once
# include "main.h"
# include "transducer.h"

# define DMA_CHANNELS 4   /* B / C / D / E */

/* ---- Per-port DMA configuration ---- */
typedef struct {
    uint8_t            port_index;
    char               port_letter;
    GPIO_TypeDef      *gpio;
    DMA_HandleTypeDef *dma_handle;
    DMA_Stream_TypeDef *dma_instance;
    uint32_t           dma_request;
    uint32_t           tim_channel;
} PortDMAConfig;

extern const PortDMAConfig port_dma_configs[DMA_CHANNELS];

# define DMA_SAMPLING_FREQ 4000000UL
# define WAVEFORM_BUFFER_SIZE ((uint32_t)(DMA_SAMPLING_FREQ/TRANSDUCER_BASE_FREQ))
/* WAVEFORM_BUFFER_SIZE = 100 = one 40kHz cycle at 4MHz DMA rate.
 * NUM_STIMULATION_SAMPLES = 200 = 200Hz AM envelope (200 cycles per 5ms period).
 * Buffer layout: Waveform_Storage[port][cycle][tick] — 4×200×100×2 = 160KB.
 * Placed in RAM_D2 (.storage_buffer) for zero-latency DMA access (DMA1/DMA2
 * are on the D2 domain; D1 would add bus-matrix traversal latency at 16M
 * transactions/sec). NOLOAD: skip boot-time zeroing; Clean_DMABuffer() handles it.
 * Static modes (Point/TwinTrap): compute cycle 0 only, memcpy to 1..199 —
 * ~200× speedup vs full recomputation. */

# define MAIN_WAVE_LENGTH_IN_BUFFER (WAVEFORM_BUFFER_SIZE)

# define TIME_GAP_PER_DMA_BUFFER_BIT ((long double)(1.0/(DMA_SAMPLING_FREQ)))

#define DMA_STRENGTH_MIN 0.0f
#define DMA_STRENGTH_MAX 100.0f
#define DMA_DUTY_CYCLE_MIN 0.0f
#define DMA_DUTY_CYCLE_MAX 0.5f

#define TRIGGER0_DEFAULT_PULSE_US 1000U

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

void Calib_SetSingleTransducer(uint8_t idx);
void Calib_SetNormalDrive(void);
