#define _USE_MATH_DEFINES
#include "dma_manager.h"
#include "utiles.h"
#include "calibration.h"
#include "stimulation.h"

extern int led0_state;

const float GPIO_Group_Output_Offset[DMA_CHANNELS] = {0U, 0.06, 0.09, 0.16, 0.12};

const uint16_t half_period = WAVEFORM_BUFFER_SIZE / 2;

const uint16_t BufferGapPerMicroseconds = ((float)(1e-6) / TIME_GAP_PER_DMA_BUFFER_BIT);


DMA_HandleTypeDef *DMA_Stream_Handles[DMA_CHANNELS];

__ALIGNED(32)
uint16_t Waveform_Storage[DMA_CHANNELS][NUM_STIMULATION_SAMPLES][WAVEFORM_BUFFER_SIZE] __attribute__((section(".ram_d1_data")));

__ALIGNED(32)
uint16_t Waveform_Play_Buffer[DMA_CHANNELS][PLAY_BUFFER_SAMPLES][WAVEFORM_BUFFER_SIZE] __attribute__((section(".dma")));

MDMA_HandleTypeDef hmdma[DMA_CHANNELS];

extern TIM_HandleTypeDef htim1;

static uint16_t Group_Offset_Ticks[DMA_CHANNELS];

static Transducer *TransducersByPort[DMA_CHANNELS][NUM_TRANSDUCER];
static int TransducersByPortCount[DMA_CHANNELS];

static void MDMA_Config(void);
static void DMA_HalfTransferComplete(DMA_HandleTypeDef *hdma);
static void DMA_TransferComplete(DMA_HandleTypeDef *hdma);

volatile uint32_t storage_load_index = 0;

void DMA_Init()
{
    MDMA_Config();

    DMA_Stream_Handles[0] = &hdma_memtomem_dma1_stream0;
    DMA_Stream_Handles[1] = &hdma_memtomem_dma1_stream1;
    DMA_Stream_Handles[2] = &hdma_memtomem_dma1_stream2;
    DMA_Stream_Handles[3] = &hdma_memtomem_dma2_stream0;
    DMA_Stream_Handles[4] = &hdma_memtomem_dma2_stream1;

    for (int i = 0; i < DMA_CHANNELS; i++)
    {
        Group_Offset_Ticks[i] = (uint16_t)(GPIO_Group_Output_Offset[i] * BufferGapPerMicroseconds);
    }

    // Build Port-Transducer Map
    memset(TransducersByPortCount, 0, sizeof(TransducersByPortCount));
    for (size_t i = 0; i < NUM_TRANSDUCER; i++)
    {
        Transducer *t = &TransducerArray[i];
        if (t->port_num < DMA_CHANNELS)
        {
            TransducersByPort[t->port_num][TransducersByPortCount[t->port_num]++] = t;
        }
    }

    Clean_DMABuffer();
    Update_Full_Waveform_Buffer();

    // Initial Fill of Play Buffer (First PLAY_BUFFER_SAMPLES)
    // Storage has NUM_STIMULATION_SAMPLES (200). Play Buffer has PLAY_BUFFER_SAMPLES (100).
    // Copy first 100 samples.
    for (int i = 0; i < DMA_CHANNELS; i++)
    {
        HAL_MDMA_Start(&hmdma[i], 
                       (uint32_t)&Waveform_Storage[i][0][0], 
                       (uint32_t)&Waveform_Play_Buffer[i][0][0], 
                       PLAY_BUFFER_SAMPLES * WAVEFORM_BUFFER_SIZE * 2,
                       1);
        HAL_MDMA_PollForTransfer(&hmdma[i], HAL_MDMA_FULL_TRANSFER, 100);
    }
    
    // Initialize storage_load_index for next load
    storage_load_index = PLAY_BUFFER_SAMPLES; 
    if (storage_load_index >= NUM_STIMULATION_SAMPLES) storage_load_index = 0;

    Start_DMAs();
}

static void MDMA_Config(void)
{
    __HAL_RCC_MDMA_CLK_ENABLE();

    for (int i = 0; i < DMA_CHANNELS; i++)
    {
        hmdma[i].Instance = (MDMA_Channel_TypeDef *)(MDMA_Channel0_BASE + (i * 0x40));
        hmdma[i].Init.Request = MDMA_REQUEST_SW;
        hmdma[i].Init.TransferTriggerMode = MDMA_BUFFER_TRANSFER;
        hmdma[i].Init.Priority = MDMA_PRIORITY_HIGH;
        hmdma[i].Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
        hmdma[i].Init.SourceInc = MDMA_SRC_INC_HALFWORD;
        hmdma[i].Init.DestinationInc = MDMA_DEST_INC_HALFWORD;
        hmdma[i].Init.SourceDataSize = MDMA_SRC_DATASIZE_HALFWORD;
        hmdma[i].Init.DestDataSize = MDMA_DEST_DATASIZE_HALFWORD;
        hmdma[i].Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
        hmdma[i].Init.BufferTransferLength = PLAY_BUFFER_HALF_SAMPLES * WAVEFORM_BUFFER_SIZE * 2; // Default length
        hmdma[i].Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
        hmdma[i].Init.DestBurst = MDMA_DEST_BURST_SINGLE;
        hmdma[i].Init.SourceBlockAddressOffset = 0;
        hmdma[i].Init.DestBlockAddressOffset = 0;

        if (HAL_MDMA_Init(&hmdma[i]) != HAL_OK)
        {
            Error_Handler();
        }
    }
}

void Start_DMAs()
{
    uint32_t total_length = PLAY_BUFFER_SAMPLES * WAVEFORM_BUFFER_SIZE;

    // Register Callbacks for Stream 0
    hdma_memtomem_dma1_stream0.XferHalfCpltCallback = DMA_HalfTransferComplete;
    hdma_memtomem_dma1_stream0.XferCpltCallback = DMA_TransferComplete;

    // Stream 0: Start with IT
    HAL_DMA_Start_IT(&hdma_memtomem_dma1_stream0, (uint32_t)(Waveform_Play_Buffer[0]), (uint32_t)(&(GPIOA->ODR)), total_length);
    
    // Enable NVIC for Stream 0
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

    // Others: Start Normal
    HAL_DMA_Start(&hdma_memtomem_dma1_stream1, (uint32_t)(Waveform_Play_Buffer[1]), (uint32_t)(&(GPIOB->ODR)), total_length);
    HAL_DMA_Start(&hdma_memtomem_dma1_stream2, (uint32_t)(Waveform_Play_Buffer[2]), (uint32_t)(&(GPIOC->ODR)), total_length);
    HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)(Waveform_Play_Buffer[3]), (uint32_t)(&(GPIOD->ODR)), total_length);
    HAL_DMA_Start(&hdma_memtomem_dma2_stream1, (uint32_t)(Waveform_Play_Buffer[4]), (uint32_t)(&(GPIOE->ODR)), total_length);

    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC1);
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC2);
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC3);
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC4);

    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIM_Base_Start(&htim1);
}

static void DMA_HalfTransferComplete(DMA_HandleTypeDef *hdma)
{
    // First Half Finished Reading. We can overwrite First Half.
    // Copy NEXT chunk from Storage to Play_Buffer[0..HALF-1]
    
    for (int i = 0; i < DMA_CHANNELS; i++)
    {
        HAL_MDMA_Start(&hmdma[i], 
                       (uint32_t)&Waveform_Storage[i][storage_load_index][0], 
                       (uint32_t)&Waveform_Play_Buffer[i][0][0], 
                       PLAY_BUFFER_HALF_SAMPLES * WAVEFORM_BUFFER_SIZE * 2,
                       1);
    }

    storage_load_index += PLAY_BUFFER_HALF_SAMPLES;
    if (storage_load_index >= NUM_STIMULATION_SAMPLES) storage_load_index = 0;
}

static void DMA_TransferComplete(DMA_HandleTypeDef *hdma)
{
    // Second Half Finished Reading. We can overwrite Second Half.
    // Copy NEXT chunk from Storage to Play_Buffer[HALF..FULL-1]

    for (int i = 0; i < DMA_CHANNELS; i++)
    {
        HAL_MDMA_Start(&hmdma[i], 
                       (uint32_t)&Waveform_Storage[i][storage_load_index][0], 
                       (uint32_t)&Waveform_Play_Buffer[i][PLAY_BUFFER_HALF_SAMPLES][0], 
                       PLAY_BUFFER_HALF_SAMPLES * WAVEFORM_BUFFER_SIZE * 2,
                       1);
    }

    storage_load_index += PLAY_BUFFER_HALF_SAMPLES;
    if (storage_load_index >= NUM_STIMULATION_SAMPLES) storage_load_index = 0;
}

void DMA1_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_memtomem_dma1_stream0);
}

void Update_Full_Waveform_Buffer()
{
    // Temporary arrays for Channel-Slice processing
    // Defined static or on stack? Stack is safer for reentrancy but this is large.
    // 200 * 2 = 400 bytes. Stack is fine.
    uint16_t turn_on[WAVEFORM_BUFFER_SIZE];
    uint16_t turn_off[WAVEFORM_BUFFER_SIZE];

    // Event indices array
    uint16_t event_indices[NUM_TRANSDUCER * 2 + 2];
    int event_count;

    for (int s = 0; s < NUM_STIMULATION_SAMPLES; s++)
    {
        float progress = (float)s / (float)NUM_STIMULATION_SAMPLES;
        
        // Update Transducer State for this time slice
        Update_Stimulation_State(progress);

        // Pre-calculate LED Mask (Port 0)
        uint16_t led_mask = 0;
        if (!led0_state)
            led_mask |= LED0_Pin;
        if (!Get_Calibration_Mode())
            led_mask |= LED1_Pin;
        if (!Get_Phase_Set_Mode())
            led_mask |= LED2_Pin;

        // Channel-Slice Loop
        for (int p = 0; p < DMA_CHANNELS; p++)
        {
            // 1. Reset Event Arrays
            memset(turn_on, 0, sizeof(turn_on));
            memset(turn_off, 0, sizeof(turn_off));

            event_count = 0;
            event_indices[event_count++] = 0;

            uint16_t current_state = 0;

            // 2. Process Transducers for this Channel
            int count = TransducersByPortCount[p];
            for (int k = 0; k < count; k++)
            {
                Transducer *t = TransducersByPort[p][k];

                // Phase Calculation
                uint16_t phase_offset = t->calib + t->shift_buffer_bits;
                phase_offset += Group_Offset_Ticks[p];
                phase_offset %= WAVEFORM_BUFFER_SIZE;

                uint32_t start_idx = (WAVEFORM_BUFFER_SIZE - phase_offset) % WAVEFORM_BUFFER_SIZE;
                uint32_t end_idx = (start_idx + half_period) % WAVEFORM_BUFFER_SIZE;
                uint16_t pin_bit = (1 << __builtin_ctz(t->pin));

                // Record Events
                if (turn_on[start_idx] == 0 && turn_off[start_idx] == 0)
                    event_indices[event_count++] = (uint16_t)start_idx;
                turn_on[start_idx] |= pin_bit;

                if (turn_on[end_idx] == 0 && turn_off[end_idx] == 0)
                    event_indices[event_count++] = (uint16_t)end_idx;
                turn_off[end_idx] |= pin_bit;

                // Handle Wrap-around Initial State
                if (start_idx >= end_idx)
                {
                    current_state |= pin_bit;
                }
            }

            // Sort event_indices (Insertion Sort)
            for (int i = 1; i < event_count; i++)
            {
                uint16_t key = event_indices[i];
                int j = i - 1;
                while (j >= 0 && event_indices[j] > key)
                {
                    event_indices[j + 1] = event_indices[j];
                    j = j - 1;
                }
                event_indices[j + 1] = key;
            }

            // 3. Fill Buffer (Run-Length Encoded)
            uint16_t *buffer_ptr_base = &Waveform_Storage[p][s][0];

            uint16_t running_state = current_state;
            if (p == 0)
                running_state |= led_mask; // Apply initial LED mask

            // Previous event index
            int prev_idx = 0;

            // Process sorted unique indices
            for (int i = 0; i < event_count; i++)
            {
                int idx = event_indices[i];

                // Skip duplicates
                if (i > 0 && idx == event_indices[i - 1])
                    continue;

                // Fill gap from prev_idx to idx
                int gap_count = idx - prev_idx;
                if (gap_count > 0)
                {
                    uint16_t *ptr = &buffer_ptr_base[prev_idx];
                    while (gap_count >= 4)
                    {
                        ptr[0] = running_state;
                        ptr[1] = running_state;
                        ptr[2] = running_state;
                        ptr[3] = running_state;
                        ptr += 4;
                        gap_count -= 4;
                    }
                    while (gap_count-- > 0)
                    {
                        *ptr++ = running_state;
                    }
                }

                // Update state AT idx
                if (idx < WAVEFORM_BUFFER_SIZE)
                {
                    running_state |= turn_on[idx];
                    running_state &= ~turn_off[idx];
                    if (p == 0)
                        running_state |= led_mask;
                }

                prev_idx = idx;
            }

            // Fill remaining tail
            int tail_count = WAVEFORM_BUFFER_SIZE - prev_idx;
            if (tail_count > 0)
            {
                uint16_t *ptr = &buffer_ptr_base[prev_idx];
                while (tail_count >= 4)
                {
                    ptr[0] = running_state;
                    ptr[1] = running_state;
                    ptr[2] = running_state;
                    ptr[3] = running_state;
                    ptr += 4;
                    tail_count -= 4;
                }
                while (tail_count-- > 0)
                {
                    *ptr++ = running_state;
                }
            }
        }
    }
}

void Clean_DMABuffer()
{
    memset(Waveform_Storage, 0x0000, sizeof(Waveform_Storage));
    memset(Waveform_Play_Buffer, 0x0000, sizeof(Waveform_Play_Buffer));
}
