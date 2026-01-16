#define _USE_MATH_DEFINES
#include "dma_manager.h"
#include "utiles.h"
#include "calibration.h"
#include "stimulation.h"

const float GPIO_Group_Output_Offset[DMA_CHANNELS] = {0U, 0.06, 0.09, 0.16, 0.12};

const uint16_t half_period = WAVEFORM_BUFFER_SIZE / 2;

const uint16_t BufferGapPerMicroseconds = ((float)(1e-6) / TIME_GAP_PER_DMA_BUFFER_BIT);

DMA_HandleTypeDef *DMA_Stream_Handles[DMA_CHANNELS];

__ALIGNED(32)
uint16_t Waveform_Storage[DMA_CHANNELS][NUM_STIMULATION_SAMPLES][WAVEFORM_BUFFER_SIZE] __attribute__((section(".storage_buffer")));

extern TIM_HandleTypeDef htim1;

static uint16_t Group_Offset_Ticks[DMA_CHANNELS];

static Transducer *TransducersByPort[DMA_CHANNELS][NUM_TRANSDUCER];
static int TransducersByPortCount[DMA_CHANNELS];

void DMA_Init()
{
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

    Start_DMAs();
}

void Start_DMAs()
{
    uint32_t total_length = NUM_STIMULATION_SAMPLES * WAVEFORM_BUFFER_SIZE;
    
    // Output Registers for each channel
    uint32_t *dest_addrs[DMA_CHANNELS] = {
        (uint32_t *)(&(GPIOA->ODR)),
        (uint32_t *)(&(GPIOB->ODR)),
        (uint32_t *)(&(GPIOC->ODR)),
        (uint32_t *)(&(GPIOD->ODR)),
        (uint32_t *)(&(GPIOE->ODR))
    };

    for (int i = 0; i < DMA_CHANNELS; i++)
    {
        // Ensure Circular Mode is enabled for continuous playback
        DMA_Stream_Handles[i]->Init.Mode = DMA_CIRCULAR;
        
        // Re-initialize the DMA with the new mode
        if (HAL_DMA_Init(DMA_Stream_Handles[i]) != HAL_OK)
        {
             Error_Handler();
        }

        // Start DMA in Circular Mode directly from Storage Buffer
        if (HAL_DMA_Start(DMA_Stream_Handles[i], 
                      (uint32_t)&Waveform_Storage[i][0][0], 
                      (uint32_t)dest_addrs[i], 
                      total_length) != HAL_OK)
        {
            Error_Handler();
        }
    }

    // Enable TIM1 DMA triggers
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC1);
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC2);
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC3);
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC4);

    // Start TIM1 Output Compare channels
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);

    // Start TIM1
    HAL_TIM_Base_Start(&htim1);
}

void Update_Full_Waveform_Buffer()
{
    // Temporary arrays for Channel-Slice processing
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
        // Note: With single buffer circular mode, this mask is fixed at generation time.
        // Dynamic blinking based on 'led0_ticks' during playback is not supported 
        // without re-generating the buffer or using a separate mechanism.
        uint16_t led_mask = Get_Current_LED_Mask();
        
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
}

void DMA_Update_LED_State(uint16_t led_mask)
{
    // LED Pins on Port A (Channel 0)
    // LED0: PA10, LED1: PA9, LED2: PA8
    const uint16_t LED_MASK_BITS = LED0_Pin | LED1_Pin | LED2_Pin;
    
    // We only touch Channel 0 (Port A)
    // Waveform_Storage is [DMA_CHANNELS][NUM_STIMULATION_SAMPLES][WAVEFORM_BUFFER_SIZE]
    // Accessing Channel 0
    
    // Optimize: Pre-calculate the masked value
    // Note: If the bit in led_mask is 1, it means LED OFF (Active Low)
    // If the bit in led_mask is 0, it means LED ON
    uint16_t led_bits = led_mask & LED_MASK_BITS;

    // Iterate over all samples for Channel 0
    for (int s = 0; s < NUM_STIMULATION_SAMPLES; s++)
    {
        uint16_t *buffer_ptr = Waveform_Storage[0][s];
        for (int i = 0; i < WAVEFORM_BUFFER_SIZE; i++)
        {
            // Read-Modify-Write
            // Preserve other bits (Transducers), replace LED bits
            buffer_ptr[i] = (buffer_ptr[i] & ~LED_MASK_BITS) | led_bits;
        }
    }
}
