#define _USE_MATH_DEFINES
#include "dma_manager.h"
#include "utiles.h"
#include "calibration.h"
#include "stimulation.h"

extern int led0_state;

const float GPIO_Group_Output_Offset[DMA_CHANNELS] = {0U, 0.06, 0.09, 0.16, 0.12};
// const float GPIO_Group_Output_Offset[DMA_CHANNELS] = {0U, 0U, 0U, 0U, 0U};

const uint16_t half_period = WAVEFORM_BUFFER_SIZE / 2;

const uint16_t BufferGapPerMicroseconds = ((float)(1e-6) / TimeGapPerDMABufferBit);


DMA_HandleTypeDef *DMA_Stream_Handles[DMA_CHANNELS];

__ALIGNED(32)
uint16_t Waveform_Buffer[DMA_CHANNELS][WAVEFORM_FULL_BUFFER_SIZE] __attribute__((section(".dma")));

extern TIM_HandleTypeDef htim1;

static uint16_t Group_Offset_Ticks[DMA_CHANNELS];

static Transducer *TransducersByPort[DMA_CHANNELS][NumTransducer];
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
    for (size_t i = 0; i < NumTransducer; i++)
    {
        Transducer *t = &TransducerArray[i];
        if (t->port_num < DMA_CHANNELS)
        {
            TransducersByPort[t->port_num][TransducersByPortCount[t->port_num]++] = t;
        }
    }

    Clean_DMABuffer();
    Start_DMAs();
}

void Start_DMAs()
{
    HAL_DMA_Start(&hdma_memtomem_dma1_stream0, (uint32_t)(Waveform_Buffer[0]), (uint32_t)(&(GPIOA->ODR)), sizeof(Waveform_Buffer[0]) / sizeof(Waveform_Buffer[0][0]));
    HAL_DMA_Start(&hdma_memtomem_dma1_stream1, (uint32_t)(Waveform_Buffer[1]), (uint32_t)(&(GPIOB->ODR)), sizeof(Waveform_Buffer[1]) / sizeof(Waveform_Buffer[1][0]));
    HAL_DMA_Start(&hdma_memtomem_dma1_stream2, (uint32_t)(Waveform_Buffer[2]), (uint32_t)(&(GPIOC->ODR)), sizeof(Waveform_Buffer[2]) / sizeof(Waveform_Buffer[2][0]));
    HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)(Waveform_Buffer[3]), (uint32_t)(&(GPIOD->ODR)), sizeof(Waveform_Buffer[3]) / sizeof(Waveform_Buffer[3][0]));
    HAL_DMA_Start(&hdma_memtomem_dma2_stream1, (uint32_t)(Waveform_Buffer[4]), (uint32_t)(&(GPIOE->ODR)), sizeof(Waveform_Buffer[4]) / sizeof(Waveform_Buffer[4][0]));

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

void Update_All_DMABuffer(int force)
{
    // Double Buffering Logic
    // Monitor Stream 0 (Lead Stream). NDTR counts DOWN.
    uint32_t dma_cnt = __HAL_DMA_GET_COUNTER(&hdma_memtomem_dma1_stream0);
    uint32_t dma_pos = WAVEFORM_FULL_BUFFER_SIZE - dma_cnt;

    // Determine which half DMA is reading (0 or 1)
    int dma_half = (dma_pos < WAVEFORM_BUFFER_SIZE) ? 0 : 1;
    // CPU writes to the other half
    int cpu_half = 1 - dma_half;

    // Safety: Only update once per swap to prevent race conditions during write
    static int last_cpu_half = -1;
    if (!force && cpu_half == last_cpu_half)
    {
        return;
    }
    last_cpu_half = cpu_half;

    // Wait-for-All Logic: Ensure ALL streams are clear of cpu_half
    uint32_t wait_start = DWT->CYCCNT;
    const uint32_t timeout_cycles = SystemCoreClock / 1000; // 1ms

    for (int i = 0; i < DMA_CHANNELS; i++)
    {
        while (1)
        {
            if ((DWT->CYCCNT - wait_start) > timeout_cycles)
            {
                return;
            }

            uint32_t cnt = __HAL_DMA_GET_COUNTER(DMA_Stream_Handles[i]);
            uint32_t pos = WAVEFORM_FULL_BUFFER_SIZE - cnt;
            int current_half = (pos < WAVEFORM_BUFFER_SIZE) ? 0 : 1;

            // If any stream is currently reading the half we want to write to, wait.
            if (current_half != cpu_half)
            {
                break;
            }
        }
    }

    uint32_t start_cyc = DWT->CYCCNT;

    // Pre-calculate LED Mask (Port 0)
    uint16_t led_mask = 0;
    if (!led0_state)
        led_mask |= LED0_Pin;
    if (!Get_Calibration_Mode())
        led_mask |= LED1_Pin;
    if (!Get_Phase_Set_Mode())
        led_mask |= LED2_Pin;

    // Temporary arrays for Channel-Slice processing
    uint16_t turn_on[WAVEFORM_BUFFER_SIZE];
    uint16_t turn_off[WAVEFORM_BUFFER_SIZE];

    // Event indices array
    uint16_t event_indices[NumTransducer * 2 + 2];
    int event_count;

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
        int start_h = (force == 2) ? 0 : cpu_half;
        int end_h = (force == 2) ? 1 : cpu_half;

        for (int h = start_h; h <= end_h; h++)
        {
            uint32_t base_offset = h * WAVEFORM_BUFFER_SIZE;
            uint16_t *buffer_ptr_base = &Waveform_Buffer[p][base_offset];

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
                int count = idx - prev_idx;
                if (count > 0)
                {
                    uint16_t *ptr = &buffer_ptr_base[prev_idx];
                    while (count >= 4)
                    {
                        ptr[0] = running_state;
                        ptr[1] = running_state;
                        ptr[2] = running_state;
                        ptr[3] = running_state;
                        ptr += 4;
                        count -= 4;
                    }
                    while (count-- > 0)
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
            int count = WAVEFORM_BUFFER_SIZE - prev_idx;
            if (count > 0)
            {
                uint16_t *ptr = &buffer_ptr_base[prev_idx];
                while (count >= 4)
                {
                    ptr[0] = running_state;
                    ptr[1] = running_state;
                    ptr[2] = running_state;
                    ptr[3] = running_state;
                    ptr += 4;
                    count -= 4;
                }
                while (count-- > 0)
                {
                    *ptr++ = running_state;
                }
            }
        }
    }

    // Ensure data is written to RAM before DMA reads it
    __DSB();

    uint32_t end_cyc = DWT->CYCCNT;
    updateDMABufferDeltaTime = (double)((double)(end_cyc - start_cyc) / (double)SystemCoreClock);
}

void Clean_DMABuffer()
{
    memset(Waveform_Buffer, 0x0000, sizeof(Waveform_Buffer));
}
