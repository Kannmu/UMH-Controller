#define _USE_MATH_DEFINES
#include "dma_manager.h"
#include "utiles.h"
#include "calibration.h"
#include "stimulation.h"

extern int led0_state;

const float GPIO_Group_Output_Offset[DMA_CHANNELS] = {0U, 3, 6.5, 9, 11.5};
// const float GPIO_Group_Output_Offset[DMA_CHANNELS] = {0U, 0U, 0U, 0U, 0U};

const uint32_t BufferResolution = DMA_Buffer_Resolution;

const uint16_t half_period = DMA_Buffer_Resolution / 2;

const uint16_t BufferGapPerMicroseconds = ((float)(1e-6)/TimeGapPerDMABufferBit);;

const uint16_t preserved_pins_mask = KEY0_Pin | KEY1_Pin | LED2_Pin | LED1_Pin | LED0_Pin;

DMA_HandleTypeDef* DMA_Stream_Handles[DMA_CHANNELS];

__ALIGNED(32) uint16_t DMA_Buffer[DMA_CHANNELS][DMA_Buffer_Resolution] __attribute__((section(".dma")));

extern TIM_HandleTypeDef htim1;

void DMA_Init()
{
    DMA_Stream_Handles[0] = &hdma_memtomem_dma1_stream0;
    DMA_Stream_Handles[1] = &hdma_memtomem_dma1_stream1;
    DMA_Stream_Handles[2] = &hdma_memtomem_dma1_stream2;
    DMA_Stream_Handles[3] = &hdma_memtomem_dma2_stream0;
    DMA_Stream_Handles[4] = &hdma_memtomem_dma2_stream1;
    Clean_DMABuffer();
    Start_DMAs();
}

void Start_DMAs()
{
    HAL_DMA_Start(&hdma_memtomem_dma1_stream0, (uint32_t)(DMA_Buffer[0]), (uint32_t)(&(GPIOA->ODR)), sizeof(DMA_Buffer[0]) / sizeof(DMA_Buffer[0][0]));
    HAL_DMA_Start(&hdma_memtomem_dma1_stream1, (uint32_t)(DMA_Buffer[1]), (uint32_t)(&(GPIOB->ODR)), sizeof(DMA_Buffer[1]) / sizeof(DMA_Buffer[1][0]));
    HAL_DMA_Start(&hdma_memtomem_dma1_stream2, (uint32_t)(DMA_Buffer[2]), (uint32_t)(&(GPIOC->ODR)), sizeof(DMA_Buffer[2]) / sizeof(DMA_Buffer[2][0]));
    HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)(DMA_Buffer[3]), (uint32_t)(&(GPIOD->ODR)), sizeof(DMA_Buffer[3]) / sizeof(DMA_Buffer[3][0]));
    HAL_DMA_Start(&hdma_memtomem_dma2_stream1, (uint32_t)(DMA_Buffer[4]), (uint32_t)(&(GPIOE->ODR)), sizeof(DMA_Buffer[4]) / sizeof(DMA_Buffer[4][0]));

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

void Update_All_DMABuffer()
{
    uint32_t start_cyc = DWT->CYCCNT;

    // Optimization: Use Sweep-Line algorithm to reduce complexity from O(N*M) to O(N+M).
    // This avoids nested loops and global buffer clears, ensuring high performance.

    // 1. Define Event Structure for Sweep-Line
    typedef struct {
        uint8_t port_num;
        uint8_t pin_index; // 0-15
        int8_t change;     // +1 (Start) or -1 (End)
        int16_t next_event_idx; // Linked list next pointer
    } EventNode;

    // 2. Prepare Data Structures
    // Event pool: Max 2 events (Start, End) per transducer.
    EventNode event_pool[NumTransducer * 2];
    uint16_t event_pool_count = 0;

    // Timeline buckets: Head index of event list for each buffer position
    // Initialize to -1 (No events)
    int16_t timeline[DMA_Buffer_Resolution];
    for (uint32_t i = 0; i < DMA_Buffer_Resolution; i++) {
        timeline[i] = -1;
    }

    // Active counters (handle overlaps) and current state
    uint16_t pin_active_counts[DMA_CHANNELS][16] = {0}; 
    uint16_t port_states[DMA_CHANNELS] = {0};

    // 3. Generate Events from Transducers
    for (size_t i = 0; i < NumTransducer; i++)
    {
        Transducer *t = &TransducerArray[i];
        
        // Calculate Phase & Indices
        uint16_t phase_offset = t->calib + t->shift_buffer_bits;
        phase_offset += (uint16_t)(GPIO_Group_Output_Offset[t->port_num] * BufferGapPerMicroseconds);
        phase_offset %= BufferResolution;
        
        uint32_t start_idx = (BufferResolution - phase_offset) % BufferResolution;
        uint32_t end_idx = start_idx + half_period;
        
        uint8_t p_num = t->port_num;
        uint8_t p_idx = __builtin_ctz(t->pin); // Get bit position (0-15)

        // Handle Wrap-around logic
        if (end_idx > BufferResolution) {
            // Wrapped: Active at index 0
            pin_active_counts[p_num][p_idx]++;
            port_states[p_num] |= (1 << p_idx);
            
            // Event: End at (end_idx % BufferResolution)
            uint32_t real_end = end_idx % BufferResolution;
            if (real_end < BufferResolution) {
                int idx = event_pool_count++;
                event_pool[idx].port_num = p_num;
                event_pool[idx].pin_index = p_idx;
                event_pool[idx].change = -1;
                event_pool[idx].next_event_idx = timeline[real_end];
                timeline[real_end] = idx;
            }
            
            // Event: Start at start_idx
            if (start_idx < BufferResolution) {
                int idx = event_pool_count++;
                event_pool[idx].port_num = p_num;
                event_pool[idx].pin_index = p_idx;
                event_pool[idx].change = 1;
                event_pool[idx].next_event_idx = timeline[start_idx];
                timeline[start_idx] = idx;
            }
        } else {
            // Normal
            if (start_idx < BufferResolution) {
                int idx = event_pool_count++;
                event_pool[idx].port_num = p_num;
                event_pool[idx].pin_index = p_idx;
                event_pool[idx].change = 1;
                event_pool[idx].next_event_idx = timeline[start_idx];
                timeline[start_idx] = idx;
            }
            if (end_idx < BufferResolution) {
                int idx = event_pool_count++;
                event_pool[idx].port_num = p_num;
                event_pool[idx].pin_index = p_idx;
                event_pool[idx].change = -1;
                event_pool[idx].next_event_idx = timeline[end_idx];
                timeline[end_idx] = idx;
            }
        }
    }

    // 4. Prepare LED Base State (Port 0)
    uint16_t led_mask = 0;
    if (!led0_state) led_mask |= LED0_Pin;
    if (!Get_Calibration_Mode()) led_mask |= LED1_Pin;
    if (!Get_Plane_Mode()) led_mask |= LED2_Pin;

    // 5. Process Timeline and Fill Buffer
    for (uint32_t i = 0; i < BufferResolution; i++)
    {
        // Process events at this time step
        int16_t e_idx = timeline[i];
        while (e_idx != -1)
        {
            EventNode *e = &event_pool[e_idx];
            uint8_t p = e->port_num;
            uint8_t bit = e->pin_index;
            
            pin_active_counts[p][bit] += e->change;
            
            if (pin_active_counts[p][bit] > 0) {
                port_states[p] |= (1 << bit);
            } else {
                port_states[p] &= ~(1 << bit);
            }
            
            e_idx = e->next_event_idx;
        }

        // Write Output
        DMA_Buffer[0][i] = port_states[0] | led_mask;
        for (int p = 1; p < DMA_CHANNELS; p++) {
            DMA_Buffer[p][i] = port_states[p];
        }
    }

    uint32_t end_cyc = DWT->CYCCNT;
    updateDMABufferDeltaTime = (double)((double)(end_cyc - start_cyc) / (double)SystemCoreClock);
}

void Clean_DMABuffer()
{
    memset(DMA_Buffer, 0x0000, sizeof(DMA_Buffer));
}
