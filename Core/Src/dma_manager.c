#define _USE_MATH_DEFINES
#include "dma_manager.h"
#include "utiles.h"
#include "calibration.h"
#include "stimulation.h"

extern int led0_state;

const float GPIO_Group_Output_Offset[DMA_CHANNELS] = {0U, 10.56, 5.66, 3.56, 11.02};
// const float GPIO_Group_Output_Offset[DMA_CHANNELS] = {0U, 0U, 0U, 0U, 0U};

const uint32_t BufferResolution = DMA_Buffer_Resolution;

const uint16_t half_period = DMA_Buffer_Resolution / 2;

const uint16_t BufferGapPerMicroseconds = ((float)(1e-6)/TimeGapPerDMABufferBit);;

const uint16_t preserved_pins_mask = KEY0_Pin | KEY1_Pin | LED2_Pin | LED1_Pin | LED0_Pin;

DMA_HandleTypeDef* DMA_Stream_Handles[DMA_CHANNELS];

__ALIGNED(32) uint16_t DMA_Buffer[DMA_CHANNELS][DMA_FULL_BUFFER_SIZE] __attribute__((section(".dma")));

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

void Update_All_DMABuffer(int force)
{
    // Double Buffering Logic
    // Monitor Stream 0 (Lead Stream). NDTR counts DOWN.
    uint32_t dma_cnt = __HAL_DMA_GET_COUNTER(&hdma_memtomem_dma1_stream0);
    uint32_t dma_pos = DMA_FULL_BUFFER_SIZE - dma_cnt;
    
    // Determine which half DMA is reading (0 or 1)
    int dma_half = (dma_pos < DMA_Buffer_Resolution) ? 0 : 1;
    // CPU writes to the other half
    int cpu_half = 1 - dma_half;
    
    // Safety: Only update once per swap to prevent race conditions during write
    static int last_cpu_half = -1;
    if (!force && cpu_half == last_cpu_half) {
        return; 
    }
    last_cpu_half = cpu_half;
    
    uint32_t start_cyc = DWT->CYCCNT;
    uint32_t offset = cpu_half * DMA_Buffer_Resolution;

    // Pre-calculate LED Mask (Port 0)
    uint16_t led_mask = 0;
    if (!led0_state) led_mask |= LED0_Pin;
    if (!Get_Calibration_Mode()) led_mask |= LED1_Pin;
    if (!Get_Plane_Mode()) led_mask |= LED2_Pin;

    // Temporary arrays for Channel-Slice processing
    uint16_t turn_on[DMA_Buffer_Resolution];
    uint16_t turn_off[DMA_Buffer_Resolution];

    // Channel-Slice Loop
    for (int p = 0; p < DMA_CHANNELS; p++)
    {
        // 1. Reset Event Arrays
        memset(turn_on, 0, sizeof(turn_on));
        memset(turn_off, 0, sizeof(turn_off));
        
        uint16_t current_state = 0;

        // 2. Process Transducers for this Channel
        for (size_t i = 0; i < NumTransducer; i++)
        {
            Transducer *t = &TransducerArray[i];
            if (t->port_num != p) continue;

            // Phase Calculation
            uint16_t phase_offset = t->calib + t->shift_buffer_bits;
            phase_offset += (uint16_t)(GPIO_Group_Output_Offset[p] * BufferGapPerMicroseconds);
            phase_offset %= BufferResolution;

            uint32_t start_idx = (BufferResolution - phase_offset) % BufferResolution;
            uint32_t end_idx = (start_idx + half_period) % BufferResolution;
            uint16_t pin_bit = (1 << __builtin_ctz(t->pin));

            // Record Events
            turn_on[start_idx] |= pin_bit;
            turn_off[end_idx] |= pin_bit;

            // Handle Wrap-around Initial State
            if (start_idx >= end_idx) {
                current_state |= pin_bit;
            }
        }

        // 3. Fill Buffer
        uint16_t *buffer_ptr = &DMA_Buffer[p][offset];
        
        for (uint32_t i = 0; i < DMA_Buffer_Resolution; i++)
        {
            current_state |= turn_on[i];
            current_state &= ~turn_off[i];
            
            if (p == 0) current_state |= led_mask;

            buffer_ptr[i] = current_state;
        }
    }

    uint32_t end_cyc = DWT->CYCCNT;
    updateDMABufferDeltaTime = (double)((double)(end_cyc - start_cyc) / (double)SystemCoreClock);
}

void Clean_DMABuffer()
{
    memset(DMA_Buffer, 0x0000, sizeof(DMA_Buffer));
}
