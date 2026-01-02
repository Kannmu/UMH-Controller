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
}

void Update_All_DMABuffer()
{
    uint32_t start_cyc = DWT->CYCCNT;

    // 1. Pre-calculate active ranges for all transducers
    typedef struct {
        uint8_t port_num;
        uint16_t pin;
        uint32_t start_idx;
        uint32_t end_idx;
        uint8_t wrap; // 0 or 1
    } TransducerState;

    TransducerState states[NumTransducer];
    
    for (size_t i = 0; i < NumTransducer; i++)
    {
        Transducer *t = &TransducerArray[i];
        states[i].port_num = t->port_num;
        states[i].pin = t->pin;
        
        uint16_t phase_offset = t->calib + t->shift_buffer_bits;
        phase_offset += (uint16_t)(GPIO_Group_Output_Offset[t->port_num] * BufferGapPerMicroseconds);
        phase_offset %= BufferResolution;
        
        states[i].start_idx = (BufferResolution - phase_offset) % BufferResolution;
        uint32_t end = states[i].start_idx + half_period;
        
        if (end <= BufferResolution) {
            states[i].end_idx = end;
            states[i].wrap = 0;
        } else {
            states[i].end_idx = end; // Used for logic check
            states[i].wrap = 1;
        }
    }

    // 2. Prepare LED Base State (Port 0)
    // Logic from Restore_LED_State in utiles.c
    uint16_t led_mask = 0;
    
    // LED0
    if (!led0_state) led_mask |= LED0_Pin; // Restore_LED: if(state) &= ~Pin (OFF). So if(!state) ON.
    
    // LED1 (Calibration)
    if (!Get_Calibration_Mode()) led_mask |= LED1_Pin; // Restore_LED: if(Mode) &= ~Pin.
    
    // LED2 (Plane)
    if (!Get_Plane_Mode()) led_mask |= LED2_Pin; // Restore_LED: if(Mode) &= ~Pin.

    // 3. Iterate Buffer and Fill
    for (uint32_t i = 0; i < BufferResolution; i++)
    {
        uint16_t port_vals[DMA_CHANNELS] = {0};
        
        // Initialize Port 0 with LEDs
        port_vals[0] = led_mask;
        
        // Accumulate Transducers
        for (size_t k = 0; k < NumTransducer; k++)
        {
            uint8_t active = 0;
            if (states[k].wrap) {
                if (i >= states[k].start_idx || i < (states[k].end_idx - BufferResolution)) active = 1;
            } else {
                if (i >= states[k].start_idx && i < states[k].end_idx) active = 1;
            }
            
            if (active) {
                port_vals[states[k].port_num] |= states[k].pin;
            }
        }
        
        // Write to DMA Buffer
        for (int p = 0; p < DMA_CHANNELS; p++) {
            DMA_Buffer[p][i] = port_vals[p];
        }
    }

    uint32_t end_cyc = DWT->CYCCNT;
    updateDMABufferDeltaTime =  (double)((double)(end_cyc - start_cyc) / (double)SystemCoreClock);
}

// Legacy function to update a single transducer's DMA buffer
void Update_Single_DMABuffer(Transducer *currentTransducer)
{
    const uint8_t port_num = currentTransducer->port_num;
    const uint16_t pin = currentTransducer->pin;
    
    uint16_t phase_offset = currentTransducer->calib + currentTransducer->shift_buffer_bits;

    phase_offset += (uint16_t)(GPIO_Group_Output_Offset[port_num] * BufferGapPerMicroseconds);

    phase_offset %= BufferResolution;
    
    uint32_t start_idx = (BufferResolution - phase_offset) % BufferResolution;
    uint32_t end_idx = start_idx + half_period;
    uint16_t* pBuffer = DMA_Buffer[port_num];

    if (end_idx <= BufferResolution)
    {
        // No wrap-around
        for (size_t j = start_idx; j < end_idx; j++)
        {
            pBuffer[j] |= pin;
        }
    }
    else
    {
        // Wrap-around: Fill to end, then start from beginning
        for (size_t j = start_idx; j < BufferResolution; j++)
        {
            pBuffer[j] |= pin;
        }
        size_t remaining = end_idx - BufferResolution;
        for (size_t j = 0; j < remaining; j++)
        {
            pBuffer[j] |= pin;
        }
    }
}

// Legacy function to clean all DMA buffers
void Clean_DMABuffer()
{
    memset(DMA_Buffer, 0x0000, sizeof(DMA_Buffer));
    Restore_LED_State();
}
