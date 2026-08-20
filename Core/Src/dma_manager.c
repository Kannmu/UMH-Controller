#define _USE_MATH_DEFINES
#include "dma_manager.h"
#include "utiles.h"
#include "calibration.h"
#include "stimulation.h"
#include "stm32h7xx_hal_dma_ex.h"

const float GPIO_Group_Output_Offset[DMA_CHANNELS] = {0U, 0.06, 0.09, 0.16, 0.12};

const uint16_t half_period = WAVEFORM_BUFFER_SIZE / 2;

const uint16_t BufferGapPerMicroseconds = ((float)(1e-6) / TIME_GAP_PER_DMA_BUFFER_BIT);

float DMA_Clamp_Stimulation_Strength(float strength)
{
    if (!isfinite(strength))
    {
        return DMA_STRENGTH_MIN;
    }
    if (strength <= DMA_STRENGTH_MIN)
    {
        return DMA_STRENGTH_MIN;
    }
    if (strength >= DMA_STRENGTH_MAX)
    {
        return DMA_STRENGTH_MAX;
    }
    return strength;
}

uint16_t DMA_Convert_Strength_To_On_Ticks(float strength)
{
    float clamped_strength = DMA_Clamp_Stimulation_Strength(strength);
    float normalized_strength = clamped_strength / DMA_STRENGTH_MAX;
    float duty_cycle = DMA_DUTY_CYCLE_MIN + ((DMA_DUTY_CYCLE_MAX - DMA_DUTY_CYCLE_MIN) * normalized_strength);
    uint32_t on_ticks = (uint32_t)lroundf(duty_cycle * (float)WAVEFORM_BUFFER_SIZE);

    if (on_ticks > half_period)
    {
        on_ticks = half_period;
    }

    return (uint16_t)on_ticks;
}

DMA_HandleTypeDef *DMA_Stream_Handles[DMA_CHANNELS];

__ALIGNED(32)
DMA_WaveformBlock Waveform_Storage __attribute__((section(".storage_buffer")));

__ALIGNED(32)
static DMA_WaveformBlock Waveform_Staging __attribute__((section(".waveform_staging")));

typedef uint16_t WaveformChannel[NUM_STIMULATION_SAMPLES][WAVEFORM_BUFFER_SIZE];

extern TIM_HandleTypeDef htim1;

static uint16_t Group_Offset_Ticks[DMA_CHANNELS];

static Transducer *TransducersByPort[DMA_CHANNELS][NUM_TRANSDUCER];
static int TransducersByPortCount[DMA_CHANNELS];
static uint16_t TransducerPinMaskByPort[DMA_CHANNELS];
static WaveformChannel *Active_Waveform = Waveform_Storage;
static uint8_t DMAs_Started;
static uint8_t Audio_DMA_Active;
static uint8_t Audio_DMA_Prepared;

static GPIO_TypeDef * const Output_Ports[DMA_CHANNELS] = {
    GPIOA, GPIOB, GPIOC, GPIOD, GPIOE
};

static int TIM1_Stop_Output(void)
{
    int ok = 1;

    /* Keep the peripheral state and the HAL handle state synchronized. */
    if (HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_1) != HAL_OK) ok = 0;
    if (HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_2) != HAL_OK) ok = 0;
    if (HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_3) != HAL_OK) ok = 0;
    if (HAL_TIM_OC_Stop(&htim1, TIM_CHANNEL_4) != HAL_OK) ok = 0;
    if (HAL_TIM_Base_Stop(&htim1) != HAL_OK) ok = 0;

    __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_UPDATE);
    __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_CC1);
    __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_CC2);
    __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_CC3);
    __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_CC4);
    return ok;
}

static int TIM1_Start_Output(void)
{
    if (HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) return 0;
    if (HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) return 0;
    if (HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) return 0;
    if (HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) return 0;
    if (HAL_TIM_Base_Start(&htim1) != HAL_OK) return 0;
    return 1;
}

static void Activate_Waveform(WaveformChannel *waveform)
{
    uint32_t total_length = NUM_STIMULATION_SAMPLES * WAVEFORM_BUFFER_SIZE;

    if (waveform == Waveform_Staging)
    {
        SCB_CleanDCache_by_Addr((uint32_t *)Waveform_Staging,
                               (int32_t)sizeof(Waveform_Staging));
    }

    CLEAR_BIT(htim1.Instance->CR1, TIM_CR1_CEN);

    for (int p = 0; p < DMA_CHANNELS; p++)
    {
        if (HAL_DMA_Abort(DMA_Stream_Handles[p]) != HAL_OK)
        {
            Error_Handler();
        }
    }
    for (int p = 0; p < DMA_CHANNELS; p++)
    {
        Output_Ports[p]->BSRR = (uint32_t)TransducerPinMaskByPort[p] << 16U;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (int p = 0; p < DMA_CHANNELS; p++)
    {
        if (HAL_DMA_Start(DMA_Stream_Handles[p],
                          (uint32_t)&waveform[p][0][0],
                          (uint32_t)&Output_Ports[p]->ODR,
                          total_length) != HAL_OK)
        {
            if (primask == 0U) __enable_irq();
            Error_Handler();
        }
    }

    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    Active_Waveform = waveform;
    SET_BIT(htim1.Instance->CR1, TIM_CR1_CEN);
    if (primask == 0U)
    {
        __enable_irq();
    }
}

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
    memset(TransducerPinMaskByPort, 0, sizeof(TransducerPinMaskByPort));
    for (size_t i = 0; i < NUM_TRANSDUCER; i++)
    {
        Transducer *t = &TransducerArray[i];
        if (t->port_num < DMA_CHANNELS)
        {
            TransducersByPort[t->port_num][TransducersByPortCount[t->port_num]++] = t;
            TransducerPinMaskByPort[t->port_num] |= t->pin;
        }
    }

    Clean_DMABuffer();
    Update_Full_Waveform_Buffer();

    Start_DMAs();
}

void Start_DMAs()
{
    uint32_t total_length = NUM_STIMULATION_SAMPLES * WAVEFORM_BUFFER_SIZE;

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
                          (uint32_t)&Active_Waveform[i][0][0],
                          (uint32_t)&Output_Ports[i]->ODR,
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

    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    if (!TIM1_Start_Output()) Error_Handler();
    DMAs_Started = 1U;
    Audio_DMA_Active = 0U;
    Audio_DMA_Prepared = 0U;
}

DMA_WaveformBlock *DMA_Audio_Get_Block(uint8_t block)
{
    return (block == 0U) ? &Waveform_Storage : &Waveform_Staging;
}

void DMA_Audio_Clean_Block(uint8_t block)
{
    DMA_WaveformBlock *waveform = DMA_Audio_Get_Block(block);
    SCB_CleanDCache_by_Addr((uint32_t *)&(*waveform)[0][0][0],
                            (int32_t)sizeof(*waveform));
}

int DMA_Is_Audio_Active(void)
{
    return Audio_DMA_Active != 0U;
}

uint8_t DMA_Audio_Get_Playing_Block(void)
{
    if (!Audio_DMA_Active) return 0U;
    return ((DMA_Stream_TypeDef *)DMA_Stream_Handles[0]->Instance)->CR & DMA_SxCR_CT ? 1U : 0U;
}

int DMA_Audio_Prepare(void)
{
    if (Audio_DMA_Active) return 0;
    if (Audio_DMA_Prepared) return 1;

    if (!TIM1_Stop_Output()) return 0;
    if (DMAs_Started)
    {
        for (uint32_t p = 0; p < DMA_CHANNELS; p++)
        {
            if (HAL_DMA_Abort(DMA_Stream_Handles[p]) != HAL_OK) return 0;
        }
    }

    for (uint32_t p = 0; p < DMA_CHANNELS; p++)
        Output_Ports[p]->BSRR = (uint32_t)TransducerPinMaskByPort[p] << 16U;

    /* Both buffers are now safe to render because the legacy DMA is stopped. */
    memset(&Waveform_Storage[0][0][0], 0, sizeof(Waveform_Storage));
    memset(&Waveform_Staging[0][0][0], 0, sizeof(Waveform_Staging));
    DMA_Audio_Clean_Block(0U);
    DMA_Audio_Clean_Block(1U);
    DMAs_Started = 0U;
    Audio_DMA_Prepared = 1U;
    return 1;
}

int DMA_Audio_Start(void)
{
    uint32_t total_length = NUM_STIMULATION_SAMPLES * WAVEFORM_BUFFER_SIZE;

    if (Audio_DMA_Active) return 1;
    if (!Audio_DMA_Prepared && !DMA_Audio_Prepare()) return 0;

    for (uint32_t p = 0; p < DMA_CHANNELS; p++)
    {
        if (HAL_DMAEx_MultiBufferStart(DMA_Stream_Handles[p],
                                       (uint32_t)&Waveform_Storage[p][0][0],
                                       (uint32_t)&Output_Ports[p]->ODR,
                                       (uint32_t)&Waveform_Staging[p][0][0],
                                       total_length) != HAL_OK)
        {
            for (uint32_t i = 0; i < p; i++)
            {
                if (HAL_DMA_Abort(DMA_Stream_Handles[i]) != HAL_OK) Error_Handler();
            }
            Audio_DMA_Prepared = 0U;
            return 0;
        }
    }

    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC1);
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC2);
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC3);
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC4);
    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    if (!TIM1_Start_Output())
    {
        if (!TIM1_Stop_Output()) Error_Handler();
        for (uint32_t p = 0; p < DMA_CHANNELS; p++)
        {
            if (HAL_DMA_Abort(DMA_Stream_Handles[p]) != HAL_OK) Error_Handler();
        }
        Audio_DMA_Prepared = 0U;
        return 0;
    }
    Audio_DMA_Active = 1U;
    DMAs_Started = 1U;
    Audio_DMA_Prepared = 0U;
    return 1;
}

int DMA_Audio_Stop(void)
{
    if (!Audio_DMA_Active) return 1;
    if (!TIM1_Stop_Output()) return 0;
    int ok = 1;
    for (uint32_t p = 0; p < DMA_CHANNELS; p++)
    {
        if (HAL_DMA_Abort(DMA_Stream_Handles[p]) != HAL_OK) ok = 0;
        Output_Ports[p]->BSRR = (uint32_t)TransducerPinMaskByPort[p] << 16U;
    }
    Audio_DMA_Active = 0U;
    DMAs_Started = 0U;
    Audio_DMA_Prepared = 0U;
    return ok;
}

void Update_Full_Waveform_Buffer()
{
    uint32_t start_cycles = DWT_GetCycles();
    WaveformChannel *waveform = DMAs_Started
        ? ((Active_Waveform == Waveform_Storage) ? Waveform_Staging : Waveform_Storage)
        : Active_Waveform;

    // Per-tick edge masks. The tick domain is only 100 entries, so scanning it
    // directly is faster and deterministic compared with sorting edge indices.
    uint16_t turn_on[WAVEFORM_BUFFER_SIZE];
    uint16_t turn_off[WAVEFORM_BUFFER_SIZE];

    int is_enabled = Get_Stimulation_Enabled();
    int is_static_stim = (CurrentStimulation.type == Point || CurrentStimulation.type == TwinTrap);

    if (is_enabled && is_static_stim)
    {
        Update_Stimulation_State_Sample(0U);
    }

    // Point and TwinTrap have an identical carrier waveform in every trajectory
    // slot. Generate it once; the loop below replicates the exact samples.
    int samples_to_generate = (!is_enabled || is_static_stim) ? 1 : NUM_STIMULATION_SAMPLES;
    int phase_set_mode = Get_Phase_Set_Mode();
    int calibration_mode = Get_Calibration_Mode();
    uint16_t common_duty_ticks = DMA_Convert_Strength_To_On_Ticks(CurrentStimulation.strength);
    uint32_t calibration_pulse_samples = (uint32_t)(0.001f * TRANSDUCER_BASE_FREQ);
    uint16_t calibration_pin = TransducerArray[NUM_TRANSDUCER - 1].pin;
    uint8_t calibration_port = TransducerArray[NUM_TRANSDUCER - 1].port_num;
    uint16_t led_mask = Get_Current_LED_Mask();

    for (int s = 0; s < samples_to_generate; s++)
    {
        // Update Transducer State for this time slice
        if (is_enabled && !is_static_stim)
        {
            Update_Stimulation_State_Sample((uint32_t)s);
        }

        // Pre-calculate LED Mask (Port 0)
        // Note: With single buffer circular mode, this mask is fixed at generation time.
        // Dynamic blinking based on 'led0_ticks' during playback is not supported
        // without re-generating the buffer or using a separate mechanism.
        // Channel-Slice Loop
        for (int p = 0; p < DMA_CHANNELS; p++)
        {
            // 1. Reset Event Arrays
            memset(turn_on, 0, sizeof(turn_on));
            memset(turn_off, 0, sizeof(turn_off));

            uint16_t current_state = 0;

            // 2. Process Transducers for this Channel
            if (is_enabled)
            {
                int count = TransducersByPortCount[p];
                for (int k = 0; k < count; k++)
                {
                    Transducer *t = TransducersByPort[p][k];

                    // Virtual Transducer PC10 (Index 60) Functionality
                    if (t->index == NUM_TRANSDUCER - 1 && calibration_mode == 0)
                    {
                        // 1ms pulse at the beginning of each stimulation cycle
                        if ((uint32_t)s < calibration_pulse_samples)
                        {
                            current_state |= t->pin;
                        }
                        continue; // Skip standard 40kHz generation
                    }

                    // Phase Calculation
                    uint16_t phase_offset = t->calib + t->shift_buffer_bits;
                    phase_offset += Group_Offset_Ticks[p];
                    phase_offset %= WAVEFORM_BUFFER_SIZE;

                    uint32_t start_idx = (WAVEFORM_BUFFER_SIZE - phase_offset) % WAVEFORM_BUFFER_SIZE;
                    uint16_t duty_ticks;
                    if (phase_set_mode == 1)
                    {
                        uint32_t on_ticks = (uint32_t)lroundf(t->duty * (float)half_period);
                        if (on_ticks > half_period) on_ticks = half_period;
                        duty_ticks = (uint16_t)on_ticks;
                    }
                    else
                    {
                        duty_ticks = common_duty_ticks;
                    }
                    
                    uint32_t end_idx = (start_idx + duty_ticks) % WAVEFORM_BUFFER_SIZE;
                    uint16_t pin_bit = t->pin;

                    if (duty_ticks == 0U)
                    {
                        continue;
                    }

                    turn_on[start_idx] |= pin_bit;
                    turn_off[end_idx] |= pin_bit;

                    // Handle Wrap-around Initial State
                    if (start_idx >= end_idx)
                    {
                        current_state |= pin_bit;
                    }
                }
            }

            // Apply all edges in chronological order while writing the fixed-size
            // carrier period. This produces the same samples as the sorted RLE path.
            uint16_t *buffer_ptr_base = &waveform[p][s][0];
            uint16_t running_state = current_state;
            uint16_t port_mask = (p == 0) ? led_mask : 0U;
            uint32_t tick = 0U;
            for (; tick + 1U < WAVEFORM_BUFFER_SIZE; tick += 2U)
            {
                running_state |= turn_on[tick];
                running_state &= (uint16_t)~turn_off[tick];
                uint16_t first = running_state | port_mask;

                running_state |= turn_on[tick + 1U];
                running_state &= (uint16_t)~turn_off[tick + 1U];
                uint16_t second = running_state | port_mask;

                uint32_t packed = (uint32_t)first | ((uint32_t)second << 16U);
                memcpy(&buffer_ptr_base[tick], &packed, sizeof(packed));
            }
            if (tick < WAVEFORM_BUFFER_SIZE)
            {
                running_state |= turn_on[tick];
                running_state &= (uint16_t)~turn_off[tick];
                buffer_ptr_base[tick] = running_state | port_mask;
            }
        }
    }

    if (samples_to_generate == 1)
    {
        for (int p = 0; p < DMA_CHANNELS; p++)
        {
            for (uint32_t s = 1; s < NUM_STIMULATION_SAMPLES; s++)
            {
                uint16_t *destination = &waveform[p][s][0];
                memcpy(destination, &waveform[p][0][0],
                       WAVEFORM_BUFFER_SIZE * sizeof(uint16_t));

                // PC10 is a 1 ms cycle marker rather than a carrier channel.
                if (calibration_mode == 0 && p == calibration_port &&
                    s >= calibration_pulse_samples)
                {
                    uint16_t keep_mask = (uint16_t)~calibration_pin;
                    for (uint32_t tick = 0; tick < WAVEFORM_BUFFER_SIZE; tick++)
                    {
                        destination[tick] &= keep_mask;
                    }
                }
            }
        }
    }

    if (DMAs_Started)
    {
        Activate_Waveform(waveform);
    }

    uint32_t end_cycles = DWT_GetCycles();
    if (SystemCoreClock > 0)
    {
        // Calculate delta time in milliseconds
        updateDMABufferDeltaTime = (double)(end_cycles - start_cycles) / (SystemCoreClock / 1000.0);
    }
}

void Clean_DMABuffer()
{
    memset(Waveform_Storage, 0x0000, sizeof(Waveform_Storage));
    memset(Waveform_Staging, 0x0000, sizeof(Waveform_Staging));
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

    uint32_t block_count = DMA_Is_Audio_Active() ? 2U : 1U;
    for (uint32_t b = 0; b < block_count; b++)
    {
        DMA_WaveformBlock *block;
        if (DMA_Is_Audio_Active())
            block = DMA_Audio_Get_Block((uint8_t)b);
        else
            block = (DMA_WaveformBlock *)Active_Waveform;
        for (uint32_t s = 0; s < NUM_STIMULATION_SAMPLES; s++)
        {
            uint16_t *buffer_ptr = &(*block)[0][s][0];
            for (uint32_t i = 0; i < WAVEFORM_BUFFER_SIZE; i++)
                buffer_ptr[i] = (buffer_ptr[i] & ~LED_MASK_BITS) | led_bits;
        }
        if (DMA_Is_Audio_Active()) DMA_Audio_Clean_Block((uint8_t)b);
    }
}
