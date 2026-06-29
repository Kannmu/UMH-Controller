#define _USE_MATH_DEFINES
#include "dma_manager.h"
#include "utiles.h"
#include "calibration.h"
#include "stimulation.h"

const uint16_t half_period = WAVEFORM_BUFFER_SIZE / 2;

const uint16_t BufferGapPerMicroseconds = ((float)(1e-6) / TIME_GAP_PER_DMA_BUFFER_BIT);

float DMA_Clamp_Stimulation_Strength(float strength)
{
    if (strength <= DMA_STRENGTH_MIN)
        return DMA_STRENGTH_MIN;
    if (strength >= DMA_STRENGTH_MAX)
        return DMA_STRENGTH_MAX;
    return strength;
}

uint16_t DMA_Convert_Strength_To_On_Ticks(float strength)
{
    float clamped_strength = DMA_Clamp_Stimulation_Strength(strength);
    float normalized_strength = clamped_strength / DMA_STRENGTH_MAX;
    float duty_cycle = DMA_DUTY_CYCLE_MIN + ((DMA_DUTY_CYCLE_MAX - DMA_DUTY_CYCLE_MIN) * normalized_strength);
    uint32_t on_ticks = (uint32_t)lroundf(duty_cycle * (float)WAVEFORM_BUFFER_SIZE);
    if (on_ticks > half_period)
        on_ticks = half_period;
    return (uint16_t)on_ticks;
}

DMA_HandleTypeDef *DMA_Stream_Handles[DMA_CHANNELS];

__ALIGNED(32)
uint16_t Waveform_Storage[DMA_CHANNELS][NUM_STIMULATION_SAMPLES][WAVEFORM_BUFFER_SIZE] __attribute__((section(".storage_buffer")));

extern TIM_HandleTypeDef htim1;

static Transducer *TransducersByPort[DMA_CHANNELS][NUM_TOTAL_CHANNELS];
static int TransducersByPortCount[DMA_CHANNELS];

/* TRIGGER0 / TRIGGER1 配置 */
static uint8_t  trigger0_enable   = 1;
static uint32_t trigger0_pulse_us = 1000;
static uint8_t  trigger1_enable   = 0;

void DMA_Init()
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA_CHANNELS=4: B(idx0)=TIM1_CH1, C(idx1)=TIM1_CH2, D(idx2)=TIM1_CH3, E(idx3)=TIM1_CH4 */
    DMA_Stream_Handles[0] = &hdma_memtomem_dma1_stream1;   /* GPIOB */
    DMA_Stream_Handles[1] = &hdma_memtomem_dma1_stream2;   /* GPIOC */
    DMA_Stream_Handles[2] = &hdma_memtomem_dma2_stream0;   /* GPIOD */
    DMA_Stream_Handles[3] = &hdma_memtomem_dma2_stream1;   /* GPIOE */

    /* Configure Init fields for each DMA stream */
    static const DMA_Stream_TypeDef *const dma_instances[4] = {
        DMA1_Stream1,  /* GPIOB */
        DMA1_Stream2,  /* GPIOC */
        DMA2_Stream0,  /* GPIOD */
        DMA2_Stream1   /* GPIOE */
    };
    static const uint32_t dma_requests[4] = {
        DMA_REQUEST_TIM1_CH1,  /* GPIOB */
        DMA_REQUEST_TIM1_CH2,  /* GPIOC */
        DMA_REQUEST_TIM1_CH3,  /* GPIOD */
        DMA_REQUEST_TIM1_CH4   /* GPIOE */
    };
    for (int i = 0; i < DMA_CHANNELS; i++)
    {
        DMA_Stream_Handles[i]->Instance               = dma_instances[i];
        DMA_Stream_Handles[i]->Init.Request           = dma_requests[i];
        DMA_Stream_Handles[i]->Init.Direction         = DMA_MEMORY_TO_PERIPH;
        DMA_Stream_Handles[i]->Init.PeriphInc         = DMA_PINC_DISABLE;
        DMA_Stream_Handles[i]->Init.MemInc            = DMA_MINC_ENABLE;
        DMA_Stream_Handles[i]->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        DMA_Stream_Handles[i]->Init.MemDataAlignment  = DMA_MDATAALIGN_HALFWORD;
        DMA_Stream_Handles[i]->Init.Mode              = DMA_CIRCULAR;
        DMA_Stream_Handles[i]->Init.Priority          = DMA_PRIORITY_VERY_HIGH;
        DMA_Stream_Handles[i]->Init.FIFOMode          = DMA_FIFOMODE_DISABLE;
        DMA_Stream_Handles[i]->Init.MemBurst          = DMA_MBURST_SINGLE;
        DMA_Stream_Handles[i]->Init.PeriphBurst       = DMA_PBURST_SINGLE;
    }

    memset(TransducersByPortCount, 0, sizeof(TransducersByPortCount));
    for (size_t i = 0; i < NUM_TOTAL_CHANNELS; i++)
    {
        Transducer *t = &TransducerArray[i];
        if (t->port_num < DMA_CHANNELS)
            TransducersByPort[t->port_num][TransducersByPortCount[t->port_num]++] = t;
    }

    Clean_DMABuffer();
    Update_Full_Waveform_Buffer();
    Start_DMAs();
}

void Start_DMAs()
{
    uint32_t total_length = NUM_STIMULATION_SAMPLES * WAVEFORM_BUFFER_SIZE;

    uint32_t *dest_addrs[DMA_CHANNELS] = {
        (uint32_t *)(&(GPIOB->ODR)),
        (uint32_t *)(&(GPIOC->ODR)),
        (uint32_t *)(&(GPIOD->ODR)),
        (uint32_t *)(&(GPIOE->ODR))};

    for (int i = 0; i < DMA_CHANNELS; i++)
    {
        DMA_Stream_Handles[i]->Init.Mode = DMA_CIRCULAR;
        if (HAL_DMA_Init(DMA_Stream_Handles[i]) != HAL_OK)
            Error_Handler();
        if (HAL_DMA_Start(DMA_Stream_Handles[i],
                          (uint32_t)&Waveform_Storage[i][0][0],
                          (uint32_t)dest_addrs[i],
                          total_length) != HAL_OK)
            Error_Handler();
    }

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

void Update_Full_Waveform_Buffer()
{
    uint32_t start_cycles = DWT_GetCycles();

    uint16_t turn_on[WAVEFORM_BUFFER_SIZE];
    uint16_t turn_off[WAVEFORM_BUFFER_SIZE];
    uint16_t event_indices[NUM_TOTAL_CHANNELS * 2 + 2];
    int event_count;

    int is_enabled = Get_Stimulation_Enabled();
    int is_static_stim = (CurrentStimulation.type == Point || CurrentStimulation.type == TwinTrap);

    if (is_enabled && is_static_stim) {
        Update_Stimulation_State(0.0f);
    }

    /* Static-mode: Point/TwinTrap produce identical 40kHz waveform across all
     * 200 stimulation samples. Compute s=0 only, replicate to s=1..199.
     * TRIGGER0 (s < pulse_samples) is included in s=0; s >= pulse must be cleared.
     * ~200x speedup for static modes. */
    uint16_t static_trigger0_pin = 0;
    uint32_t static_pulse_samples = 0;
    int static_mode = (is_enabled && is_static_stim);
    if (static_mode && trigger0_enable) {
        static_trigger0_pin = TransducerArray[TRIGGER0_INDEX].pin;
        static_pulse_samples = (uint32_t)(trigger0_pulse_us * (uint32_t)TRANSDUCER_BASE_FREQ / 1000000u);
    }

    int s_end = static_mode ? 1 : NUM_STIMULATION_SAMPLES;

    for (int s = 0; s < s_end; s++)
    {
        float progress = (float)s / (float)NUM_STIMULATION_SAMPLES;

        if (is_enabled && !is_static_stim)
            Update_Stimulation_State(progress);

        /* Hoist: duty_ticks is identical for all transducers when phase_set_mode==0 */
        uint16_t static_duty_ticks = 0;
        if (is_enabled && Get_Phase_Set_Mode() == 0)
            static_duty_ticks = DMA_Convert_Strength_To_On_Ticks(CurrentStimulation.strength);

        for (int p = 0; p < DMA_CHANNELS; p++)
        {
            memset(turn_on, 0, sizeof(turn_on));
            memset(turn_off, 0, sizeof(turn_off));
            event_count = 0;
            event_indices[event_count++] = 0;

            uint16_t current_state = 0;

            if (is_enabled)
            {
                int count = TransducersByPortCount[p];
                for (int k = 0; k < count; k++)
                {
                    Transducer *t = TransducersByPort[p][k];

                    /* Virtual reference (PC13): continuous 40kHz, phase=0, 50% duty */
                    if (t->index == VIRTUAL_INDEX)
                    {
                        uint16_t pin_bit = (uint16_t)(1U << __builtin_ctz(t->pin));
                        uint32_t start_idx = 0;
                        uint32_t end_idx = half_period;  /* 50 */

                        if (turn_on[start_idx] == 0 && turn_off[start_idx] == 0)
                            event_indices[event_count++] = (uint16_t)start_idx;
                        turn_on[start_idx] |= pin_bit;
                        if (turn_on[end_idx] == 0 && turn_off[end_idx] == 0)
                            event_indices[event_count++] = (uint16_t)end_idx;
                        turn_off[end_idx] |= pin_bit;
                        continue;
                    }

                    /* TRIGGER0 (PC14): 1ms pulse at start of each 200Hz cycle */
                    if (t->index == TRIGGER0_INDEX)
                    {
                        if (trigger0_enable)
                        {
                            uint32_t pulse = (uint32_t)(trigger0_pulse_us * (uint32_t)TRANSDUCER_BASE_FREQ / 1000000u);
                            if (s < (int)pulse)
                                current_state |= t->pin;
                        }
                        continue;
                    }

                    /* TRIGGER1 (PC15): configurable, default all-0 */
                    if (t->index == TRIGGER1_INDEX)
                        continue;

                    /* Real transducers (0..59): standard phase computation */
                    uint16_t phase_offset = t->calib + t->shift_buffer_bits;
                    phase_offset %= WAVEFORM_BUFFER_SIZE;

                    uint32_t start_idx = (WAVEFORM_BUFFER_SIZE - phase_offset) % WAVEFORM_BUFFER_SIZE;
                    uint16_t duty_ticks;
                    if (Get_Phase_Set_Mode() == 1)
                    {
                        uint32_t on = (uint32_t)lroundf(t->duty * (float)half_period);
                        if (on > half_period) on = half_period;
                        duty_ticks = (uint16_t)on;
                    }
                    else
                    {
                        duty_ticks = static_duty_ticks;
                    }
                    if (duty_ticks == 0U) continue;

                    uint32_t end_idx = (start_idx + duty_ticks) % WAVEFORM_BUFFER_SIZE;
                    uint16_t pin_bit = (1U << __builtin_ctz(t->pin));

                    if (turn_on[start_idx] == 0 && turn_off[start_idx] == 0)
                        event_indices[event_count++] = (uint16_t)start_idx;
                    turn_on[start_idx] |= pin_bit;
                    if (turn_on[end_idx] == 0 && turn_off[end_idx] == 0)
                        event_indices[event_count++] = (uint16_t)end_idx;
                    turn_off[end_idx] |= pin_bit;

                    if (start_idx >= end_idx)
                        current_state |= pin_bit;
                }
            }

            /* Insertion sort event indices */
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

            /* Run-length fill */
            uint16_t *buf = &Waveform_Storage[p][s][0];
            uint16_t running = current_state;
            int prev_idx = 0;

            for (int i = 0; i < event_count; i++)
            {
                int idx = event_indices[i];
                if (i > 0 && idx == event_indices[i - 1])
                    continue;

                int gap = idx - prev_idx;
                if (gap > 0)
                {
                    uint16_t *ptr = &buf[prev_idx];
                    while (gap >= 4) { ptr[0]=running; ptr[1]=running; ptr[2]=running; ptr[3]=running; ptr+=4; gap-=4; }
                    while (gap-- > 0) *ptr++ = running;
                }

                if (idx < WAVEFORM_BUFFER_SIZE)
                {
                    running |= turn_on[idx];
                    running &= ~turn_off[idx];
                }
                prev_idx = idx;
            }

            int tail = WAVEFORM_BUFFER_SIZE - prev_idx;
            if (tail > 0)
            {
                uint16_t *ptr = &buf[prev_idx];
                while (tail >= 4) { ptr[0]=running; ptr[1]=running; ptr[2]=running; ptr[3]=running; ptr+=4; tail-=4; }
                while (tail-- > 0) *ptr++ = running;
            }
        }
    }

    /* Static-mode replication: copy s=0 to s=1..199, clear TRIGGER0 for s>=pulse */
    if (static_mode)
    {
        for (int p = 0; p < DMA_CHANNELS; p++)
        {
            for (int s = 1; s < NUM_STIMULATION_SAMPLES; s++)
            {
                memcpy(&Waveform_Storage[p][s][0],
                       &Waveform_Storage[p][0][0],
                       WAVEFORM_BUFFER_SIZE * sizeof(uint16_t));

                if (static_pulse_samples > 0 && s >= (int)static_pulse_samples)
                {
                    uint16_t mask = ~static_trigger0_pin;
                    for (int i = 0; i < (int)WAVEFORM_BUFFER_SIZE; i++)
                        Waveform_Storage[p][s][i] &= mask;
                }
            }
        }
    }

    uint32_t end_cycles = DWT_GetCycles();
    if (SystemCoreClock > 0)
        updateDMABufferDeltaTime = (double)(end_cycles - start_cycles) / (SystemCoreClock / 1000.0);
}

void Clean_DMABuffer()
{
    memset(Waveform_Storage, 0x0000, sizeof(Waveform_Storage));
}

void Configure_Trigger0(uint8_t enable, uint32_t pulse_us)
{
    trigger0_enable   = enable;
    trigger0_pulse_us = (pulse_us == 0) ? 1000u : pulse_us;
    Update_Full_Waveform_Buffer();
}

void Configure_Trigger1(uint8_t enable)
{
    trigger1_enable = enable;
    Update_Full_Waveform_Buffer();
}

/* ---- Single-transducer mode for semi-auto calibration ---- */
void Calib_SetSingleTransducer(uint8_t idx)
{
    if (idx >= NUM_REAL_TRANSDUCER) return;

    /* Bypass Get_Stimulation_Enabled gate: enable stimulation temporarily */
    if (!Get_Stimulation_Enabled())
        Stimulation_Enable();

    /* Zero all calibration, phase, and duty; only target emits at 50% duty */
    Enter_Calibration_Mode();
    for (int i = 0; i < NUM_REAL_TRANSDUCER; i++)
        TransducerArray[i].duty = 0.0f;
    TransducerArray[idx].duty = 0.5f;

    /* PC13 VIRTUAL reference continues unchanged (not in real-transducer range) */
    Update_Full_Waveform_Buffer();
}

void Calib_SetNormalDrive(void)
{
    Load_Calib_to_Transducers();
    for (int i = 0; i < NUM_REAL_TRANSDUCER; i++)
        TransducerArray[i].duty = 0.5f;
    Update_Full_Waveform_Buffer();
}
