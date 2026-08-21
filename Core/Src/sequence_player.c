#include "sequence_player.h"

#include <math.h>
#include <string.h>

#include "dma_manager.h"
#include "sequence_math.h"
#include "stimulation.h"
#include "transducer.h"
#include "utiles.h"

static uint8_t sequence_phase_offsets[SEQUENCE_MAX_STATES][SEQUENCE_OUTPUT_CHANNELS];
#define SEQUENCE_DUTY_LEVELS ((WAVEFORM_BUFFER_SIZE / 2U) + 1U)

/* The generic event cache supports arbitrary per-transducer phase tables.
 * For the common shared-phase table, the same storage is reused for complete
 * mono waveforms at every duty level, which removes the hot event scan. */
typedef union
{
    uint16_t start_masks[DMA_CHANNELS][SEQUENCE_RENDER_STATES]
                        [WAVEFORM_BUFFER_SIZE];
    uint16_t uniform_waveforms[DMA_CHANNELS][SEQUENCE_DUTY_LEVELS]
                              [WAVEFORM_BUFFER_SIZE];
} SequenceRenderCache;

static SequenceRenderCache sequence_render_cache
    __attribute__((section(".sequence_templates")));
static uint16_t sequence_port_masks[DMA_CHANNELS];
static uint8_t sequence_duty_ticks[256];
static uint8_t sequence_state_tick_indices[SEQUENCE_RENDER_STATES]
                                         [WAVEFORM_BUFFER_SIZE];
static uint8_t sequence_uniform_phase;
static uint16_t sequence_ring[SEQUENCE_RING_CAPACITY];
static uint16_t sequence_base_shift[SEQUENCE_OUTPUT_CHANNELS];
static SequenceDescriptor sequence_descriptor;
static SequenceState sequence_state;
static uint16_t sequence_uploaded_states;
static uint16_t sequence_ring_head;
static uint16_t sequence_ring_tail;
static uint8_t sequence_last_state;
static uint8_t sequence_packet_valid;
static uint32_t sequence_expected_packet;
static uint32_t sequence_underrun_count;
static uint32_t sequence_overrun_count;
static uint32_t sequence_packet_loss_count;
static uint32_t sequence_render_timeout_count;
static int32_t sequence_clock_correction_ppm;
static uint32_t sequence_current_render_us;
static uint32_t sequence_maximum_render_us;
static uint32_t sequence_rendered_block_count;
static uint32_t sequence_render_over_budget_count;
static uint32_t sequence_dma_deadline_miss_count;

static uint16_t Sequence_Ring_Fill(void)
{
    return (uint16_t)((sequence_ring_head - sequence_ring_tail) &
                      (SEQUENCE_RING_CAPACITY - 1U));
}

static uint16_t Sequence_Ring_Free(void)
{
    return (uint16_t)((SEQUENCE_RING_CAPACITY - 1U) - Sequence_Ring_Fill());
}

static uint16_t Sequence_Ring_Peek(uint16_t offset)
{
    return sequence_ring[(sequence_ring_tail + offset) &
                         (SEQUENCE_RING_CAPACITY - 1U)];
}

static void Sequence_Ring_Consume(uint16_t count)
{
    while (count-- > 0U)
    {
        sequence_ring_tail = (uint16_t)((sequence_ring_tail + 1U) &
                                        (SEQUENCE_RING_CAPACITY - 1U));
    }
}

static void Sequence_Reset_Stream(void)
{
    sequence_ring_head = 0U;
    sequence_ring_tail = 0U;
    sequence_last_state = sequence_descriptor.neutral_state;
    sequence_packet_valid = 0U;
    sequence_expected_packet = 0U;
    sequence_clock_correction_ppm = 0;
}

static int Sequence_Descriptor_Is_Valid(const SequenceDescriptor *descriptor)
{
    if (descriptor == NULL || descriptor->state_count == 0U ||
        descriptor->state_count > SEQUENCE_RENDER_STATES ||
        descriptor->neutral_state >= descriptor->state_count ||
        descriptor->control_scale_q16 < 0 ||
        descriptor->mapping > SEQUENCE_MAPPING_CYCLIC_INCREMENT)
    {
        return 0;
    }
    for (uint32_t axis = 0U; axis < 3U; axis++)
    {
        if (!isfinite(descriptor->focus[axis])) return 0;
    }
    return 1;
}

static void Sequence_Build_Render_Cache(void)
{
    memset(sequence_port_masks, 0, sizeof(sequence_port_masks));
    sequence_uniform_phase = 1U;
    for (uint16_t state = 0U; state < sequence_descriptor.state_count; state++)
    {
        for (uint32_t index = 1U; index < SEQUENCE_OUTPUT_CHANNELS; index++)
        {
            if (sequence_phase_offsets[state][index] != sequence_phase_offsets[state][0])
            {
                sequence_uniform_phase = 0U;
                break;
            }
        }
        if (sequence_uniform_phase == 0U) break;
    }

    if (sequence_uniform_phase == 0U)
    {
        memset(sequence_render_cache.start_masks, 0,
               sizeof(sequence_render_cache.start_masks));
    }

    uint8_t base_starts[SEQUENCE_OUTPUT_CHANNELS];
    for (uint32_t index = 0U; index < SEQUENCE_OUTPUT_CHANNELS; index++)
    {
        const Transducer *transducer = &TransducerArray[index];
        if (transducer->port_num >= DMA_CHANNELS) continue;

        uint8_t port = transducer->port_num;
        sequence_port_masks[port] |= transducer->pin;

        uint32_t phase = (uint32_t)transducer->calib + sequence_base_shift[index] +
            sequence_phase_offsets[0][index] +
            (uint32_t)(GPIO_Group_Output_Offset[port] * BufferGapPerMicroseconds);
        phase %= WAVEFORM_BUFFER_SIZE;
        base_starts[index] = (uint8_t)((WAVEFORM_BUFFER_SIZE - phase) % WAVEFORM_BUFFER_SIZE);

        if (sequence_uniform_phase == 0U)
        {
            for (uint16_t state = 0U; state < sequence_descriptor.state_count; state++)
            {
                phase = (uint32_t)transducer->calib + sequence_base_shift[index] +
                    sequence_phase_offsets[state][index] +
                    (uint32_t)(GPIO_Group_Output_Offset[port] * BufferGapPerMicroseconds);
                phase %= WAVEFORM_BUFFER_SIZE;
                uint8_t start = (uint8_t)((WAVEFORM_BUFFER_SIZE - phase) % WAVEFORM_BUFFER_SIZE);
                sequence_render_cache.start_masks[port][state][start] |= transducer->pin;
            }
        }
    }

    if (sequence_uniform_phase != 0U)
    {
        uint8_t phase0 = sequence_phase_offsets[0][0];
        for (uint16_t state = 0U; state < sequence_descriptor.state_count; state++)
        {
            uint8_t shift = (uint8_t)((sequence_phase_offsets[state][0] +
                                       WAVEFORM_BUFFER_SIZE - phase0) % WAVEFORM_BUFFER_SIZE);
            for (uint32_t tick = 0U; tick < WAVEFORM_BUFFER_SIZE; tick++)
            {
                uint32_t index = tick + shift;
                if (index >= WAVEFORM_BUFFER_SIZE) index -= WAVEFORM_BUFFER_SIZE;
                sequence_state_tick_indices[state][tick] = (uint8_t)index;
            }
        }

        for (uint32_t port = 0U; port < DMA_CHANNELS; port++)
        {
            for (uint32_t duty = 0U; duty < SEQUENCE_DUTY_LEVELS; duty++)
            {
                uint16_t turn_on[WAVEFORM_BUFFER_SIZE] = {0};
                uint16_t turn_off[WAVEFORM_BUFFER_SIZE] = {0};
                uint16_t running_state = 0U;
                for (uint32_t index = 0U; index < SEQUENCE_OUTPUT_CHANNELS; index++)
                {
                    const Transducer *transducer = &TransducerArray[index];
                    if (transducer->port_num != port) continue;
                    uint32_t start = base_starts[index];
                    uint32_t end = start + duty;
                    if (end >= WAVEFORM_BUFFER_SIZE) end -= WAVEFORM_BUFFER_SIZE;
                    turn_on[start] |= transducer->pin;
                    turn_off[end] |= transducer->pin;
                    if (start >= end) running_state |= transducer->pin;
                }
                for (uint32_t tick = 0U; tick < WAVEFORM_BUFFER_SIZE; tick++)
                {
                    running_state |= turn_on[tick];
                    running_state &= (uint16_t)~turn_off[tick];
                    sequence_render_cache.uniform_waveforms[port][duty][tick] =
                        running_state & sequence_port_masks[port];
                }
            }
        }
    }

    for (uint32_t envelope = 0U; envelope < 256U; envelope++)
    {
        sequence_duty_ticks[envelope] = (uint8_t)
            (((envelope * (WAVEFORM_BUFFER_SIZE / 2U)) + 127U) / 255U);
    }
}

static void Sequence_Render_Sample(DMA_WaveformBlock *waveform, uint32_t sample,
                                   uint16_t state, uint8_t envelope,
                                   uint16_t led_mask)
{
    if (state >= sequence_descriptor.state_count)
        state = sequence_descriptor.neutral_state;
    uint32_t duty = sequence_duty_ticks[envelope];

    if (sequence_uniform_phase != 0U)
    {
        const uint8_t *indices = sequence_state_tick_indices[state];
        const uint32_t shift = indices[0];
        for (uint32_t port = 0U; port < DMA_CHANNELS; port++)
        {
            const uint16_t *source = sequence_render_cache.uniform_waveforms[port][duty];
            uint16_t *destination = (*waveform)[port][sample];
            uint16_t port_mask = port == 0U ? led_mask : 0U;
            if (port != 0U || port_mask == 0U)
            {
                if (shift == 0U)
                {
                    memcpy(destination, source,
                           WAVEFORM_BUFFER_SIZE * sizeof(uint16_t));
                }
                else
                {
                    const uint32_t first = WAVEFORM_BUFFER_SIZE - shift;
                    memcpy(destination, &source[shift], first * sizeof(uint16_t));
                    memcpy(&destination[first], source, shift * sizeof(uint16_t));
                }
            }
            else
            {
                for (uint32_t tick = 0U; tick < WAVEFORM_BUFFER_SIZE; tick++)
                    destination[tick] = source[indices[tick]] | port_mask;
            }
        }
        return;
    }

    for (uint32_t port = 0U; port < DMA_CHANNELS; port++)
    {
        const uint16_t *starts = sequence_render_cache.start_masks[port][state];
        const uint16_t port_mask = sequence_port_masks[port];
        uint16_t running_state = 0U;

        /* Restore pulses that cross tick zero. The loop is bounded by the
         * 50-tick maximum duty and replaces the old per-channel setup. */
        for (uint32_t offset = 0U; offset < duty; offset++)
        {
            uint32_t tick = WAVEFORM_BUFFER_SIZE - duty + offset;
            running_state |= starts[tick] & port_mask;
        }

        for (uint32_t tick = 0U; tick < WAVEFORM_BUFFER_SIZE; tick++)
        {
            uint32_t end = tick + WAVEFORM_BUFFER_SIZE - duty;
            if (end >= WAVEFORM_BUFFER_SIZE) end -= WAVEFORM_BUFFER_SIZE;
            uint16_t turn_on = starts[tick] & port_mask;
            uint16_t turn_off = starts[end] & port_mask;
            running_state |= turn_on;
            running_state &= (uint16_t)~turn_off;
            (*waveform)[port][sample][tick] = running_state |
                (port == 0U ? led_mask : 0U);
        }
    }
}

static void Sequence_Render_Data_Block(uint8_t block)
{
    DMA_WaveformBlock *waveform = DMA_Sequence_Get_Block(block);
    sequence_clock_correction_ppm = 0;
    uint16_t led_mask = Get_Current_LED_Mask();
    for (uint32_t sample = 0U; sample < SEQUENCE_BLOCK_SAMPLES; sample++)
    {
        uint16_t packed = Sequence_Ring_Peek(0U);
        Sequence_Ring_Consume(1U);
        uint8_t state = (uint8_t)(packed & 0x7FU);
        uint8_t envelope = (uint8_t)((packed >> 7U) & 0xffU);
        sequence_last_state = state;
        Sequence_Render_Sample(waveform, sample, state, envelope, led_mask);
    }
}

static void Sequence_Render_Hold_Block(uint8_t block, int ramp)
{
    DMA_WaveformBlock *waveform = DMA_Sequence_Get_Block(block);
    uint16_t led_mask = Get_Current_LED_Mask();
    for (uint32_t sample = 0U; sample < SEQUENCE_BLOCK_SAMPLES; sample++)
    {
        (void)ramp;
        Sequence_Render_Sample(waveform, sample, sequence_last_state, 0U, led_mask);
    }
}

static int Sequence_Has_Render_Input(void)
{
    return Sequence_Ring_Fill() >= SEQUENCE_BLOCK_SAMPLES;
}

static void Sequence_Record_Render_Time(uint32_t started_cycles)
{
    sequence_rendered_block_count++;
    uint32_t elapsed_cycles = DWT_GetCycles() - started_cycles;
    uint32_t cycles_per_microsecond = SystemCoreClock / 1000000U;
    sequence_current_render_us = cycles_per_microsecond == 0U
        ? 0U : elapsed_cycles / cycles_per_microsecond;
    if (sequence_current_render_us > sequence_maximum_render_us)
        sequence_maximum_render_us = sequence_current_render_us;
    if (sequence_current_render_us >= 4000U)
    {
        sequence_render_timeout_count++;
        sequence_render_over_budget_count++;
    }
}

void Sequence_Init(void)
{
    memset(&sequence_descriptor, 0, sizeof(sequence_descriptor));
    sequence_state = SEQUENCE_STATE_IDLE;
    Sequence_Reset_Stream();
}

int Sequence_Begin_Configuration(const SequenceDescriptor *descriptor)
{
    if (!Sequence_Descriptor_Is_Valid(descriptor)) return 0;

    Sequence_Abort();
    sequence_descriptor = *descriptor;
    sequence_uploaded_states = 0U;
    memset(sequence_phase_offsets, 0, sizeof(sequence_phase_offsets));
    sequence_underrun_count = 0U;
    sequence_overrun_count = 0U;
    sequence_packet_loss_count = 0U;
    sequence_render_timeout_count = 0U;
    sequence_current_render_us = 0U;
    sequence_maximum_render_us = 0U;
    sequence_rendered_block_count = 0U;
    sequence_render_over_budget_count = 0U;
    sequence_dma_deadline_miss_count = 0U;
    Sequence_Reset_Stream();

    Stimulation base = CurrentStimulation;
    base.type = Point;
    memcpy(base.position, descriptor->focus, sizeof(base.position));
    base.strength = 100.0f;
    base.frequency = 200.0f;
    Set_Stimulation(&base);
    Set_Point_Focus(base.position);
    for (uint32_t index = 0U; index < SEQUENCE_OUTPUT_CHANNELS; index++)
        sequence_base_shift[index] = TransducerArray[index].shift_buffer_bits;

    sequence_state = SEQUENCE_STATE_CONFIGURING;
    return 1;
}

int Sequence_Upload_States(uint16_t first_state, uint8_t count,
                           const uint8_t *phase_offsets)
{
    if (sequence_state != SEQUENCE_STATE_CONFIGURING || phase_offsets == NULL ||
        count == 0U || count > SEQUENCE_MAX_UPLOAD_STATES ||
        first_state != sequence_uploaded_states ||
        (uint32_t)first_state + count > sequence_descriptor.state_count)
    {
        return 0;
    }

    uint32_t value_count = (uint32_t)count * SEQUENCE_OUTPUT_CHANNELS;
    for (uint32_t index = 0U; index < value_count; index++)
    {
        if (phase_offsets[index] >= SEQUENCE_PHASE_TICKS) return 0;
    }
    memcpy(&sequence_phase_offsets[first_state][0], phase_offsets, value_count);
    sequence_uploaded_states = (uint16_t)(sequence_uploaded_states + count);
    return 1;
}

int Sequence_Commit(void)
{
    if (sequence_state != SEQUENCE_STATE_CONFIGURING ||
        sequence_uploaded_states != sequence_descriptor.state_count)
    {
        return 0;
    }

    Sequence_Build_Render_Cache();
    sequence_state = SEQUENCE_STATE_BUFFERING;
    return 1;
}

void Sequence_Push_Data(uint32_t packet_sequence, const uint16_t *samples,
                        uint16_t sample_count)
{
    if (samples == NULL || sample_count != SEQUENCE_PACKET_SAMPLES ||
        (sequence_state != SEQUENCE_STATE_BUFFERING &&
         sequence_state != SEQUENCE_STATE_RUNNING &&
         sequence_state != SEQUENCE_STATE_HOLD))
    {
        return;
    }

    if (sequence_packet_valid)
    {
        int32_t difference = (int32_t)(packet_sequence - sequence_expected_packet);
        if (difference < 0)
        {
            sequence_packet_loss_count++;
            return;
        }
        if (difference > 0) sequence_packet_loss_count += (uint32_t)difference;
    }
    sequence_packet_valid = 1U;
    sequence_expected_packet = packet_sequence + 1U;

    if (Sequence_Ring_Free() < sample_count)
    {
        sequence_overrun_count++;
        return;
    }
    for (uint16_t index = 0U; index < sample_count; index++)
    {
        sequence_ring[sequence_ring_head] = samples[index];
        sequence_ring_head = (uint16_t)((sequence_ring_head + 1U) &
                                        (SEQUENCE_RING_CAPACITY - 1U));
    }
}

void Sequence_Task(void)
{
    if (sequence_state == SEQUENCE_STATE_BUFFERING &&
        Sequence_Ring_Fill() >= SEQUENCE_PREBUFFER_SAMPLES)
    {
        uint32_t started_at = DWT_GetCycles();
        Sequence_Render_Data_Block(0U);
        Sequence_Render_Data_Block(1U);
        DMA_Sequence_Clean_Block(0U);
        DMA_Sequence_Clean_Block(1U);
        Sequence_Record_Render_Time(started_at);
        if (!DMA_Sequence_Start())
        {
            sequence_state = SEQUENCE_STATE_FAULT;
            return;
        }
        sequence_state = SEQUENCE_STATE_RUNNING;
        return;
    }

    if (!DMA_Is_Sequence_Active()) return;
    uint32_t deadline_misses = DMA_Sequence_Take_Deadline_Misses();
    sequence_dma_deadline_miss_count += deadline_misses;
    sequence_render_timeout_count += deadline_misses;

    uint8_t block;
    if (!DMA_Sequence_Take_Free_Block(&block)) return;
    uint32_t started_at = DWT_GetCycles();

    if (sequence_state == SEQUENCE_STATE_RUNNING && !Sequence_Has_Render_Input())
    {
        sequence_underrun_count++;
        Sequence_Render_Hold_Block(block, 1);
        sequence_state = SEQUENCE_STATE_HOLD;
    }
    else if (sequence_state == SEQUENCE_STATE_HOLD &&
             Sequence_Ring_Fill() < SEQUENCE_PREBUFFER_SAMPLES)
    {
        Sequence_Render_Hold_Block(block, 0);
    }
    else
    {
        Sequence_Render_Data_Block(block);
        sequence_state = SEQUENCE_STATE_RUNNING;
    }

    Sequence_Record_Render_Time(started_at);
    DMA_Sequence_Release_Block(block);
}

void Sequence_Abort(void)
{
    if (DMA_Is_Sequence_Active())
    {
        DMA_Sequence_Stop();
        Update_Full_Waveform_Buffer();
        Start_DMAs();
    }
    sequence_state = SEQUENCE_STATE_IDLE;
    Sequence_Reset_Stream();
}

int Sequence_Is_Active(void)
{
    return sequence_state != SEQUENCE_STATE_IDLE;
}

void Sequence_Get_Capabilities(SequenceCapabilities *capabilities)
{
    if (capabilities == NULL) return;
    capabilities->input_rate = SEQUENCE_INPUT_RATE;
    capabilities->output_rate = SEQUENCE_OUTPUT_RATE;
    capabilities->block_samples = SEQUENCE_BLOCK_SAMPLES;
    capabilities->prebuffer_samples = SEQUENCE_PREBUFFER_SAMPLES;
    capabilities->max_states = SEQUENCE_MAX_STATES;
    capabilities->phase_ticks = SEQUENCE_PHASE_TICKS;
    capabilities->output_channels = SEQUENCE_OUTPUT_CHANNELS;
}

void Sequence_Get_Status(SequenceStatus *status, uint32_t comm_rx_dropped_bytes)
{
    if (status == NULL) return;
    status->state = (uint8_t)sequence_state;
    status->mapping = sequence_descriptor.mapping;
    status->ring_fill = Sequence_Ring_Fill();
    status->underrun_count = sequence_underrun_count;
    status->overrun_count = sequence_overrun_count;
    status->packet_loss_count = sequence_packet_loss_count;
    status->comm_rx_dropped_bytes = comm_rx_dropped_bytes;
    status->render_timeout_count = sequence_render_timeout_count;
    status->clock_correction_ppm = sequence_clock_correction_ppm;
    status->current_render_us = sequence_current_render_us;
    status->maximum_render_us = sequence_maximum_render_us;
    status->rendered_block_count = sequence_rendered_block_count;
    status->render_over_budget_count = sequence_render_over_budget_count;
    status->dma_deadline_miss_count = sequence_dma_deadline_miss_count;
    status->render_mode = sequence_uniform_phase != 0U ? 1U : 0U;
}
