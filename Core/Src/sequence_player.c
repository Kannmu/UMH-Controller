#include "sequence_player.h"

#include <math.h>
#include <string.h>

#include "dma_manager.h"
#include "sequence_math.h"
#include "stimulation.h"
#include "transducer.h"
#include "utiles.h"

static uint8_t sequence_phase_offsets[SEQUENCE_MAX_STATES][SEQUENCE_OUTPUT_CHANNELS];
/* Carrier pulse starts are tiny: 100 states x 60 transducers x one byte. */
static uint8_t sequence_phase_starts[SEQUENCE_RENDER_STATES][SEQUENCE_OUTPUT_CHANNELS];
static uint32_t sequence_ring[SEQUENCE_RING_CAPACITY];
static uint16_t sequence_base_shift[SEQUENCE_OUTPUT_CHANNELS];
static SequenceDescriptor sequence_descriptor;
static SequenceState sequence_state;
static uint16_t sequence_uploaded_states;
static uint16_t sequence_ring_head;
static uint16_t sequence_ring_tail;
static uint8_t sequence_last_left_state;
static uint8_t sequence_last_right_state;
static uint8_t sequence_packet_valid;
static uint32_t sequence_expected_packet;
static uint32_t sequence_underrun_count;
static uint32_t sequence_overrun_count;
static uint32_t sequence_packet_loss_count;
static uint32_t sequence_render_timeout_count;
static int32_t sequence_clock_correction_ppm;
static uint32_t sequence_current_render_us;
static uint32_t sequence_maximum_render_us;

static uint16_t Sequence_Ring_Fill(void)
{
    return (uint16_t)((sequence_ring_head - sequence_ring_tail) &
                      (SEQUENCE_RING_CAPACITY - 1U));
}

static uint16_t Sequence_Ring_Free(void)
{
    return (uint16_t)((SEQUENCE_RING_CAPACITY - 1U) - Sequence_Ring_Fill());
}

static uint32_t Sequence_Ring_Peek(uint16_t offset)
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
    sequence_last_left_state = sequence_descriptor.neutral_state;
    sequence_last_right_state = sequence_descriptor.neutral_state;
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
    for (uint32_t index = 0U; index < SEQUENCE_OUTPUT_CHANNELS; index++)
    {
        const Transducer *transducer = &TransducerArray[index];
        if (transducer->port_num >= DMA_CHANNELS) continue;
        for (uint16_t state = 0U; state < sequence_descriptor.state_count; state++)
        {
            uint32_t phase = (uint32_t)transducer->calib + sequence_base_shift[index] +
                sequence_phase_offsets[state][index] +
                (uint32_t)(GPIO_Group_Output_Offset[transducer->port_num] *
                           BufferGapPerMicroseconds);
            phase %= WAVEFORM_BUFFER_SIZE;
            sequence_phase_starts[state][index] =
                (uint8_t)((WAVEFORM_BUFFER_SIZE - phase) % WAVEFORM_BUFFER_SIZE);
        }
    }
}

static void Sequence_Render_Sample(DMA_WaveformBlock *waveform, uint32_t sample,
                                   uint16_t left_state, uint16_t right_state,
                                   uint8_t left_envelope, uint8_t right_envelope)
{
    uint16_t led_mask = Get_Current_LED_Mask();
    uint32_t left_duty = ((uint32_t)left_envelope * (WAVEFORM_BUFFER_SIZE / 2U) + 127U) / 255U;
    uint32_t right_duty = ((uint32_t)right_envelope * (WAVEFORM_BUFFER_SIZE / 2U) + 127U) / 255U;
    uint16_t turn_on[WAVEFORM_BUFFER_SIZE];
    uint16_t turn_off[WAVEFORM_BUFFER_SIZE];

    for (uint32_t port = 0U; port < DMA_CHANNELS; port++)
    {
        memset(turn_on, 0, sizeof(turn_on));
        memset(turn_off, 0, sizeof(turn_off));
        uint16_t running_state = 0U;

        /* Build only pulse-boundary events. The expensive tick loop below is
         * per GPIO port, not per transducer, so the full envelope remains
         * phase-aware without multiplying the DMA deadline cost by 60. */
        for (uint32_t index = 0U; index < SEQUENCE_OUTPUT_CHANNELS; index++)
        {
            const Transducer *transducer = &TransducerArray[index];
            if (transducer->port_num != port) continue;

            uint32_t duty = transducer->position3D[0] < 0.0f
                ? left_duty : right_duty;
            if (duty == 0U) continue;
            uint16_t state = transducer->position3D[0] < 0.0f
                ? left_state : right_state;
            uint32_t start = sequence_phase_starts[state][index];
            uint32_t end = (start + duty) % WAVEFORM_BUFFER_SIZE;
            turn_on[start] |= transducer->pin;
            turn_off[end] |= transducer->pin;
            if (start >= end) running_state |= transducer->pin;
        }

        for (uint32_t tick = 0U; tick < WAVEFORM_BUFFER_SIZE; tick++)
        {
            running_state |= turn_on[tick];
            running_state &= (uint16_t)~turn_off[tick];
            (*waveform)[port][sample][tick] = running_state |
                (port == 0U ? led_mask : 0U);
        }
    }
}

static void Sequence_Render_Data_Block(uint8_t block)
{
    DMA_WaveformBlock *waveform = DMA_Sequence_Get_Block(block);
    sequence_clock_correction_ppm = 0;
    for (uint32_t sample = 0U; sample < SEQUENCE_BLOCK_SAMPLES; sample++)
    {
        uint32_t packed = Sequence_Ring_Peek(0U);
        Sequence_Ring_Consume(1U);
        uint8_t left_state = (uint8_t)(packed & 0x7FU);
        uint8_t right_state = (uint8_t)((packed >> 7U) & 0x7FU);
        uint8_t left_envelope = (uint8_t)((packed >> 14U) & 0xffU);
        uint8_t right_envelope = (uint8_t)((packed >> 22U) & 0xffU);
        sequence_last_left_state = left_state;
        sequence_last_right_state = right_state;
        Sequence_Render_Sample(waveform, sample, left_state, right_state,
                               left_envelope, right_envelope);
    }
}

static void Sequence_Render_Hold_Block(uint8_t block, int ramp)
{
    DMA_WaveformBlock *waveform = DMA_Sequence_Get_Block(block);
    for (uint32_t sample = 0U; sample < SEQUENCE_BLOCK_SAMPLES; sample++)
    {
        (void)ramp;
        Sequence_Render_Sample(waveform, sample, sequence_last_left_state,
                               sequence_last_right_state, 0U, 0U);
    }
}

static int Sequence_Has_Render_Input(void)
{
    return Sequence_Ring_Fill() >= SEQUENCE_BLOCK_SAMPLES;
}

static void Sequence_Record_Render_Time(uint32_t started_at)
{
    sequence_current_render_us = DWT_GetMicroseconds() - started_at;
    if (sequence_current_render_us > sequence_maximum_render_us)
        sequence_maximum_render_us = sequence_current_render_us;
    if (sequence_current_render_us >= 4000U) sequence_render_timeout_count++;
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

void Sequence_Push_Data(uint32_t packet_sequence, const uint32_t *samples,
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
        uint32_t started_at = DWT_GetMicroseconds();
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
    sequence_render_timeout_count += DMA_Sequence_Take_Deadline_Misses();

    uint8_t block;
    if (!DMA_Sequence_Take_Free_Block(&block)) return;
    uint32_t started_at = DWT_GetMicroseconds();

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
}
