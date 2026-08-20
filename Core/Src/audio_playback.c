#include "audio_playback.h"
#include "audio_resampler.h"
#include "audio_waveform.h"
#include "dma_manager.h"
#include "stimulation.h"
#include "transducer.h"
#include "calibration.h"
#include "utiles.h"

static int16_t audio_ring[AUDIO_RING_CAPACITY] __attribute__((section(".audio_ring"), aligned(32)));
static volatile uint16_t audio_head;
static volatile uint16_t audio_tail;
static AudioResampler audio_resampler;
static AudioState audio_state;
static float audio_level;
static float audio_target_level;
static uint8_t audio_muted;
static uint8_t audio_last_playing_block;
static uint32_t audio_drain_start;
static uint32_t underrun_count;
static uint32_t overrun_count;
static uint32_t usb_reset_count;
static uint32_t packet_loss_count;
static int32_t audio_clock_correction_ppm;
static uint16_t saved_shift[NUM_TRANSDUCER];
static uint8_t saved_shift_valid;

static uint16_t Audio_Ring_Fill(void)
{
    uint16_t head = audio_head;
    uint16_t tail = audio_tail;
    return (uint16_t)((head - tail) & (AUDIO_RING_CAPACITY - 1U));
}

static int Audio_Ring_Push(int16_t sample)
{
    uint16_t head = audio_head;
    uint16_t next = (uint16_t)((head + 1U) & (AUDIO_RING_CAPACITY - 1U));
    if (next == audio_tail)
    {
        overrun_count++;
        return 0;
    }
    audio_ring[head] = sample;
    __DMB();
    audio_head = next;
    return 1;
}

static int Audio_Ring_Peek(uint16_t offset, int16_t *sample)
{
    if (sample == 0 || offset >= Audio_Ring_Fill()) return 0;
    *sample = audio_ring[(audio_tail + offset) & (AUDIO_RING_CAPACITY - 1U)];
    return 1;
}

static void Audio_Ring_Consume(uint16_t count)
{
    uint16_t available = Audio_Ring_Fill();
    if (count > available) count = available;
    __DMB();
    audio_tail = (uint16_t)((audio_tail + count) & (AUDIO_RING_CAPACITY - 1U));
}

static int16_t Audio_Next_Output_Sample(void)
{
    int16_t current = 0;
    int16_t next = 0;
    uint8_t consumed = 0U;

    if (!Audio_Ring_Peek(0U, &current))
    {
        underrun_count++;
        return 0;
    }
    if (!Audio_Ring_Peek(1U, &next)) next = current;
    int16_t output = Audio_Resampler_Interpolate(&audio_resampler, current, next, &consumed);
    Audio_Ring_Consume(consumed);
    return output;
}

static void Audio_Render_Block(uint8_t block)
{
    int16_t samples[NUM_STIMULATION_SAMPLES];
    float level = audio_muted ? 0.0f : audio_level;

    if (audio_level < audio_target_level) audio_level += 0.05f;
    if (audio_level > audio_target_level) audio_level -= 0.05f;
    if (audio_level < 0.0f) audio_level = 0.0f;
    if (audio_level > 1.0f) audio_level = 1.0f;

    for (uint32_t i = 0; i < NUM_STIMULATION_SAMPLES; i++)
        samples[i] = Audio_Next_Output_Sample();
    Audio_Waveform_Render(block, samples, NUM_STIMULATION_SAMPLES, level);
}

static void Audio_Render_Initial_Carrier(void)
{
    int16_t silence[NUM_STIMULATION_SAMPLES] = {0};

    Audio_Waveform_Render(0U, silence, NUM_STIMULATION_SAMPLES, 0.0f);
    Audio_Waveform_Render(1U, silence, NUM_STIMULATION_SAMPLES, 0.0f);
}

void Audio_Init(void)
{
    audio_head = 0U;
    audio_tail = 0U;
    audio_state = AUDIO_STATE_LEGACY;
    audio_level = 0.0f;
    audio_target_level = 1.0f;
    audio_muted = 0U;
    saved_shift_valid = 0U;
    Audio_Resampler_Init(&audio_resampler);
}

int Audio_Is_Active(void)
{
    return audio_state != AUDIO_STATE_LEGACY;
}

AudioState Audio_Get_State(void)
{
    return audio_state;
}

int Audio_Enter(void)
{
    if (audio_state != AUDIO_STATE_LEGACY || Get_Calibration_Mode()) return 0;

    for (uint32_t i = 0; i < NUM_TRANSDUCER; i++) saved_shift[i] = TransducerArray[i].shift_buffer_bits;
    saved_shift_valid = 1U;
    audio_head = 0U;
    audio_tail = 0U;
    Audio_Resampler_Init(&audio_resampler);
    audio_level = 0.0f;
    audio_target_level = 1.0f;
    audio_muted = 0U;
    audio_state = AUDIO_STATE_PREPARE;
    if (!DMA_Audio_Prepare())
    {
        audio_state = AUDIO_STATE_FAULT;
        return 0;
    }
    Audio_Render_Initial_Carrier();
    if (!DMA_Audio_Start())
    {
        audio_state = AUDIO_STATE_FAULT;
        return 0;
    }
    audio_last_playing_block = DMA_Audio_Get_Playing_Block();
    return 1;
}

void Audio_Exit(void)
{
    if (audio_state == AUDIO_STATE_LEGACY) return;
    audio_target_level = 0.0f;
    audio_drain_start = HAL_GetTick();
    audio_state = AUDIO_STATE_DRAINING;
}

int Audio_Set_Focus(const float position[3])
{
    if (!Audio_Is_Active() || position == 0) return 0;
    Set_Point_Focus((float *)position);
    return 1;
}

void Audio_Set_Level(float level)
{
    if (!isfinite(level)) return;
    if (level < 0.0f) level = 0.0f;
    if (level > 1.0f) level = 1.0f;
    audio_target_level = level;
}

void Audio_Set_Mute(int mute)
{
    audio_muted = mute ? 1U : 0U;
}

void Audio_Push_PCM(const uint8_t *data, uint32_t length)
{
    if (data == 0 || audio_state == AUDIO_STATE_LEGACY) return;
    if ((length & 1U) != 0U)
    {
        packet_loss_count++;
        length--;
    }
    for (uint32_t i = 0; i < length; i += 2U)
    {
        int16_t sample = (int16_t)((uint16_t)data[i] | ((uint16_t)data[i + 1U] << 8U));
        (void)Audio_Ring_Push(sample);
    }
    if (audio_state == AUDIO_STATE_DRAINING)
    {
        audio_state = AUDIO_STATE_RUNNING;
        audio_target_level = 1.0f;
    }
}

void Audio_Task(void)
{
    if (audio_state == AUDIO_STATE_LEGACY) return;

    if (audio_state == AUDIO_STATE_DRAINING &&
        (HAL_GetTick() - audio_drain_start) >= 20U)
    {
        if (!DMA_Audio_Stop())
        {
            audio_state = AUDIO_STATE_FAULT;
            return;
        }
        if (saved_shift_valid)
        {
            for (uint32_t i = 0; i < NUM_TRANSDUCER; i++) TransducerArray[i].shift_buffer_bits = saved_shift[i];
        }
        saved_shift_valid = 0U;
        audio_head = 0U;
        audio_tail = 0U;
        audio_state = AUDIO_STATE_LEGACY;
        Update_Full_Waveform_Buffer();
        Start_DMAs();
        return;
    }

    /* Keep the producer/consumer midpoint stable without changing the carrier clock. */
    int32_t fill_error = (int32_t)Audio_Ring_Fill() - (int32_t)(AUDIO_RING_CAPACITY / 2U);
    audio_clock_correction_ppm = fill_error / 4;
    if (audio_clock_correction_ppm > 500) audio_clock_correction_ppm = 500;
    if (audio_clock_correction_ppm < -500) audio_clock_correction_ppm = -500;
    audio_resampler.step_q16 = (uint32_t)(((uint64_t)((6U << 16) / 5U) *
                                           (1000000 + audio_clock_correction_ppm)) / 1000000U);

    if (audio_state == AUDIO_STATE_PREPARE && Audio_Ring_Fill() >= AUDIO_PREBUFFER_SAMPLES)
        audio_state = AUDIO_STATE_RUNNING;

    uint8_t playing = DMA_Audio_Get_Playing_Block();
    if (playing != audio_last_playing_block)
    {
        if (audio_state != AUDIO_STATE_PREPARE)
            Audio_Render_Block(audio_last_playing_block);
        audio_last_playing_block = playing;
    }
}

void Audio_Get_Status(AudioStatus *status)
{
    if (status == 0) return;
    status->state = (uint8_t)audio_state;
    status->muted = audio_muted;
    status->ring_fill = Audio_Ring_Fill();
    status->underrun_count = underrun_count;
    status->overrun_count = overrun_count;
    status->usb_reset_count = usb_reset_count;
    status->packet_loss_count = packet_loss_count;
    status->clock_correction_ppm = (int32_t)audio_clock_correction_ppm;
}
