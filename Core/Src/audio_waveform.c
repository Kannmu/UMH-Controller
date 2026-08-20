#include "audio_waveform.h"
#include "dma_manager.h"
#include "transducer.h"
#include "utiles.h"

static uint16_t Audio_Duty_To_Ticks(float sample, float level)
{
    static float quantization_error;
    float duty = 25.0f + (sample / 32768.0f) * 25.0f * level + quantization_error;
    if (duty < 0.0f) duty = 0.0f;
    if (duty > (float)(WAVEFORM_BUFFER_SIZE / 2U)) duty = (float)(WAVEFORM_BUFFER_SIZE / 2U);
    uint16_t ticks = (uint16_t)duty;
    quantization_error = duty - (float)ticks;
    return ticks;
}

void Audio_Waveform_Render(uint8_t block,
                           const int16_t *samples,
                           uint32_t sample_count,
                           float level)
{
    DMA_WaveformBlock *waveform = DMA_Audio_Get_Block(block);
    uint16_t led_mask = Get_Current_LED_Mask();

    if (waveform == 0 || samples == 0 || sample_count > NUM_STIMULATION_SAMPLES) return;

    for (uint32_t s = 0; s < NUM_STIMULATION_SAMPLES; s++)
    {
        int16_t sample = (s < sample_count) ? samples[s] : 0;
        uint16_t duty = Audio_Duty_To_Ticks((float)sample, level);

        for (uint32_t p = 0; p < DMA_CHANNELS; p++)
        {
            uint16_t *destination = &(*waveform)[p][s][0];
            uint16_t turn_on[WAVEFORM_BUFFER_SIZE] = {0};
            uint16_t turn_off[WAVEFORM_BUFFER_SIZE] = {0};
            uint16_t current_state = 0U;

            for (uint32_t t = 0; t < NUM_TRANSDUCER - 1U; t++)
            {
                const Transducer *transducer = &TransducerArray[t];
                if (transducer->port_num != p || duty == 0U) continue;
                uint32_t phase = ((uint32_t)transducer->calib +
                                  transducer->shift_buffer_bits +
                                  (uint32_t)(GPIO_Group_Output_Offset[p] * BufferGapPerMicroseconds)) %
                                 WAVEFORM_BUFFER_SIZE;
                uint32_t start = (WAVEFORM_BUFFER_SIZE - phase) % WAVEFORM_BUFFER_SIZE;
                uint32_t end = (start + duty) % WAVEFORM_BUFFER_SIZE;
                turn_on[start] |= transducer->pin;
                turn_off[end] |= transducer->pin;
                if (start >= end) current_state |= transducer->pin;
            }

            uint16_t running_state = current_state;
            for (uint32_t tick = 0; tick < WAVEFORM_BUFFER_SIZE; tick++)
            {
                running_state |= turn_on[tick];
                running_state &= (uint16_t)~turn_off[tick];
                destination[tick] = running_state | ((p == 0U) ? led_mask : 0U);
            }
        }
    }
    DMA_Audio_Clean_Block(block);
}
