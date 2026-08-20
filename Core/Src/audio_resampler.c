#include "audio_resampler.h"

void Audio_Resampler_Init(AudioResampler *resampler)
{
    if (resampler == 0) return;
    resampler->phase_q16 = 0U;
    /* 48 kHz / 40 kHz = 6 / 5 input samples per output sample. */
    resampler->step_q16 = (6U << 16) / 5U;
    resampler->previous = 0;
    resampler->primed = 0U;
}

int16_t Audio_Resampler_Interpolate(AudioResampler *resampler,
                                    int16_t current, int16_t next,
                                    uint8_t *consume_current)
{
    int32_t delta;
    int32_t value;

    if (resampler == 0 || consume_current == 0) return 0;
    delta = (int32_t)next - (int32_t)current;
    value = (int32_t)current + ((delta * (int32_t)resampler->phase_q16) >> 16);

    resampler->phase_q16 += resampler->step_q16;
    *consume_current = (uint8_t)(resampler->phase_q16 >> 16);
    resampler->phase_q16 &= 0xFFFFU;
    if (value > 32767) value = 32767;
    if (value < -32768) value = -32768;
    return (int16_t)value;
}
