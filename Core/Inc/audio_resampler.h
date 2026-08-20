#pragma once

#include <stdint.h>

/* Streaming 48 kHz mono PCM to the 40 kHz carrier update rate. */
typedef struct
{
    uint32_t phase_q16;
    uint32_t step_q16;
    int16_t previous;
    uint8_t primed;
} AudioResampler;

void Audio_Resampler_Init(AudioResampler *resampler);
int16_t Audio_Resampler_Interpolate(AudioResampler *resampler,
                                    int16_t current, int16_t next,
                                    uint8_t *consume_current);
