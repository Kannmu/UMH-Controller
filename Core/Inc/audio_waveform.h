#pragma once

#include <stdint.h>

void Audio_Waveform_Render(uint8_t block,
                           const int16_t *samples,
                           uint32_t sample_count,
                           float level);
