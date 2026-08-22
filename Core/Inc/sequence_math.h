#pragma once

#include <stdint.h>

int16_t Sequence_Cubic_Interpolate(int16_t previous, int16_t current,
                                   int16_t next, int16_t following,
                                   uint16_t fraction_q16);
void Sequence_Clock_Correction_Reset(void);
int32_t Sequence_Clock_Correction_Ppm(uint16_t ring_fill, uint16_t target_fill);
uint32_t Sequence_Resampler_Step_Q16(int32_t correction_ppm);
uint32_t Sequence_Map_Cyclic_Q16(uint32_t coordinate_q16, int16_t control,
                                 int32_t scale_q16, uint16_t state_count);
uint16_t Sequence_Map_Absolute(int16_t control, int32_t scale_q16,
                               uint8_t neutral_state, uint16_t state_count);
