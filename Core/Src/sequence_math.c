#include "sequence_math.h"

#define Q16_ONE 65536LL

static int16_t Clamp_Int16(int64_t value)
{
    if (value > 32767LL) return 32767;
    if (value < -32768LL) return -32768;
    return (int16_t)value;
}

int16_t Sequence_Cubic_Interpolate(int16_t previous, int16_t current,
                                   int16_t next, int16_t following,
                                   uint16_t fraction_q16)
{
    int64_t t = fraction_q16;
    int64_t t2 = (t * t) >> 16;
    int64_t t3 = (t2 * t) >> 16;
    int64_t a = -(int64_t)previous + 3LL * current - 3LL * next + following;
    int64_t b = 2LL * previous - 5LL * current + 4LL * next - following;
    int64_t c = -(int64_t)previous + next;
    int64_t value_q16 = (2LL * current * Q16_ONE) + c * t + b * t2 + a * t3;
    int64_t rounded = value_q16 >= 0 ? value_q16 + Q16_ONE : value_q16 - Q16_ONE;
    return Clamp_Int16(rounded / (2LL * Q16_ONE));
}

int32_t Sequence_Clock_Correction_Ppm(uint16_t ring_fill, uint16_t target_fill)
{
    int32_t correction = ((int32_t)ring_fill - (int32_t)target_fill) * 4;
    if (correction > 1000) return 1000;
    if (correction < -1000) return -1000;
    return correction;
}

uint32_t Sequence_Resampler_Step_Q16(int32_t correction_ppm)
{
    const uint32_t base_step_q16 = (6U << 16U) / 5U;
    return (uint32_t)(((uint64_t)base_step_q16 *
                       (uint32_t)(1000000 + correction_ppm)) / 1000000U);
}

uint32_t Sequence_Map_Cyclic_Q16(uint32_t coordinate_q16, int16_t control,
                                 int32_t scale_q16, uint16_t state_count)
{
    if (state_count == 0U) return 0U;

    int64_t denominator = control < 0 ? 32768LL : 32767LL;
    int64_t delta = ((int64_t)control * scale_q16) / denominator;
    int64_t modulus = (int64_t)state_count << 16U;
    int64_t coordinate = (int64_t)coordinate_q16 + delta;
    coordinate %= modulus;
    if (coordinate < 0) coordinate += modulus;
    return (uint32_t)coordinate;
}

uint16_t Sequence_Map_Absolute(int16_t control, int32_t scale_q16,
                               uint8_t neutral_state, uint16_t state_count)
{
    if (state_count == 0U) return 0U;

    int64_t denominator = control < 0 ? 32768LL : 32767LL;
    int64_t offset_q16 = ((int64_t)control * scale_q16) / denominator;
    int64_t coordinate_q16 = ((int64_t)neutral_state << 16U) + offset_q16;
    int64_t maximum_q16 = ((int64_t)state_count - 1LL) << 16U;
    if (coordinate_q16 < 0) coordinate_q16 = 0;
    if (coordinate_q16 > maximum_q16) coordinate_q16 = maximum_q16;
    return (uint16_t)((coordinate_q16 + 32768LL) >> 16U);
}
