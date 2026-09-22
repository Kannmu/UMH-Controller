#ifndef UMH_FAST_MATH_H
#define UMH_FAST_MATH_H

#include <stdint.h>

/* Fixed-point quarter-turn sine table shared by the real-time renderers.
 *
 * The ultrasound frame is phase-quantised to 1/256 carrier turn (8 bit) and
 * the FPGA takes even that value as a byte.  A 1024-entry table therefore
 * has four times the phase resolution of the wire format, which keeps the
 * complex accumulation error far below one output LSB while avoiding the
 * ~200-cycle newlib sinf()/cosf() calls that dominated the motion engine.
 *
 * phase_q10 is a signed phase in 1/1024 turn units; negative and
 * out-of-range values wrap exactly like the hardware phase accumulator. */
#define UMH_FAST_TRIG_SIZE 1024u
#define UMH_FAST_TRIG_MASK (UMH_FAST_TRIG_SIZE - 1u)

/* Non-const so the linker places it in .data: the C runtime copies it to RAM
 * at boot.  The renderer reads it 168..1344 times per frame and random flash
 * data accesses were measurably more expensive than the 2 KiB of RAM. */
extern int16_t umh_fast_sin_table[UMH_FAST_TRIG_SIZE];

static inline float umh_fast_sin_q10(int32_t phase_q10)
{
  return (float)umh_fast_sin_table[(uint32_t)phase_q10 & UMH_FAST_TRIG_MASK] *
         (1.0f / 32768.0f);
}

static inline float umh_fast_cos_q10(int32_t phase_q10)
{
  return (float)umh_fast_sin_table[((uint32_t)phase_q10 + (UMH_FAST_TRIG_SIZE / 4u)) &
                                   UMH_FAST_TRIG_MASK] * (1.0f / 32768.0f);
}

#endif
