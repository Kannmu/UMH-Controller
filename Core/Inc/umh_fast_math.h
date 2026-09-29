#ifndef UMH_FAST_MATH_H
#define UMH_FAST_MATH_H

#include <stdint.h>
#include <math.h>

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

/* Hardware single-precision square root.
 *
 * newlib's sqrtf() is an out-of-line call even with -fno-math-errno: it saves
 * a register pair, calls __ieee754_sqrtf (which is one VSQRT), then re-tests
 * the input for NaN and for the negative-domain errno case and branches to
 * __errno().  On Cortex-M4 the VSQRT itself is 14 cycles and the wrapper costs
 * roughly as much again, which the renderers pay 84..1344 times per frame.
 * VSQRT.F32 does not raise anything a bare-metal build acts on, so calling it
 * directly is exactly the same numeric result with none of the call overhead.
 *
 * The inputs here are sums of squares of finite single-precision values, so a
 * negative operand is impossible; a NaN input (from an inf coordinate) still
 * propagates as a NaN instead of touching errno. */
static inline float umh_sqrtf(float x)
{
#if defined(__arm__) && defined(__VFP_FP__)
  float result;
  __asm__ ("vsqrt.f32 %0, %1" : "=t" (result) : "t" (x));
  return result;
#else
  return sqrtf(x);
#endif
}

/* |(re, im)| in one VSQRT plus the two FMAs that build the sum of squares. */
static inline float umh_hypotf(float re, float im)
{
  return umh_sqrtf(fmaf(re, re, im * im));
}

#endif
