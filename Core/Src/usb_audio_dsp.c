#include "usb_audio_dsp.h"
#include <string.h>
#include <math.h>
#if defined(__arm__) || defined(__thumb__)
#include "cmsis_gcc.h"
#endif

/* SWD diagnostic: 1 bypasses the limiter/AGC and maps the FIR output with a
 * fixed gain, to separate DSP dynamics from the resampler/feed path. */
volatile uint8_t umh_usb_audio_dsp_bypass;


/* 48 kHz -> 20 kHz, rational 5/12 resampler.  A 48-tap Kaiser (beta=6)
 * 8.4 kHz low-pass per fractional phase: 0.3 dB loss at 7 kHz, 3.2 dB at
 * 8 kHz, 12.8 dB at 9 kHz and 37 dB at 10 kHz.  The previous table cut at
 * 7.5 kHz (-9.5 dB at 8 kHz, -30 dB at 9 kHz), which removed the top octave
 * that the CDC focused-AM path (8 kHz baseband biquad) still carries.  Each
 * row sums to 32768; its L1 norm is <=55420, so even full-scale PCM with
 * adversarial signs cannot overflow int32_t.  Only output instants run the
 * FIR: 24 M4 SMLADs/output, 480k SMLADs/second.  FMAC remains available to
 * calibration; there are no peripheral/DMA locks. */
static const int16_t coefficients[5][USB_AUDIO_DSP_TAPS]
    = {
  {-1,2,1,-4,-4,5,11,-1,-21,-14,25,47,-1,-80,-75,58,179,93,-182,-338,-93,390,554,54,-706,-843,21,1139,1232,-104,-1700,-1804,77,2377,2762,393,-3021,-4522,-2388,2443,7195,9405,8518,5779,2941,1067,239,19},
  {-1,2,2,-3,-6,2,11,3,-17,-18,15,45,13,-65,-84,25,163,129,-124,-331,-172,299,571,197,-576,-897,-208,962,1335,239,-1451,-1954,-405,2005,2913,1034,-2399,-4491,-3095,1385,6389,9222,8899,6387,3452,1349,338,37},
  {-1,1,2,-2,-5,1,10,5,-15,-21,9,45,24,-54,-93,-3,149,158,-69,-318,-240,206,568,325,-433,-915,-418,756,1382,555,-1159,-2023,-849,1574,2950,1609,-1726,-4316,-3673,352,5494,8907,9189,6978,3999,1679,468,64},
  {0,0,2,-2,-4,2,10,5,-16,-24,7,48,35,-48,-104,-28,138,188,-17,-304,-302,115,554,438,-288,-908,-605,538,1383,835,-842,-2017,-1239,1110,2881,2095,-1033,-4011,-4105,-628,4529,8458,9372,7539,4579,2059,631,102},
  {1,-1,0,-2,-2,4,10,3,-19,-25,8,54,43,-45,-117,-50,131,215,30,-290,-358,28,533,538,-145,-880,-765,319,1343,1074,-519,-1946,-1564,638,2718,2480,-348,-3597,-4383,-1525,3519,7877,9431,8051,5182,2490,832,155}
};

static inline int32_t fir_q15(const int16_t *history, const int16_t *coeff)
{
  int32_t sum = 0;
  uint32_t i;
  for (i = 0u; i < USB_AUDIO_DSP_TAPS; i += 2u) {
    /* Scalar path on the MCU as well: the previous __SMLAD/memcpy form did an
     * unaligned 32-bit load from history + i when head was odd, which is the
     * prime suspect for the intermittent 40..70 LSB level jumps seen on the
     * MCU but not in the PC harness. */
    sum += (int32_t)history[i] * coeff[i];
    sum += (int32_t)history[i + 1u] * coeff[i + 1u];
  }
  sum = (sum + 16384) >> 15;
#if defined(__arm__) || defined(__thumb__)
  return __SSAT(sum, 16);
#else
  return sum > 32767 ? 32767 : (sum < -32768 ? -32768 : sum);
#endif
}


/* The dynamic stages are a float port of the UMH Speaker reference chain:
 * AudioSignalProcessor (5 ms look-ahead limiter, 0.98 ceiling) followed by
 * ParametricAmModulator (5 ms peak AGC, 0.655 programme peak, 68.7 %
 * modulation).  The FIR/DC blocker stay fixed point; the limiter, AGC and
 * level use the FPU so their release curves match the host exactly.  The
 * previous Q16 version used a constant slew for the limiter release and a
 * partial peak window for the AGC; both only misbehave on real programme
 * material, which is why a steady tone always looked and sounded clean. */
#define DSP_DC_POLE 0.993732f          /* 20 Hz at 20 kHz */
#define DSP_PREGAIN 3.98107f           /* +12 dB, the app's gainDb=12 */
#define DSP_LIMIT_CEILING 0.98f
#define DSP_LIMITER_RELEASE 0.001f     /* 50 ms one-pole */
#define DSP_PROGRAMME_PEAK 0.655f      /* 0.9 * 0.5 / 0.687 */
#define DSP_MODULATION 0.687f          /* depth 12/16 */
#define DSP_AGC_RISE 0.0005f
#define DSP_AGC_FALL 0.001f
#define DSP_GATE_FULL 80.0f

/* The limiter look-ahead is CPU-only and lives in CCM: the float window would
 * not fit main RAM together with the AGC window.  One DSP instance exists. */
static float dsp_lookahead[USB_AUDIO_DSP_LOOKAHEAD]
    __attribute__((section(".ccmram")));

/* Sliding-window maxima.
 *
 * Both dynamic stages need the peak of a ring of recent samples on every
 * output instant: 96 samples for the limiter, 64 for the AGC, i.e. 160
 * data-dependent float compares (each with an FPU->APSR transfer and a branch)
 * per 20 kHz output sample.  That scan was the dominant cost of USB audio
 * playback.
 *
 * The maxima are therefore maintained per 16-sample bucket.  A write only
 * touches its own bucket, and the bucket is rescanned only when the value it
 * loses was that bucket's maximum, so the amortised cost is 6+4 compares per
 * output sample plus a rare 16-entry rescan.  The comparison runs on the
 * magnitude bit patterns: for IEEE-754 single precision and non-negative
 * values the bit pattern order equals the numeric order, so the maxima are
 * bit-identical to the full scans this replaces (the windows are cleared to
 * +0.0, whose pattern is the minimum). */
#define DSP_BUCKET_SHIFT 4u
#define DSP_BUCKET_SIZE (1u << DSP_BUCKET_SHIFT)
#define DSP_LOOKAHEAD_BUCKETS (USB_AUDIO_DSP_LOOKAHEAD / DSP_BUCKET_SIZE)
#define DSP_AGC_BUCKETS (USB_AUDIO_DSP_PEAK_WINDOW / DSP_BUCKET_SIZE)

static uint32_t lookahead_max[DSP_LOOKAHEAD_BUCKETS];
static uint32_t agc_max[DSP_AGC_BUCKETS];

static inline uint32_t magnitude_bits(float value)
{
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  return bits & 0x7FFFFFFFu;
}

static inline float bits_value(uint32_t bits)
{
  float value;
  memcpy(&value, &bits, sizeof(value));
  return value;
}

/* window[pos] = value, keeping buckets[] an exact per-bucket magnitude max. */
static inline void window_write(float *window, uint32_t *buckets,
                                uint16_t pos, float value)
{
  uint16_t bucket = (uint16_t)(pos >> DSP_BUCKET_SHIFT);
  uint32_t previous = magnitude_bits(window[pos]);
  uint32_t current = magnitude_bits(value);
  window[pos] = value;
  if (current >= buckets[bucket]) {
    buckets[bucket] = current;
    return;
  }
  if (previous < buckets[bucket]) return;
  {
    uint16_t first = (uint16_t)(bucket << DSP_BUCKET_SHIFT);
    uint16_t i;
    uint32_t peak = 0u;
    for (i = 0u; i < DSP_BUCKET_SIZE; ++i) {
      uint32_t candidate = magnitude_bits(window[first + i]);
      if (candidate > peak) peak = candidate;
    }
    buckets[bucket] = peak;
  }
}

static inline uint32_t window_peak(const uint32_t *buckets, uint16_t count)
{
  uint16_t i;
  uint32_t peak = 0u;
  for (i = 0u; i < count; ++i)
    if (buckets[i] > peak) peak = buckets[i];
  return peak;
}

void usb_audio_dsp_reset(usb_audio_dsp_t *dsp)
{
  memset(dsp, 0, sizeof(*dsp));
  memset(dsp_lookahead, 0, sizeof(dsp_lookahead));
  memset(lookahead_max, 0, sizeof(lookahead_max));
  memset(agc_max, 0, sizeof(agc_max));
  dsp->limiter_gain = 1.0f;
  dsp->agc_gain = 1.0f;
}

uint16_t usb_audio_dsp_process(usb_audio_dsp_t *dsp, const int16_t *pcm,
                               uint16_t input_count, uint16_t gain_q15,
                               uint8_t *envelope)
{
  uint16_t n, written = 0u;
  (void)gain_q15;
  for (n = 0u; n < input_count; ++n) {
    int32_t fir;
    uint8_t phase;
    float input, value, delayed, peak, desired, limited, magnitude;
    float peak2, target, am;

    dsp->history[dsp->head] = pcm[n];
    dsp->history[dsp->head + USB_AUDIO_DSP_TAPS] = pcm[n];
    if (++dsp->head == USB_AUDIO_DSP_TAPS) dsp->head = 0u;
    phase = (uint8_t)(dsp->phase + 5u);
    if (phase < 12u) { dsp->phase = phase; continue; }
    dsp->phase = (uint8_t)(phase - 12u);
    fir = fir_q15(&dsp->history[dsp->head], coefficients[dsp->phase]);
    input = (float)fir * (1.0f / 32768.0f);

    if (umh_usb_audio_dsp_bypass != 0u) {
      am = 0.5f + DSP_MODULATION * input * 0.9f * 0.7f;
      if (am < 0.0f) am = 0.0f;
      if (am > 1.0f) am = 1.0f;
      {
        int level = (int)lrintf(am * 128.0f);
        if (level < 0) level = 0;
        if (level > 128) level = 128;
        envelope[written++] = (uint8_t)level;
      }
      continue;
    }
    /* Same 20 Hz DC blocker and +12 dB programme gain as the host. */
    dsp->dc_state = input - dsp->previous_input + DSP_DC_POLE * dsp->dc_state;
    dsp->previous_input = input;
    value = dsp->dc_state * DSP_PREGAIN;

    /* 5 ms look-ahead limiter: read the oldest slot, overwrite it, measure
     * the new window, then apply the gain to the delayed sample. */
    delayed = dsp_lookahead[dsp->lookahead_pos];
    window_write(dsp_lookahead, lookahead_max, dsp->lookahead_pos, value);
    peak = bits_value(window_peak(lookahead_max, DSP_LOOKAHEAD_BUCKETS));
    if (++dsp->lookahead_pos == USB_AUDIO_DSP_LOOKAHEAD) dsp->lookahead_pos = 0u;
    desired = peak > DSP_LIMIT_CEILING ? DSP_LIMIT_CEILING / peak : 1.0f;
    if (desired < dsp->limiter_gain) {
      dsp->limiter_gain = desired;
    } else {
      dsp->limiter_gain += (desired - dsp->limiter_gain) * DSP_LIMITER_RELEASE;
    }
    limited = delayed * dsp->limiter_gain;
    if (limited > DSP_LIMIT_CEILING) limited = DSP_LIMIT_CEILING;
    if (limited < -DSP_LIMIT_CEILING) limited = -DSP_LIMIT_CEILING;
    magnitude = fabsf(limited);

    window_write(dsp->peak_window, agc_max, dsp->peak_window_pos, magnitude);
    if (dsp->peak_window_count < USB_AUDIO_DSP_PEAK_WINDOW)
      ++dsp->peak_window_count;
    if (++dsp->peak_window_pos == USB_AUDIO_DSP_PEAK_WINDOW)
      dsp->peak_window_pos = 0u;
    peak2 = bits_value(window_peak(agc_max, DSP_AGC_BUCKETS));
    target = dsp->agc_gain;
    if (dsp->peak_window_count >= USB_AUDIO_DSP_PEAK_WINDOW && peak2 > 1e-6f) {
      target = DSP_PROGRAMME_PEAK / peak2;
      if (target < 0.25f) target = 0.25f;
      if (target > 5.0f) target = 5.0f;
    }

    if (magnitude > (3.0f / 32768.0f)) {
      dsp->silent_samples = 0u;
      if (dsp->gate < 80u) ++dsp->gate;
    } else if (dsp->silent_samples < 2400u) {
      ++dsp->silent_samples;
    } else if (dsp->gate != 0u) {
      --dsp->gate;
    }

    am = 0.5f * ((float)dsp->gate / DSP_GATE_FULL) +
         DSP_MODULATION * limited * dsp->agc_gain;
    if (am < 0.0f) am = 0.0f;
    if (am > 1.0f) am = 1.0f;
    /* Plain rounding.  The microphone comparison against the UMH Speaker
     * CDC reference showed the 1..5 Hz sidebands are present in BOTH paths
     * (the CDC's are even stronger at +-1.5 Hz), while the error-feedback
     * version added 10..20 dB of HF hiss and much stronger 2nd/3rd
     * harmonics.  The beat is not the level quantiser, so do not shape it. */
    {
      int level = (int)lrintf(am * 128.0f);
      if (level < 0) level = 0;
      if (level > 128) level = 128;
      envelope[written++] = (uint8_t)level;

    }

    if (target > dsp->agc_gain) {
      float d = target - dsp->agc_gain;
      dsp->agc_gain += d > DSP_AGC_RISE ? DSP_AGC_RISE : d;
    } else if (target < dsp->agc_gain) {
      float d = dsp->agc_gain - target;
      dsp->agc_gain -= d > DSP_AGC_FALL ? DSP_AGC_FALL : d;
    }
  }
  return written;
}
