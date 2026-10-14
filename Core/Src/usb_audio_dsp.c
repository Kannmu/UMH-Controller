#include "usb_audio_dsp.h"
#include <string.h>
#if defined(__arm__) || defined(__thumb__)
#include "cmsis_gcc.h"
#endif

/* 48 kHz -> 20 kHz, rational 5/12 resampler.  A 48-tap Kaiser (beta=6)
 * 7.5 kHz low-pass per fractional phase: <0.20 dB loss at 6 kHz, >65 dB
 * rejection at/above 10 kHz.  Each row sums to 32768; its L1 norm is <=53830,
 * so even full-scale PCM with adversarial signs cannot overflow int32_t.
 * Only output instants run the FIR: 24 M4 SMLADs/output, 480k SMLADs/second.
 * FMAC remains available to calibration; there are no peripheral/DMA locks. */
static const int16_t coefficients[5][USB_AUDIO_DSP_TAPS]
    __attribute__((aligned(4))) = {
  {-6,-1,17,33,15,-46,-100,-63,83,226,183,-105,-433,-433,68,748,922,127,-1259,-2004,-814,2565,6842,9819,9819,6842,2565,-814,-2004,-1259,127,922,748,68,-433,-433,-105,183,226,83,-63,-100,-46,15,33,17,-1,-6},
  {-5,1,20,31,5,-57,-97,-40,113,230,138,-174,-458,-362,204,831,830,-131,-1491,-1933,-291,3398,7592,10079,9441,6038,1765,-1250,-1996,-997,365,974,641,-66,-484,-392,-35,220,215,51,-84,-99,-34,24,34,14,-4,-6},
  {-4,4,22,28,-5,-66,-91,-14,140,224,87,-238,-467,-275,340,886,701,-401,-1682,-1778,310,4249,8274,10215,8953,5198,1013,-1597,-1916,-716,576,988,514,-192,-516,-338,35,248,195,17,-102,-94,-21,32,34,10,-6,-6},
  {-3,6,23,23,-16,-73,-81,14,162,210,30,-296,-458,-173,468,910,536,-674,-1823,-1536,984,5101,8873,10229,8365,4339,320,-1853,-1772,-427,754,965,374,-306,-528,-273,101,265,169,-17,-115,-86,-7,39,33,6,-8,-6},
  {-2,9,23,17,-25,-76,-67,41,179,187,-30,-343,-430,-60,584,899,341,-941,-1903,-1205,1718,5937,9376,10110,7691,3477,-301,-2019,-1573,-140,895,907,226,-405,-519,-200,161,272,137,-49,-123,-75,7,44,30,2,-10,-6}
};

static inline int32_t fir_q15(const int16_t *history, const int16_t *coeff)
{
  int32_t sum = 0;
  uint32_t i;
  for (i = 0u; i < USB_AUDIO_DSP_TAPS; i += 2u) {
#if defined(__arm__) || defined(__thumb__)
    uint32_t x, c;
    /* memcpy compiles to word loads, including a permitted unaligned SRAM
     * load when history starts on an odd halfword.  No aliasing violation. */
    memcpy(&x, history + i, sizeof(x));
    memcpy(&c, coeff + i, sizeof(c));
    sum = __SMLAD(x, c, sum);
#else
    sum += (int32_t)history[i] * coeff[i];
    sum += (int32_t)history[i + 1u] * coeff[i + 1u];
#endif
  }
  sum = (sum + 16384) >> 15;
#if defined(__arm__) || defined(__thumb__)
  return __SSAT(sum, 16);
#else
  return sum > 32767 ? 32767 : (sum < -32768 ? -32768 : sum);
#endif
}

void usb_audio_dsp_reset(usb_audio_dsp_t *dsp)
{
  memset(dsp, 0, sizeof(*dsp));
  dsp->normalization_gain_q16 = 65536u;
}

uint16_t usb_audio_dsp_process(usb_audio_dsp_t *dsp, const int16_t *pcm,
                               uint16_t input_count, uint16_t gain_q15,
                               uint8_t *envelope)
{
  enum {
    MODULATION_Q15 = 15401u,       /* 47%, UMH Speaker default depth 8/16 */
    PROGRAMME_PEAK_Q15 = 31372u,   /* 0.9 * 0.5 / 0.47 headroom target */
    MIN_GAIN_Q16 = 16384u,         /* 0.25 */
    MAX_GAIN_Q16 = 327680u,        /* 5.0 */
    GAIN_RISE_STEP_Q16 = 33u,      /* 0.05 per 100 samples at 20 kHz */
    GAIN_FALL_STEP_Q16 = 66u       /* 0.10 per 100 samples at 20 kHz */
  };
  uint16_t n, written = 0u;
  for (n = 0u; n < input_count; ++n) {
    int32_t sample, magnitude, peak = 0, normalized, program_q15, delta, level;
    uint32_t target_gain_q16;
    uint8_t phase;
    uint16_t i;
    int64_t product;

    dsp->history[dsp->head] = pcm[n];
    dsp->history[dsp->head + USB_AUDIO_DSP_TAPS] = pcm[n];
    if (++dsp->head == USB_AUDIO_DSP_TAPS) dsp->head = 0u;
    phase = (uint8_t)(dsp->phase + 5u);
    if (phase < 12u) { dsp->phase = phase; continue; }
    dsp->phase = (uint8_t)(phase - 12u);
    sample = fir_q15(&dsp->history[dsp->head], coefficients[dsp->phase]);

    /* Match UMH Speaker's 20 Hz DC blocker at the 20 kHz envelope rate.
     * Q8 state keeps the fixed-point error below one PCM LSB. */
    int32_t dc_error = sample * 256 - dsp->dc_q8;
    dsp->dc_q8 += (int32_t)(((int64_t)dc_error * 206) >> 15);
    sample -= dsp->dc_q8 >> 8;
    magnitude = sample < 0 ? -sample : sample;

    dsp->peak_window[dsp->peak_window_pos] = (uint16_t)magnitude;
    if (dsp->peak_window_count < USB_AUDIO_DSP_PEAK_WINDOW)
      ++dsp->peak_window_count;
    if (++dsp->peak_window_pos == USB_AUDIO_DSP_PEAK_WINDOW)
      dsp->peak_window_pos = 0u;
    for (i = 0u; i < dsp->peak_window_count; ++i)
      if (dsp->peak_window[i] > peak) peak = dsp->peak_window[i];

    target_gain_q16 = dsp->normalization_gain_q16;
    if (peak != 0) {
      target_gain_q16 = (PROGRAMME_PEAK_Q15 * 65536u) / (uint32_t)peak;
      if (target_gain_q16 < MIN_GAIN_Q16) target_gain_q16 = MIN_GAIN_Q16;
      if (target_gain_q16 > MAX_GAIN_Q16) target_gain_q16 = MAX_GAIN_Q16;
    }

    /* Silence gates only the carrier bias; the programme is never multiplied
     * by its own level follower.  This matches UMH Speaker's AM path. */
    if (magnitude > 3) {
      dsp->silent_samples = 0u;
      if (dsp->gate < 80u) ++dsp->gate;
    } else if (dsp->silent_samples < 2400u) {
      ++dsp->silent_samples;
    } else if (dsp->gate != 0u) {
      --dsp->gate;
    }

    product = (int64_t)sample * dsp->normalization_gain_q16;
    normalized = product >= 0 ? (int32_t)((product + 32768) >> 16) :
                 -(int32_t)((-product + 32768) >> 16);
    product = (int64_t)normalized * MODULATION_Q15;
    program_q15 = product >= 0 ? (int32_t)((product + 16384) >> 15) :
                  -(int32_t)((-product + 16384) >> 15);
    delta = program_q15 >= 0 ? (program_q15 + 128) / 256 :
            -((-program_q15 + 128) / 256);
    level = ((int32_t)dsp->gate * 64 + 40) / 80 + delta;
    if (level < 0) level = 0;
    if (level > 128) level = 128;
    envelope[written++] = (uint8_t)((level * gain_q15 + 16384) >> 15);

    if (target_gain_q16 > dsp->normalization_gain_q16) {
      uint32_t difference = target_gain_q16 - dsp->normalization_gain_q16;
      dsp->normalization_gain_q16 += difference > GAIN_RISE_STEP_Q16 ?
                                     GAIN_RISE_STEP_Q16 : difference;
    } else if (target_gain_q16 < dsp->normalization_gain_q16) {
      uint32_t difference = dsp->normalization_gain_q16 - target_gain_q16;
      dsp->normalization_gain_q16 -= difference > GAIN_FALL_STEP_Q16 ?
                                     GAIN_FALL_STEP_Q16 : difference;
    }
  }
  return written;
}
