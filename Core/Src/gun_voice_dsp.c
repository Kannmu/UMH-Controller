#include "gun_voice_dsp.h"
#include "arm_math.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define VOICE_HOP GUN_VOICE_HOP
#define VOICE_FFT (2u * GUN_VOICE_HOP)
#define VOICE_BINS (VOICE_FFT / 2u)
#define VOICE_MIN_RMS 8u
#define VOICE_MIN_SPEECH_FRAMES 6u
#define VOICE_RMS_TARGET 32.0f
#define VOICE_GAIN_MAX 4.0f
#define VOICE_GAIN_FLOOR 0.06f

/* Tables are generated once, not calculated on the real-time path. */
static const int16_t voice_decode[256] = {
  -32124, -31100, -30076, -29052, -28028, -27004, -25980, -24956, -23932, -22908, -21884, -20860, -19836, -18812, -17788, -16764,
  -15996, -15484, -14972, -14460, -13948, -13436, -12924, -12412, -11900, -11388, -10876, -10364, -9852, -9340, -8828, -8316,
  -7932, -7676, -7420, -7164, -6908, -6652, -6396, -6140, -5884, -5628, -5372, -5116, -4860, -4604, -4348, -4092,
  -3900, -3772, -3644, -3516, -3388, -3260, -3132, -3004, -2876, -2748, -2620, -2492, -2364, -2236, -2108, -1980,
  -1884, -1820, -1756, -1692, -1628, -1564, -1500, -1436, -1372, -1308, -1244, -1180, -1116, -1052, -988, -924,
  -876, -844, -812, -780, -748, -716, -684, -652, -620, -588, -556, -524, -492, -460, -428, -396,
  -372, -356, -340, -324, -308, -292, -276, -260, -244, -228, -212, -196, -180, -164, -148, -132,
  -120, -112, -104, -96, -88, -80, -72, -64, -56, -48, -40, -32, -24, -16, -8, 0,
  32124, 31100, 30076, 29052, 28028, 27004, 25980, 24956, 23932, 22908, 21884, 20860, 19836, 18812, 17788, 16764,
  15996, 15484, 14972, 14460, 13948, 13436, 12924, 12412, 11900, 11388, 10876, 10364, 9852, 9340, 8828, 8316,
  7932, 7676, 7420, 7164, 6908, 6652, 6396, 6140, 5884, 5628, 5372, 5116, 4860, 4604, 4348, 4092,
  3900, 3772, 3644, 3516, 3388, 3260, 3132, 3004, 2876, 2748, 2620, 2492, 2364, 2236, 2108, 1980,
  1884, 1820, 1756, 1692, 1628, 1564, 1500, 1436, 1372, 1308, 1244, 1180, 1116, 1052, 988, 924,
  876, 844, 812, 780, 748, 716, 684, 652, 620, 588, 556, 524, 492, 460, 428, 396,
  372, 356, 340, 324, 308, 292, 276, 260, 244, 228, 212, 196, 180, 164, 148, 132,
  120, 112, 104, 96, 88, 80, 72, 64, 56, 48, 40, 32, 24, 16, 8, 0,
};
static const float voice_window[128] = {
  0.012271538f, 0.036807223f, 0.061320736f, 0.085797312f, 0.110222207f, 0.134580709f, 0.158858143f, 0.183039888f,
  0.207111376f, 0.231058108f, 0.254865660f, 0.278519689f, 0.302005949f, 0.325310292f, 0.348418680f, 0.371317194f,
  0.393992040f, 0.416429560f, 0.438616239f, 0.460538711f, 0.482183772f, 0.503538384f, 0.524589683f, 0.545324988f,
  0.565731811f, 0.585797857f, 0.605511041f, 0.624859488f, 0.643831543f, 0.662415778f, 0.680600998f, 0.698376249f,
  0.715730825f, 0.732654272f, 0.749136395f, 0.765167266f, 0.780737229f, 0.795836905f, 0.810457198f, 0.824589303f,
  0.838224706f, 0.851355193f, 0.863972856f, 0.876070094f, 0.887639620f, 0.898674466f, 0.909167983f, 0.919113852f,
  0.928506080f, 0.937339012f, 0.945607325f, 0.953306040f, 0.960430519f, 0.966976471f, 0.972939952f, 0.978317371f,
  0.983105487f, 0.987301418f, 0.990902635f, 0.993906970f, 0.996312612f, 0.998118113f, 0.999322385f, 0.999924702f,
  0.999924702f, 0.999322385f, 0.998118113f, 0.996312612f, 0.993906970f, 0.990902635f, 0.987301418f, 0.983105487f,
  0.978317371f, 0.972939952f, 0.966976471f, 0.960430519f, 0.953306040f, 0.945607325f, 0.937339012f, 0.928506080f,
  0.919113852f, 0.909167983f, 0.898674466f, 0.887639620f, 0.876070094f, 0.863972856f, 0.851355193f, 0.838224706f,
  0.824589303f, 0.810457198f, 0.795836905f, 0.780737229f, 0.765167266f, 0.749136395f, 0.732654272f, 0.715730825f,
  0.698376249f, 0.680600998f, 0.662415778f, 0.643831543f, 0.624859488f, 0.605511041f, 0.585797857f, 0.565731811f,
  0.545324988f, 0.524589683f, 0.503538384f, 0.482183772f, 0.460538711f, 0.438616239f, 0.416429560f, 0.393992040f,
  0.371317194f, 0.348418680f, 0.325310292f, 0.302005949f, 0.278519689f, 0.254865660f, 0.231058108f, 0.207111376f,
  0.183039888f, 0.158858143f, 0.134580709f, 0.110222207f, 0.085797312f, 0.061320736f, 0.036807223f, 0.012271538f,
};

/* Only the 128-point CMSIS real FFT tables are linked. */
extern arm_status arm_rfft_128_fast_init_f32(arm_rfft_fast_instance_f32 *instance);

enum { VOICE_HISTOGRAM, VOICE_NOISE, VOICE_DENOISE,
       VOICE_DONE, VOICE_REJECTED };

/* may_alias permits reusing the inactive aperture's byte ring as this aligned
 * scratch object.  Both consumers are serialized by gun ownership and run in
 * the render task; configure() overwrites it after we have finished. */
typedef struct __attribute__((may_alias)) {
  float time[VOICE_FFT];
  float spectrum[VOICE_FFT];
  float noise[VOICE_BINS];
  float gain[VOICE_BINS];
  float overlap[VOICE_HOP];
  uint16_t histogram[64];
  arm_rfft_fast_instance_f32 fft;
} voice_workspace_t;
_Static_assert(sizeof(voice_workspace_t) <= GUN_VOICE_WORK_BYTES,
               "voice FFT must fit the inactive aperture ring");

uint8_t gun_voice_encode(int32_t pcm)
{
  uint32_t magnitude = pcm < 0 ? 0u - (uint32_t)pcm : (uint32_t)pcm;
  uint32_t sign = pcm < 0 ? 0x80u : 0u;
  uint32_t exponent;
  if (magnitude > 32124u) magnitude = 32124u;
  magnitude += 132u;
  exponent = 24u - (uint32_t)__builtin_clz(magnitude);
  return (uint8_t)~(sign | (exponent << 4) |
                    ((magnitude >> (exponent + 3u)) & 15u));
}

int16_t gun_voice_decode(uint8_t code)
{
  return voice_decode[code];
}

#if GUN_VOICE_CODEC != GUN_VOICE_CODEC_MULAW8
/* IMA ADPCM generalised from the three magnitude bits of the canonical form to
 * GUN_VOICE_ADPCM_MAG_BITS.  The encoder reconstructs with exactly the
 * decoder's increment, so the two state machines never drift apart. */
#define GUN_VOICE_ADPCM_MAG_BITS (GUN_VOICE_BITS_PER_SAMPLE - 1u)
#define GUN_VOICE_ADPCM_STEP_MAX 88u

static const int16_t adpcm_step_table[GUN_VOICE_ADPCM_STEP_MAX + 1u] = {
      7,     8,     9,    10,    11,    12,    13,    14,    16,    17,
     19,    21,    23,    25,    28,    31,    34,    37,    41,    45,
     50,    55,    60,    66,    73,    80,    88,    97,   107,   118,
    130,   143,   157,   173,   190,   209,   230,   253,   279,   307,
    337,   371,   408,   449,   494,   544,   598,   658,   724,   796,
    876,   963,  1060,  1166,  1282,  1411,  1552,  1707,  1878,  2066,
   2272,  2499,  2749,  3024,  3327,  3660,  4026,  4428,  4871,  5358,
   5894,  6484,  7132,  7845,  8630,  9493, 10442, 11487, 12635, 13899,
  15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

#if GUN_VOICE_ADPCM_MAG_BITS == 3u
static const int8_t adpcm_index_delta[8] = { -1, -1, -1, -1, 2, 4, 6, 8 };
#else
static const int8_t adpcm_index_delta[16] = {
  -1, -1, -1, -1, -1, -1, -1, -1, 1, 2, 3, 4, 5, 6, 7, 8
};
#endif

static void adpcm_apply(gun_voice_codec_t *codec, int32_t step, uint32_t code)
{
  int32_t increment = step >> GUN_VOICE_ADPCM_MAG_BITS;
  int32_t predict;
  uint8_t bit;
  for (bit = 0u; bit < GUN_VOICE_ADPCM_MAG_BITS; ++bit) {
    if ((code & (1u << bit)) != 0u)
      increment += step >> (GUN_VOICE_ADPCM_MAG_BITS - 1u - bit);
  }
  if ((code & (1u << GUN_VOICE_ADPCM_MAG_BITS)) != 0u) increment = -increment;
  predict = (int32_t)codec->predict + increment;
  if (predict > 32767) predict = 32767;
  else if (predict < -32768) predict = -32768;
  codec->predict = (int16_t)predict;
  codec->index = (int8_t)((int32_t)codec->index +
      (int32_t)adpcm_index_delta[code & ((1u << GUN_VOICE_ADPCM_MAG_BITS) - 1u)]);
  if (codec->index < 0) codec->index = 0;
  else if ((uint8_t)codec->index > GUN_VOICE_ADPCM_STEP_MAX)
    codec->index = (int8_t)GUN_VOICE_ADPCM_STEP_MAX;
}

static uint32_t adpcm_encode(gun_voice_codec_t *codec, int32_t sample)
{
  int32_t step = adpcm_step_table[(uint8_t)codec->index];
  int32_t diff = sample - (int32_t)codec->predict;
  int32_t threshold = step;
  uint32_t code = 0u;
  uint8_t bit;
  if (diff < 0) {
    code = 1u << GUN_VOICE_ADPCM_MAG_BITS;
    diff = -diff;
  }
  for (bit = GUN_VOICE_ADPCM_MAG_BITS; bit-- > 0u; ) {
    if (diff >= threshold) {
      code |= 1u << bit;
      diff -= threshold;
    }
    threshold >>= 1;
  }
  adpcm_apply(codec, step, code);
  return code;
}
#endif

void gun_voice_codec_reset(gun_voice_codec_t *codec)
{
  if (codec == NULL) return;
  memset(codec, 0, sizeof(*codec));
}

void gun_voice_codec_push(gun_voice_codec_t *codec, uint8_t *buffer, int32_t pcm)
{
  uint32_t code;
  if (codec == NULL || buffer == NULL) return;
  if (pcm > 32767) pcm = 32767;
  else if (pcm < -32768) pcm = -32768;
#if GUN_VOICE_CODEC == GUN_VOICE_CODEC_MULAW8
  code = gun_voice_encode(pcm);
#else
  code = adpcm_encode(codec, pcm);
#endif
  codec->acc |= code << codec->bits;
  codec->bits = (uint8_t)(codec->bits + GUN_VOICE_BITS_PER_SAMPLE);
  if (codec->bits >= 8u) {
    buffer[codec->byte_pos++] = (uint8_t)codec->acc;
    codec->acc >>= 8;
    codec->bits = (uint8_t)(codec->bits - 8u);
  }
}

void gun_voice_codec_flush(gun_voice_codec_t *codec, uint8_t *buffer)
{
  if (codec == NULL || buffer == NULL || codec->bits == 0u) return;
  buffer[codec->byte_pos++] = (uint8_t)codec->acc;
  codec->acc = 0u;
  codec->bits = 0u;
}

int16_t gun_voice_codec_pull(gun_voice_codec_t *codec, const uint8_t *buffer,
                             uint16_t byte_limit)
{
  uint32_t code;
  if (codec == NULL || buffer == NULL) return 0;
  while (codec->bits < GUN_VOICE_BITS_PER_SAMPLE) {
    if (codec->byte_pos >= byte_limit) return 0;
    codec->acc |= (uint32_t)buffer[codec->byte_pos++] << codec->bits;
    codec->bits = (uint8_t)(codec->bits + 8u);
  }
  code = codec->acc & ((1u << GUN_VOICE_BITS_PER_SAMPLE) - 1u);
  codec->acc >>= GUN_VOICE_BITS_PER_SAMPLE;
  codec->bits = (uint8_t)(codec->bits - GUN_VOICE_BITS_PER_SAMPLE);
#if GUN_VOICE_CODEC == GUN_VOICE_CODEC_MULAW8
  return gun_voice_decode((uint8_t)code);
#else
  adpcm_apply(codec, adpcm_step_table[(uint8_t)codec->index], code);
  return codec->predict;
#endif
}

static uint8_t rms_bin(uint32_t rms)
{
  uint32_t exponent;
  uint32_t fraction;
  if (rms == 0u) return 0u;
  exponent = 31u - (uint32_t)__builtin_clz(rms);
  fraction = ((rms << 2) >> exponent) - 4u;
  return (uint8_t)(exponent * 4u + fraction);
}

static uint32_t bin_rms(uint32_t bin)
{
  return ((4u + (bin & 3u)) << (bin >> 2)) >> 2;
}

static void stream_rewind(gun_voice_processor_t *p)
{
  gun_voice_codec_reset(&p->dec);
}

/* Next stored sample.  Past the end it returns silence without consuming
 * bits, which reproduces the original zero-filled random-access reads. */
static int16_t stream_next(gun_voice_processor_t *p)
{
  return gun_voice_codec_pull(&p->dec, p->samples, p->byte_limit);
}

static void hop_read(gun_voice_processor_t *p)
{
  uint32_t i;
  for (i = 0u; i < GUN_VOICE_HOP; ++i) p->fresh[i] = stream_next(p);
}

void gun_voice_process_begin(gun_voice_processor_t *p, uint8_t *samples,
                             uint16_t count, void *workspace)
{
  voice_workspace_t *w = workspace;
  memset(p, 0, sizeof(*p));
  memset(w, 0, sizeof(*w));
  p->samples = samples;
  p->workspace = workspace;
  p->count = count;
  p->byte_limit = (uint16_t)(((uint32_t)count * GUN_VOICE_BITS_PER_SAMPLE + 7u) / 8u);
  p->codec_version = (uint8_t)GUN_VOICE_CODEC_VERSION;
  p->fallback_power = 1.0e30f;
  gun_voice_codec_reset(&p->dec);
  gun_voice_codec_reset(&p->enc);
  if (count < GUN_VOICE_HOP * VOICE_MIN_SPEECH_FRAMES ||
      arm_rfft_128_fast_init_f32(&w->fft) != ARM_MATH_SUCCESS)
    p->stage = VOICE_REJECTED;
}

static void histogram_step(gun_voice_processor_t *p, voice_workspace_t *w)
{
  uint32_t i;
  uint32_t end = (uint32_t)p->position + GUN_VOICE_HOP;
  uint32_t square_sum = 0u;
  uint32_t rms;
  if (end > p->count) end = p->count;
  for (i = (uint32_t)p->position; i < end; ++i) {
    /* Divide by 8 before squaring: all 64 samples then fit in uint32_t. */
    int32_t x = (int32_t)stream_next(p) / 8;
    square_sum += (uint32_t)(x * x);
  }
  rms = (uint32_t)(sqrtf((float)square_sum / (float)(end - p->position)) * 8.0f);
  ++w->histogram[rms_bin(rms)];
  p->position = (int32_t)end;
  if (end == p->count) {
    uint32_t frames = ((uint32_t)p->count + GUN_VOICE_HOP - 1u) / GUN_VOICE_HOP;
    uint32_t percentile = (frames + 4u) / 5u;
    uint32_t accumulated = 0u;
    for (i = 0u; i < 64u; ++i) {
      accumulated += w->histogram[i];
      if (accumulated >= percentile) break;
    }
    p->noise_rms = bin_rms(i);
    if (p->noise_rms < VOICE_MIN_RMS) p->noise_rms = VOICE_MIN_RMS;
    /* Upper edge of the quantile bucket, with margin for noise variance. */
    p->noise_ceiling = (bin_rms(i + 1u) * 3u) / 2u;
    if (p->noise_ceiling < VOICE_MIN_RMS) p->noise_ceiling = VOICE_MIN_RMS;
    p->position = -(int32_t)GUN_VOICE_HOP;
    p->stage = VOICE_NOISE;
    stream_rewind(p);
  }
}

static float transform(gun_voice_processor_t *p, voice_workspace_t *w)
{
  uint32_t i;
  float square_sum = 0.0f;
  hop_read(p);
  for (i = 0u; i < GUN_VOICE_HOP; ++i) {
    float x = (float)p->window[i];
    square_sum += x * x;
    w->time[i] = x * voice_window[i];
  }
  for (i = 0u; i < GUN_VOICE_HOP; ++i) {
    float x = (float)p->fresh[i];
    square_sum += x * x;
    w->time[GUN_VOICE_HOP + i] = x * voice_window[GUN_VOICE_HOP + i];
  }
  memcpy(p->window, p->fresh, sizeof(p->window));
  arm_rfft_fast_f32(&w->fft, w->time, w->spectrum, 0u);
  return square_sum * (1.0f / (float)VOICE_FFT);
}

static float bin_power(const voice_workspace_t *w, uint32_t bin)
{
  float re = w->spectrum[2u * bin];
  float im = w->spectrum[2u * bin + 1u];
  return re * re + im * im;
}

static void noise_step(gun_voice_processor_t *p, voice_workspace_t *w)
{
  float mean_square = transform(p, w);
  float power = 0.0f;
  float root_sum = 0.0f;
  float upper_power = 0.0f;
  float upper_root_sum = 0.0f;
  float trimmed = 0.0f;
  uint32_t trimmed_count = 0u;
  uint32_t k;
  for (k = 2u; k <= 55u; ++k) {
    float q = bin_power(w, k);
    float root = sqrtf(q);
    power += q;
    root_sum += root;
    if (k >= 6u) { upper_power += q; upper_root_sum += root; }
  }
  /* Tonal/voiced frames must not teach the estimator to remove the voice, even
   * when speech starts with the trigger and there is no leading silence.
   * Colored fan/room noise can have low full-band flatness: the second test
   * checks above 375 Hz too, without relaxing the tonal-voice exclusion. */
  if (p->position >= 0 && p->position + (int32_t)VOICE_FFT <= p->count &&
      mean_square <= (float)p->noise_ceiling * (float)p->noise_ceiling &&
      (root_sum * root_sum > 0.64f * 54.0f * power ||
       (root_sum * root_sum > 0.36f * 54.0f * power &&
        upper_root_sum * upper_root_sum > 0.50f * 50.0f * upper_power))) {
    for (k = 1u; k < VOICE_BINS; ++k) w->noise[k] += bin_power(w, k);
    ++p->noise_frames;
  }
  /* If every frame is voiced, use the weak bins as a conservative white-noise
   * fallback instead of declaring the whole first syllable to be noise. */
  for (k = 2u; k <= 55u; ++k) {
    float q = bin_power(w, k);
    if (q <= power * 0.025f) { trimmed += q; ++trimmed_count; }
  }
  if (p->position >= 0 && p->position + (int32_t)VOICE_FFT <= p->count &&
      trimmed_count >= 16u) {
    float floor_power = trimmed / (float)trimmed_count;
    if (floor_power < p->fallback_power) p->fallback_power = floor_power;
  }
  p->position += (int32_t)GUN_VOICE_HOP;
  if (p->position >= p->count) {
    float scale = p->noise_frames != 0u ? 1.0f / (float)p->noise_frames : 0.0f;
    /* A sine window has sum(w*w)=64.  This floor prevents silence quantization
     * and the low-power tails of tones from requesting unbounded gain. */
    float minimum = (float)(VOICE_MIN_RMS * VOICE_MIN_RMS * GUN_VOICE_HOP);
    if (p->fallback_power > 1.0e29f) p->fallback_power = minimum;
    if (p->fallback_power < minimum) p->fallback_power = minimum;
    p->noise_power = 0.0f;
    for (k = 1u; k < VOICE_BINS; ++k) {
      float noise = p->noise_frames != 0u ? w->noise[k] * scale : p->fallback_power;
      if (noise < minimum) noise = minimum;
      w->noise[k] = noise;
      w->gain[k] = VOICE_GAIN_FLOOR;
      if (k >= 2u && k <= 55u) p->noise_power += noise;
    }
    p->noise_rms = (uint32_t)sqrtf(p->noise_power / (54.0f * (float)GUN_VOICE_HOP));
    p->position = -(int32_t)GUN_VOICE_HOP;
    p->stage = VOICE_DENOISE;
    stream_rewind(p);
  }
}

static void denoise_step(gun_voice_processor_t *p, voice_workspace_t *w)
{
  float power = 0.0f;
  float whitened_power = 0.0f;
  float whitened_roots = 0.0f;
  float target;
  uint32_t k;
  uint8_t noise_like;
  (void)transform(p, w);
  for (k = 2u; k <= 55u; ++k) {
    float q = bin_power(w, k);
    float ratio = q / w->noise[k];
    power += q;
    whitened_power += ratio;
    whitened_roots += sqrtf(ratio);
  }
  noise_like = whitened_roots * whitened_roots >
               0.74f * 54.0f * whitened_power ? 1u : 0u;
  /* SNR alone would treat a sudden louder fan/hiss as speech.  The whitened
   * spectrum also tests structure, independent of background level/color. */
  target = (power / p->noise_power - 1.5f) * (1.0f / 1.5f);
  if (target < 0.0f) target = 0.0f;
  if (target > 1.0f) target = 1.0f;
  if (noise_like != 0u) {
    /* Retain short unvoiced tails beside a vowel, at reduced strength.  Only
     * structured voice teaches the global loudness estimate and refreshes this
     * hold; broadband noise cannot keep it open indefinitely. */
    if (p->voice_hold == 0u) {
      target = 0.0f;
      float ratio = power / p->noise_power;
      /* Ordinary frame variance must not pump the estimated floor.  Follow
       * clear level changes; the whole-record spectral shape remains fixed. */
      if (ratio > 4.0f || ratio < 0.25f) {
        float scale = 0.5f + 0.5f * ratio;
        p->noise_power = 0.0f;
        for (k = 1u; k < VOICE_BINS; ++k) {
          float noise = w->noise[k] * scale;
          float minimum = (float)(VOICE_MIN_RMS * VOICE_MIN_RMS * GUN_VOICE_HOP);
          if (noise < minimum) noise = minimum;
          w->noise[k] = noise;
          if (k >= 2u && k <= 55u) p->noise_power += noise;
        }
      }
    } else if (target > 0.35f) target = 0.35f;
  }
  if (p->voice_hold != 0u) --p->voice_hold;
  if (target >= 0.75f) {
    if (++p->speech_run > p->speech_run_max) p->speech_run_max = p->speech_run;
    if (p->speech_run >= 3u) p->voice_hold = 10u; /* 80 ms */
    else if (p->voice_hold == 0u) target = 0.0f;
  } else p->speech_run = 0u;
  if (target >= 0.75f) ++p->speech_frames;
  p->gate += (target - p->gate) * (target > p->gate ? 0.75f : 0.25f);
  if (p->gate < 0.002f) p->gate = 0.0f;
  if (p->gate == 0.0f) {
    memset(w->time, 0, sizeof(w->time));
    for (k = 1u; k < VOICE_BINS; ++k) w->gain[k] = VOICE_GAIN_FLOOR;
  } else {
    w->spectrum[0] = 0.0f; /* no DC or Nyquist */
    w->spectrum[1] = 0.0f;
    for (k = 1u; k < VOICE_BINS; ++k) {
      float q = bin_power(w, k);
      float gain = 1.0f - 1.5f * w->noise[k] / (q + 1.0f);
      float band = k < 2u || k > 56u ? 0.0f : k > 51u ? (57.0f - (float)k) / 6.0f : 1.0f;
      float emitted;
      if (gain < VOICE_GAIN_FLOOR) gain = VOICE_GAIN_FLOOR;
      /* Smooth spectral gains, not loudness.  The common gain is calculated once
       * from all speech after this pass.  This avoids musical-noise transients. */
      w->gain[k] += (gain - w->gain[k]) * (gain > w->gain[k] ? 0.75f : 0.20f);
      emitted = w->gain[k] * p->gate * band;
      w->spectrum[2u * k] *= emitted;
      w->spectrum[2u * k + 1u] *= emitted;
    }
    arm_rfft_fast_f32(&w->fft, w->spectrum, w->time, 1u);
  }
  for (k = 0u; k < VOICE_HOP; ++k) {
    int32_t index = p->position + (int32_t)k;
    float x = w->overlap[k] + w->time[k] * voice_window[k];
    w->overlap[k] = w->time[k + VOICE_HOP] * voice_window[k + VOICE_HOP];
    if (index >= 0 && index < (int32_t)p->count) {
      int32_t pcm = (int32_t)(x + (x >= 0.0f ? 0.5f : -0.5f));
      /* The rewrite trails the read cursor by one hop, so the encoder never
       * touches a sample this pass has not decoded yet. */
      gun_voice_codec_push(&p->enc, p->samples, pcm);
      if (target >= 0.75f) {
        p->speech_square_sum += x * x;
        ++p->active_samples;
      }
    }
  }
  p->position += (int32_t)GUN_VOICE_HOP;
  if (p->position >= (int32_t)p->count) {
    gun_voice_codec_flush(&p->enc, p->samples);
    if (p->speech_frames < VOICE_MIN_SPEECH_FRAMES || p->speech_run_max < 3u ||
        p->active_samples == 0u) {
      p->stage = VOICE_REJECTED;
      return;
    }
    p->speech_rms = sqrtf(p->speech_square_sum / (float)p->active_samples);
    if (p->speech_rms < (float)VOICE_MIN_RMS) { p->stage = VOICE_REJECTED; return; }
    p->playback_gain = VOICE_RMS_TARGET / p->speech_rms;
    if (p->playback_gain > VOICE_GAIN_MAX) p->playback_gain = VOICE_GAIN_MAX;
    p->position = 0;
    p->stage = VOICE_DONE;
  }
}

int gun_voice_process_service(gun_voice_processor_t *p)
{
  voice_workspace_t *w = p->workspace;
  uint32_t step;
  for (step = 0u; step < GUN_VOICE_PROCESS_STEPS; ++step) {
    switch (p->stage) {
      case VOICE_HISTOGRAM: histogram_step(p, w); break;
      case VOICE_NOISE: noise_step(p, w); break;
      case VOICE_DENOISE: denoise_step(p, w); break;
      case VOICE_DONE: return 1;
      default: return -1;
    }
  }
  return p->stage == VOICE_DONE ? 1 : p->stage == VOICE_REJECTED ? -1 : 0;
}

void gun_voice_player_open(gun_voice_player_t *player, const uint8_t *data,
                           uint16_t samples, float gain)
{
  if (player == NULL) return;
  memset(player, 0, sizeof(*player));
  player->data = data;
  player->total = samples;
  player->byte_limit = (uint16_t)(((uint32_t)samples * GUN_VOICE_BITS_PER_SAMPLE + 7u) / 8u);
  player->gain = gain;
  gun_voice_codec_reset(&player->codec);
}

uint16_t gun_voice_player_read(gun_voice_player_t *player, uint8_t *out,
                               uint16_t capacity)
{
  uint16_t count = 0u;
  if (player == NULL || out == NULL) return 0u;
  while (count < capacity && player->position < player->total) {
    float x = (float)gun_voice_codec_pull(&player->codec, player->data,
                                          player->byte_limit) * player->gain;
    float magnitude = fabsf(x);
    /* A soft knee preserves headroom without severe hard-clipping harmonics. */
    if (magnitude > 48.0f) {
      magnitude = 48.0f + 16.0f * (magnitude - 48.0f) / (magnitude - 32.0f);
      x = x < 0.0f ? -magnitude : magnitude;
    }
    out[count++] = (uint8_t)(int32_t)(64.0f + x + 0.5f);
    ++player->position;
  }
  return count;
}
