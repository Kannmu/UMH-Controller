/* UMH v7 four-microphone echo self-calibration.
 *
 * Measurement: 21 Hadamard patterns locate the wall-echo step, then six
 * 400 us gates after the step are recorded over all 84 patterns.  A
 * Walsh-Hadamard transform decodes one complex channel response per
 * (channel, microphone).  A wall-image model with unknown distance and two
 * tilt angles is fitted by exploiting the phase coherence of microphone
 * difference phasors across channels; the per-channel phase is then
 * estimated by a rank-1 (a_i * rho_j) fit.  A final one-pattern array-gain
 * check chooses the correction sign before the result is committed.
 */
#include "us_calibration.h"
#include "mic_capture.h"
#include "main.h"
#include "cmsis_os.h"
#include "system_status.h"
#include <math.h>
#include <string.h>

#define CAL_CHANNELS  UMH_DEVICE_CHANNEL_COUNT
#define CAL_MICS      UMH_DEVICE_MIC_COUNT
#define CAL_PI        3.14159265358979323846f
#define CAL_TWO_PI    6.28318530717958647692f

#define CAL_SURVEY_PATTERNS 21u
#define CAL_SURVEY_GATES    64u
#define CAL_SURVEY_WIDTH    20u      /* 200 us,  8 carrier cycles  */
#define CAL_SURVEY_STEP     30u      /* 300 us, 12 carrier cycles  */
#define CAL_FINAL_GATES     4u
#define CAL_FINAL_WIDTH     40u      /* 400 us, 16 carrier cycles  */
#define CAL_BURST_US         1000u    /* transmit on-time per pattern */
#define CAL_DIRECT_SKIP_GATES 8u      /* ignore direct-coupling gates */
#ifndef CAL_QUALITY_RELAXED
/* Bring-up switch: 1 completes the flow and writes EEPROM even when the
 * environment is not a clean plane; set to 0 for final accuracy gates. */
#define CAL_QUALITY_RELAXED 1
#endif

#define CAL_MIN_ECHO_RATIO  3.0f
#define CAL_MAX_MIC_RATIO   2.0f
#define CAL_MAX_RESIDUAL    5.0e-3f
#define CAL_MAX_MIC_CONSIST 12.0f
#define CAL_MAX_PRERMS      90.0f
#define CAL_MAX_TILT        55.0f
#define CAL_MIN_VERIFY_DB   3.0f
#define CAL_GRID_TILT_DEG   4.375f    /* 17 points over +/-35 deg */

/* Microphone coordinates from the 2026-09-02 pick-and-place file, ordered
 * by the FPGA demodulator slots:
 *   slot 0 DATA0 rising (U181, SELECT high), slot 1 DATA0 falling (U180),
 *   slot 2 DATA1 rising (U204, SELECT high), slot 3 DATA1 falling (U182). */
static const float cal_mic_x_mm[CAL_MICS] = {-42.496f, 42.501f, 0.000f, 0.004f};
static const float cal_mic_y_mm[CAL_MICS] = { 24.420f, 24.420f, 0.814f, -49.179f};

static const uint8_t cal_pair_j[6] = {0u, 0u, 0u, 1u, 1u, 2u};
static const uint8_t cal_pair_k[6] = {1u, 2u, 3u, 2u, 3u, 3u};

static float cal_chan_x_mm[CAL_CHANNELS];
static float cal_chan_y_mm[CAL_CHANNELS];
static float cal_survey_power[CAL_SURVEY_GATES];
static float cal_survey_noise[CAL_SURVEY_GATES];
static float cal_final_noise[CAL_FINAL_GATES];
static int16_t cal_raw[CAL_FINAL_GATES][CAL_CHANNELS][CAL_MICS][2];
static float cal_zre[CAL_CHANNELS][CAL_MICS];
static float cal_zim[CAL_CHANNELS][CAL_MICS];
static float cal_path[CAL_CHANNELS][CAL_MICS];
static float cal_phi[CAL_CHANNELS];
static float cal_fit_deg[CAL_CHANNELS];
static float cal_a_re[CAL_CHANNELS];
static float cal_a_im[CAL_CHANNELS];
static float cal_rho_re[CAL_MICS];
static float cal_rho_im[CAL_MICS];
static uint8_t cal_corr_a[CAL_CHANNELS];
static uint8_t cal_corr_b[CAL_CHANNELS];

static uint32_t cal_frame_sequence;
static uint16_t cal_block_expected;
static float cal_k_wave_mm;     /* 2*pi*f/c in rad/mm */
static float cal_baseline_power;
static uint8_t cal_arrival_gate;

static float cal_wrap_pi(float x)
{
  while (x > CAL_PI) x -= CAL_TWO_PI;
  while (x < -CAL_PI) x += CAL_TWO_PI;
  return x;
}

static float cal_deg(float rad) { return rad * (180.0f / CAL_PI); }

/* The 84 channels need an 84 x 84 binary orthogonal code.  A Sylvester
 * 128 x 128 transform truncated to the first 84 rows and columns is NOT
 * orthogonal, so the firmware builds the Paley type-I Hadamard matrix of
 * order q+1 = 84 (q = 83 is prime and 3 mod 4):
 *   H[0][*] = H[*][0] = +1,
 *   H[p][i] = chi((p-1)-(i-1)) - delta(p,i), p,i >= 1,
 * where chi is the quadratic-residue character modulo 83.  Rows are
 * patterns, columns are channels, and code -1 commands a pi phase shift. */
static const uint8_t cal_residue83[83] = {
  0,1,0,1,1,0,0,1,0,1,1,1,1,0,0,0,1,1,0,0,0,1,0,1,0,1,1,1,1,1,1,1,0,1,0,0,1,1,1,0,1,1,0,0,1,0,0,0,1,1,0,1,0,0,0,0,0,0,0,1,0,1,0,1,1,1,0,0,1,1,1,0,0,0,0,1,0,1,1,0,0,1,0
};

static uint8_t cal_code_bits[CAL_CHANNELS][(CAL_CHANNELS + 7u) / 8u];

static int8_t cal_code_sign(uint8_t pattern, uint8_t channel)
{
  int d;
  if (pattern == 0u || channel == 0u) return 1;
  d = (int)pattern - (int)channel;
  d %= 83;
  if (d < 0) d += 83;
  if (d == 0) return -1;
  return cal_residue83[d] != 0u ? 1 : -1;
}

static void cal_code_init(void)
{
  uint8_t pattern, channel;
  memset(cal_code_bits, 0, sizeof(cal_code_bits));
  for (pattern = 0u; pattern < CAL_CHANNELS; ++pattern) {
    for (channel = 0u; channel < CAL_CHANNELS; ++channel) {
      if (cal_code_sign(pattern, channel) < 0)
        cal_code_bits[pattern][channel >> 3] |= (uint8_t)(1u << (channel & 7u));
    }
  }
}

static uint8_t cal_code(uint8_t pattern, uint8_t channel)
{
  return (cal_code_bits[pattern][channel >> 3] & (uint8_t)(1u << (channel & 7u))) != 0u ?
         128u : 0u;
}

static void cal_report(us_cal_progress_cb_t cb, void *context, uint8_t state, uint8_t progress)
{
  if (cb != NULL) cb(state, progress, context);
}

static int cal_wait_credit(fpga_link_t *link, uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  while (fpga_link_status(link)->fifo_credit == 0u) {
    if ((HAL_GetTick() - start) > timeout_ms) return -1;
    if (fpga_link_poll_status(link) != 0) return -2;
    osDelay(1u);
  }
  return 0;
}

/* All channels off.  The off frame is swapped while the microphone gate
 * sequencer is already collecting, so it does not restart the block: it only
 * terminates the transmit burst and lets the wall echo be time-gated. */
static int cal_submit_silence(fpga_link_t *link)
{
  umh_output_frame_t frame;
  uint16_t i;
  if (link == NULL) return -1;
  memset(&frame, 0, sizeof(frame));
  frame.update_flags = UMH_FRAME_FLAG_ULTRASOUND;
  frame.sequence = ++cal_frame_sequence;
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    frame.channels[i].phase = 0u;
    frame.channels[i].level = 0u;
  }
  if (cal_wait_credit(link, 200u) != 0) return -2;
  return fpga_link_submit(link, &frame);
}

/* Transmit one orthogonal pattern as a burst.  A 1 ms on-time is long enough
 * for the 40 kHz transducers to reach steady state but ends before the wall
 * echo of any reasonable distance, so late gates contain only the echo. */
static int cal_submit_pattern(fpga_link_t *link, uint8_t pattern, const uint8_t *correction)
{
  umh_output_frame_t frame;
  uint16_t i;
  if (link == NULL) return -1;
  memset(&frame, 0, sizeof(frame));
  frame.update_flags = UMH_FRAME_FLAG_ULTRASOUND;
  frame.sequence = ++cal_frame_sequence;
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    uint8_t phase = cal_code(pattern, i);
    if (correction != NULL) phase = (uint8_t)(phase + correction[i]);
    frame.channels[i].phase = phase;
    frame.channels[i].level = 128u;
  }
  if (cal_wait_credit(link, 200u) != 0) return -2;
  if (fpga_link_submit(link, &frame) != 0) return -3;
  if (CAL_BURST_US != 0u) {
    osDelay((CAL_BURST_US + 999u) / 1000u);
    /* STOP is a hard carrier-stop, independent of the event-table swap. */
    if (fpga_link_safe_stop(link) != 0) return -4;
  }
  return 0;
}
static int cal_survey_collect(fpga_link_t *link)
{
  uint8_t pattern, gate, mic;
  if (mic_capture_configure(link, CAL_SURVEY_GATES, 0u, CAL_SURVEY_STEP,
                            CAL_SURVEY_WIDTH) != 0) return -1;
  memset(cal_survey_power, 0, sizeof(cal_survey_power));
  memset(cal_survey_noise, 0, sizeof(cal_survey_noise));
  cal_block_expected = 0u;

  /* Transmitter-off probe: quantifies the demodulator / microphone noise and
   * any residual ringing floor per gate. */
  if (cal_submit_silence(link) != 0) return -5;
  if (mic_capture_wait_block(link, 1u, 150u, NULL) != 0) return -6;
  cal_block_expected = 1u;
  for (gate = 0u; gate < CAL_SURVEY_GATES; ++gate) {
    mic_capture_gate_t sample;
    if (mic_capture_read_gate(link, gate, &sample, NULL) != 0) return -7;
    for (mic = 0u; mic < CAL_MICS; ++mic) {
      float iv = (float)sample.i[mic];
      float qv = (float)sample.q[mic];
      cal_survey_noise[gate] += iv * iv + qv * qv;
    }
  }

  for (pattern = 0u; pattern < CAL_SURVEY_PATTERNS; ++pattern) {
    mic_capture_gate_t sample;
    uint16_t expected = (uint16_t)(pattern + 2u);
    if (cal_submit_pattern(link, pattern, NULL) != 0) return -10;
    if (mic_capture_wait_block(link, expected, 150u, NULL) != 0) return -20;
    cal_block_expected = expected;
    for (gate = 0u; gate < CAL_SURVEY_GATES; ++gate) {
      if (mic_capture_read_gate(link, gate, &sample, NULL) != 0) return -30;
      for (mic = 0u; mic < CAL_MICS; ++mic) {
        float iv = (float)sample.i[mic];
        float qv = (float)sample.q[mic];
        cal_survey_power[gate] += iv * iv + qv * qv;
      }
    }
  }
  return 0;
}
static uint8_t cal_find_arrival(void)
{
  float net[CAL_SURVEY_GATES];
  float smooth[CAL_SURVEY_GATES];
  float sorted[CAL_SURVEY_GATES];
  float noise = 0.0f, peak = 0.0f, echo_peak = 0.0f;
  uint8_t i, j, peak_idx = 0u, echo_idx = (uint8_t)(CAL_SURVEY_GATES / 2u);
  const uint8_t noise_count = 16u;
  for (i = 0u; i < CAL_SURVEY_GATES; ++i) {
    float expected_noise = cal_survey_noise[i] * (float)CAL_SURVEY_PATTERNS;
    net[i] = cal_survey_power[i] - expected_noise;
    if (net[i] < 0.0f) net[i] = 0.0f;
  }
  for (i = 0u; i < CAL_SURVEY_GATES; ++i) {
    float prev = net[i > 0u ? i - 1u : 0u];
    float next = net[i + 1u < CAL_SURVEY_GATES ? i + 1u : i];
    smooth[i] = 0.25f * prev + 0.5f * net[i] + 0.25f * next;
    sorted[i] = smooth[i];
  }
  for (i = 1u; i < CAL_SURVEY_GATES; ++i) {
    float value = sorted[i];
    j = i;
    while (j > 0u && sorted[j - 1u] > value) {
      sorted[j] = sorted[j - 1u];
      --j;
    }
    sorted[j] = value;
  }
  for (i = 0u; i < noise_count; ++i) noise += sorted[i];
  noise /= (float)noise_count;
  for (i = 0u; i < CAL_SURVEY_GATES; ++i) {
    if (smooth[i] > peak) { peak = smooth[i]; peak_idx = i; }
  }
  for (i = CAL_DIRECT_SKIP_GATES; i < CAL_SURVEY_GATES; ++i) {
    if (smooth[i] > echo_peak) { echo_peak = smooth[i]; echo_idx = i; }
  }
  cal_baseline_power = noise;
  if (peak < 1.0f) return (uint8_t)(CAL_SURVEY_GATES / 2u);        /* totally silent */
  if (echo_peak < noise + 0.15f * (peak - noise)) return peak_idx; /* no clear echo */
  return echo_idx;
}
static int cal_final_collect(fpga_link_t *link, uint16_t start, uint16_t step,
                             uint8_t width, us_cal_progress_cb_t cb, void *context)
{
  uint8_t pattern, gate, mic;
  if (mic_capture_configure(link, CAL_FINAL_GATES, start, step, width) != 0) return -1;
  memset(cal_final_noise, 0, sizeof(cal_final_noise));
  cal_block_expected = 0u;
  if (cal_submit_silence(link) != 0) return -4;
  if (mic_capture_wait_block(link, 1u, 150u, NULL) != 0) return -5;
  cal_block_expected = 1u;
  for (gate = 0u; gate < CAL_FINAL_GATES; ++gate) {
    mic_capture_gate_t sample;
    if (mic_capture_read_gate(link, gate, &sample, NULL) != 0) return -6;
    for (mic = 0u; mic < CAL_MICS; ++mic) {
      float iv = (float)sample.i[mic];
      float qv = (float)sample.q[mic];
      cal_final_noise[gate] += iv * iv + qv * qv;
    }
  }
  for (pattern = 0u; pattern < CAL_CHANNELS; ++pattern) {
    mic_capture_gate_t sample;
    uint16_t expected = (uint16_t)(pattern + 2u);
    if (cal_submit_pattern(link, pattern, NULL) != 0) return -1;
    if (mic_capture_wait_block(link, expected, 150u, NULL) != 0) return -2;
    cal_block_expected = expected;
    for (gate = 0u; gate < CAL_FINAL_GATES; ++gate) {
      if (mic_capture_read_gate(link, gate, &sample, NULL) != 0) return -3;
      for (mic = 0u; mic < CAL_MICS; ++mic) {
        cal_raw[gate][pattern][mic][0] = sample.i[mic];
        cal_raw[gate][pattern][mic][1] = sample.q[mic];
      }
    }
    if ((pattern & 7u) == 0u)
      cal_report(cb, context, US_CAL_MEASURE, (uint8_t)((pattern * 100u) / CAL_CHANNELS));
  }
  cal_report(cb, context, US_CAL_MEASURE, 100u);
  return 0;
}
static void cal_decode_final(void)
{
  uint8_t gate, mic, channel, pattern;
  float scale = 1.0f / ((float)CAL_CHANNELS * (float)CAL_FINAL_GATES);
  memset(cal_zre, 0, sizeof(cal_zre));
  memset(cal_zim, 0, sizeof(cal_zim));
  for (gate = 0u; gate < CAL_FINAL_GATES; ++gate) {
    for (mic = 0u; mic < CAL_MICS; ++mic) {
      for (channel = 0u; channel < CAL_CHANNELS; ++channel) {
        float sumre = 0.0f, sumim = 0.0f;
        for (pattern = 0u; pattern < CAL_CHANNELS; ++pattern) {
          uint8_t negative = (uint8_t)((cal_code_bits[pattern][channel >> 3] >>
                                        (channel & 7u)) & 1u);
          float iv = (float)cal_raw[gate][pattern][mic][0];
          float qv = (float)cal_raw[gate][pattern][mic][1];
          if (negative != 0u) { sumre -= iv; sumim -= qv; }
          else { sumre += iv; sumim += qv; }
        }
        cal_zre[channel][mic] += sumre * scale;
        cal_zim[channel][mic] += sumim * scale;
      }
    }
  }
}
static void cal_paths(float D_mm, float tilt_x_deg, float tilt_y_deg)
{
  float tx = tilt_x_deg * CAL_PI / 180.0f;
  float ty = tilt_y_deg * CAL_PI / 180.0f;
  float sx = sinf(tx), sy = sinf(ty);
  float c2 = 1.0f - sx * sx - sy * sy;
  float sz;
  uint8_t i, mic;
  if (c2 < 0.05f) c2 = 0.05f;
  sz = sqrtf(c2);
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    float px = cal_chan_x_mm[i], py = cal_chan_y_mm[i];
    float nd = px * sx + py * sy;
    float offset = 2.0f * (D_mm - nd);
    float ix = px + offset * sx;
    float iy = py + offset * sy;
    float iz = offset * sz;
    for (mic = 0u; mic < CAL_MICS; ++mic) {
      float dx = ix - cal_mic_x_mm[mic];
      float dy = iy - cal_mic_y_mm[mic];
      float dz = iz;
      cal_path[i][mic] = sqrtf(dx * dx + dy * dy + dz * dz);
    }
  }
}

static float cal_pose_cost(float D_mm, float tilt_x_deg, float tilt_y_deg)
{
  uint8_t i, pair;
  float total = 0.0f;
  cal_paths(D_mm, tilt_x_deg, tilt_y_deg);
  for (pair = 0u; pair < 6u; ++pair) {
    uint8_t j = cal_pair_j[pair], k = cal_pair_k[pair];
    float sr = 0.0f, si = 0.0f, sw = 0.0f;
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float mre = cal_zre[i][j] * cal_zre[i][k] + cal_zim[i][j] * cal_zim[i][k];
      float mim = cal_zim[i][j] * cal_zre[i][k] - cal_zre[i][j] * cal_zim[i][k];
      float g = cal_k_wave_mm * (cal_path[i][j] - cal_path[i][k]);
      float c = cosf(g), s = sinf(g);
      sr += mre * c - mim * s;
      si += mre * s + mim * c;
      sw += sqrtf(mre * mre + mim * mim);
    }
    total += 1.0f - sqrtf(sr * sr + si * si) / (sw + 1.0e-9f);
  }
  return total;
}

static void cal_pose_refine(float *D_m, float *tilt_x, float *tilt_y)
{
  float step_d = 20.0f, step_t = 1.0f;   /* mm and degrees */
  float best = cal_pose_cost(*D_m * 1000.0f, *tilt_x, *tilt_y);
  int iter;
  for (iter = 0; iter < 80; ++iter) {
    int improved = 0;
    if ((iter & 7) == 0) osDelay(1u);
    float cand;
    cand = cal_pose_cost((*D_m + step_d / 1000.0f) * 1000.0f, *tilt_x, *tilt_y);
    if (cand < best) { best = cand; *D_m += step_d / 1000.0f; improved = 1; }
    cand = cal_pose_cost((*D_m - step_d / 1000.0f) * 1000.0f, *tilt_x, *tilt_y);
    if (cand < best) { best = cand; *D_m -= step_d / 1000.0f; improved = 1; }
    cand = cal_pose_cost(*D_m * 1000.0f, *tilt_x + step_t, *tilt_y);
    if (cand < best) { best = cand; *tilt_x += step_t; improved = 1; }
    cand = cal_pose_cost(*D_m * 1000.0f, *tilt_x - step_t, *tilt_y);
    if (cand < best) { best = cand; *tilt_x -= step_t; improved = 1; }
    cand = cal_pose_cost(*D_m * 1000.0f, *tilt_x, *tilt_y + step_t);
    if (cand < best) { best = cand; *tilt_y += step_t; improved = 1; }
    cand = cal_pose_cost(*D_m * 1000.0f, *tilt_x, *tilt_y - step_t);
    if (cand < best) { best = cand; *tilt_y -= step_t; improved = 1; }
    if (improved == 0) {
      step_d *= 0.5f;
      step_t *= 0.5f;
      if (step_d < 0.5f && step_t < 0.05f) break;
    }
  }
}

static void cal_pose_search(float D0_m, float *D_m, float *tilt_x, float *tilt_y)
{
  float dmin, dmax;
  float best = 1.0e30f;
  int di, ti, ui;
  if (D0_m >= 0.3f && D0_m <= 2.5f) { dmin = D0_m * 0.55f; dmax = D0_m * 1.70f; }
  else { dmin = 0.15f; dmax = 3.0f; }
  if (dmin < 0.12f) dmin = 0.12f;
  if (dmax > 3.0f) dmax = 3.0f;
  for (di = 0; di < 6; ++di) {
    float D = (di == 0) ? dmin : dmin * powf(dmax / dmin, (float)di / 5.0f);
    for (ti = -8; ti <= 8; ++ti) {
      for (ui = -8; ui <= 8; ++ui) {
        float tx = (float)ti * CAL_GRID_TILT_DEG;
        float ty = (float)ui * CAL_GRID_TILT_DEG;
        float cost = cal_pose_cost(D * 1000.0f, tx, ty);
        if (cost < best) { best = cost; *D_m = D; *tilt_x = tx; *tilt_y = ty; }
      }
    }
    osDelay(1u);
  }
  /* One-degree local grid around the coarse minimum. */
  {
    float dc = *D_m, xc = *tilt_x, yc = *tilt_y;
    for (di = -2; di <= 2; ++di) {
      for (ti = -4; ti <= 4; ++ti) {
        for (ui = -4; ui <= 4; ++ui) {
          float D = dc * (1.0f + 0.06f * (float)di);
          float tx = xc + (float)ti;
          float ty = yc + (float)ui;
          float cost;
          if (fabsf(tx) > CAL_MAX_TILT || fabsf(ty) > CAL_MAX_TILT) continue;
          cost = cal_pose_cost(D * 1000.0f, tx, ty);
          if (cost < best) { best = cost; *D_m = D; *tilt_x = tx; *tilt_y = ty; }
        }
      }
      osDelay(1u);
    }
  }
  cal_pose_refine(D_m, tilt_x, tilt_y);
}

static void cal_dvalue(uint8_t channel, uint8_t mic, float *dr, float *di)
{
  float g = cal_k_wave_mm * cal_path[channel][mic];
  float c = cosf(g), s = sinf(g);
  float zr = cal_zre[channel][mic], zi = cal_zim[channel][mic];
  *dr = zr * c - zi * s;
  *di = zr * s + zi * c;
}

static void cal_channel_at_mic(uint8_t channel, uint8_t mic, float *re, float *im)
{
  float dr, di;
  float rr = cal_rho_re[mic], ri = cal_rho_im[mic];
  float den = rr * rr + ri * ri;
  cal_dvalue(channel, mic, &dr, &di);
  if (den < 1.0e-12f) den = 1.0e-12f;
  *re = (dr * rr + di * ri) / den;
  *im = (di * rr - dr * ri) / den;
}

static void cal_fit_arho(float D_mm, float tilt_x, float tilt_y,
                         float *residual, float *consistency_deg)
{
  uint8_t i, j, iter;
  float denz = 0.0f, numerr = 0.0f;
  float pair_cos[6], pair_sin[6];
  cal_paths(D_mm, tilt_x, tilt_y);
  for (j = 0u; j < CAL_MICS; ++j) {
    float sr = 0.0f, si = 0.0f, sw = 0.0f;
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float dr, di, w;
      cal_dvalue(i, j, &dr, &di);
      w = sqrtf(dr * dr + di * di);
      sr += w * dr; si += w * di; sw += w;
    }
    if (sw < 1.0e-12f) sw = 1.0e-12f;
    cal_rho_re[j] = sr / sw;
    cal_rho_im[j] = si / sw;
  }
  for (iter = 0u; iter < 6u; ++iter) {
    float arho2 = 0.0f;
    for (j = 0u; j < CAL_MICS; ++j) arho2 += cal_rho_re[j] * cal_rho_re[j] + cal_rho_im[j] * cal_rho_im[j];
    if (arho2 < 1.0e-12f) arho2 = 1.0e-12f;
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float ar = 0.0f, ai = 0.0f;
      for (j = 0u; j < CAL_MICS; ++j) {
        float dr, di;
        cal_dvalue(i, j, &dr, &di);
        ar += dr * cal_rho_re[j] + di * cal_rho_im[j];
        ai += di * cal_rho_re[j] - dr * cal_rho_im[j];
      }
      cal_a_re[i] = ar / arho2;
      cal_a_im[i] = ai / arho2;
    }
    {
      float aa2 = 0.0f;
      for (i = 0u; i < CAL_CHANNELS; ++i) aa2 += cal_a_re[i] * cal_a_re[i] + cal_a_im[i] * cal_a_im[i];
      if (aa2 < 1.0e-12f) aa2 = 1.0e-12f;
      for (j = 0u; j < CAL_MICS; ++j) {
        float rr = 0.0f, ri = 0.0f;
        for (i = 0u; i < CAL_CHANNELS; ++i) {
          float dr, di;
          cal_dvalue(i, j, &dr, &di);
          rr += dr * cal_a_re[i] + di * cal_a_im[i];
          ri += di * cal_a_re[i] - dr * cal_a_im[i];
        }
        cal_rho_re[j] = rr / aa2;
        cal_rho_im[j] = ri / aa2;
      }
    }
  }
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    cal_phi[i] = atan2f(cal_a_im[i], cal_a_re[i]);
    for (j = 0u; j < CAL_MICS; ++j) {
      float dr, di, mr, mi, er, ei;
      cal_dvalue(i, j, &dr, &di);
      mr = cal_a_re[i] * cal_rho_re[j] - cal_a_im[i] * cal_rho_im[j];
      mi = cal_a_re[i] * cal_rho_im[j] + cal_a_im[i] * cal_rho_re[j];
      er = dr - mr; ei = di - mi;
      numerr += er * er + ei * ei;
      denz += dr * dr + di * di;
    }
  }
  if (residual != NULL) *residual = numerr / (denz + 1.0e-12f);
  if (consistency_deg == NULL) return;
  for (j = 0u; j < 6u; ++j) { pair_cos[j] = 0.0f; pair_sin[j] = 0.0f; }
  for (j = 0u; j < 6u; ++j) {
    uint8_t mj = cal_pair_j[j], mk = cal_pair_k[j];
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float jr, ji, kr, ki;
      cal_channel_at_mic(i, mj, &jr, &ji);
      cal_channel_at_mic(i, mk, &kr, &ki);
      {
        float diff = atan2f(ji, jr) - atan2f(ki, kr);
        diff = cal_wrap_pi(diff);
        pair_cos[j] += cosf(diff);
        pair_sin[j] += sinf(diff);
      }
    }
  }
  {
    float sum = 0.0f;
    for (j = 0u; j < 6u; ++j) {
      float mean = atan2f(pair_sin[j], pair_cos[j]);
      for (i = 0u; i < CAL_CHANNELS; ++i) {
        float jr, ji, kr, ki, diff;
        cal_channel_at_mic(i, cal_pair_j[j], &jr, &ji);
        cal_channel_at_mic(i, cal_pair_k[j], &kr, &ki);
        diff = cal_wrap_pi(atan2f(ji, jr) - atan2f(ki, kr) - mean);
        sum += diff * diff;
      }
    }
    *consistency_deg = cal_deg(sqrtf(sum / (float)(6u * CAL_CHANNELS)));
  }
}

static float cal_gauge(const float *phi_rad, float *out_deg)
{
  uint8_t i;
  float sr = 0.0f, si = 0.0f, mean, rms = 0.0f;
  static float v[CAL_CHANNELS];
  double xx = 0.0, xy = 0.0, yy = 0.0, x = 0.0, y = 0.0;
  double sv = 0.0, xv = 0.0, yv = 0.0;
  double a[3][4], det;
  for (i = 0u; i < CAL_CHANNELS; ++i) { sr += cosf(phi_rad[i]); si += sinf(phi_rad[i]); }
  mean = atan2f(si, sr);
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    double px = cal_chan_x_mm[i], py = cal_chan_y_mm[i];
    v[i] = cal_wrap_pi(phi_rad[i] - mean);
    xx += px * px; xy += px * py; yy += py * py; x += px; y += py;
    xv += px * v[i]; yv += py * v[i]; sv += v[i];
    rms += v[i] * v[i];
  }
  /* Least-squares plane p0 + px*x + py*y over the channel coordinates. */
  a[0][0] = xx; a[0][1] = xy; a[0][2] = x;  a[0][3] = xv;
  a[1][0] = xy; a[1][1] = yy; a[1][2] = y;  a[1][3] = yv;
  a[2][0] = x;  a[2][1] = y;  a[2][2] = (double)CAL_CHANNELS; a[2][3] = sv;
  {
    int c, r, pivot;
    for (c = 0; c < 3; ++c) {
      pivot = c;
      for (r = c + 1; r < 3; ++r)
        if (fabs(a[r][c]) > fabs(a[pivot][c])) pivot = r;
      if (pivot != c) {
        int kk;
        for (kk = c; kk < 4; ++kk) { double t = a[c][kk]; a[c][kk] = a[pivot][kk]; a[pivot][kk] = t; }
      }
      det = a[c][c];
      if (fabs(det) < 1.0e-12) det = 1.0e-12;
      for (r = c + 1; r < 3; ++r) {
        double f = a[r][c] / det;
        int kk;
        for (kk = c; kk < 4; ++kk) a[r][kk] -= f * a[c][kk];
      }
    }
    for (r = 2; r >= 0; --r) {
      double sum = a[r][3];
      int kk;
      for (kk = r + 1; kk < 3; ++kk) sum -= a[r][kk] * a[kk][3];
      a[r][3] = sum / a[r][r];
    }
  }
  rms = sqrtf(rms / (float)CAL_CHANNELS) * (180.0f / CAL_PI);
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    double fit = a[0][3] * cal_chan_x_mm[i] + a[1][3] * cal_chan_y_mm[i] + a[2][3];
    out_deg[i] = cal_deg(cal_wrap_pi((float)(v[i] - fit)));
  }
  return rms;
}

#if !CAL_QUALITY_RELAXED
static int cal_measure_power(fpga_link_t *link, const uint8_t *correction, float *power_out)
{
  uint8_t gate, mic;
  float power = 0.0f;
  if (cal_submit_pattern(link, 0u, correction) != 0) return -1;
  if (mic_capture_wait_block(link, (uint16_t)(cal_block_expected + 1u), 200u, NULL) != 0) return -2;
  cal_block_expected = (uint16_t)(cal_block_expected + 1u);
  for (gate = 0u; gate < CAL_FINAL_GATES; ++gate) {
    mic_capture_gate_t sample;
    if (mic_capture_read_gate(link, gate, &sample, NULL) != 0) return -3;
    for (mic = 0u; mic < CAL_MICS; ++mic) {
      float iv = (float)sample.i[mic], qv = (float)sample.q[mic];
      power += iv * iv + qv * qv;
    }
  }
  *power_out = power;
  return 0;
}

/* Test every candidate correction and both global polarities, then keep the
 * one that produces the strongest coherent wall echo.  This resolves the
 * mirror (tilt sign) ambiguity of a microphone array that is coplanar with
 * the transmit aperture. */
static int cal_verify(fpga_link_t *link, uint8_t candidate_count,
                      uint8_t *chosen, float *gain_db)
{
  uint8_t negated[CAL_CHANNELS];
  uint8_t variant[CAL_CHANNELS];
  uint8_t c, i, best_index = 0u, best_negated = 0u;
  float p0 = 0.0f, best_power = -1.0f;
  uint8_t gate, mic;
  for (gate = 0u; gate < CAL_FINAL_GATES; ++gate) {
    for (mic = 0u; mic < CAL_MICS; ++mic) {
      float iv = (float)cal_raw[gate][0][mic][0];
      float qv = (float)cal_raw[gate][0][mic][1];
      p0 += iv * iv + qv * qv;
    }
  }
  if (p0 < 1.0f || candidate_count == 0u) return -1;
  for (c = 0u; c < candidate_count; ++c) {
    const uint8_t *cand = (c == 0u) ? cal_corr_a : cal_corr_b;
    float power;
    if (cal_measure_power(link, cand, &power) != 0) return -2;
    if (power > best_power) { best_power = power; best_index = c; best_negated = 0u; }
    for (i = 0u; i < CAL_CHANNELS; ++i) negated[i] = (uint8_t)(0u - cand[i]);
    if (cal_measure_power(link, negated, &power) != 0) return -3;
    if (power > best_power) { best_power = power; best_index = c; best_negated = 1u; }
  }
  memcpy(variant, (best_index == 0u) ? cal_corr_a : cal_corr_b, CAL_CHANNELS);
  if (best_negated != 0u) {
    for (i = 0u; i < CAL_CHANNELS; ++i) variant[i] = (uint8_t)(0u - variant[i]);
  }
  memcpy(chosen, variant, CAL_CHANNELS);
  *gain_db = 10.0f * log10f((best_power + 1.0f) / (p0 + 1.0f));
  return 0;
}
#endif
uint32_t us_calibration_raw_size(void) { return (uint32_t)sizeof(cal_raw); }

int us_calibration_raw_read(uint32_t offset, uint8_t *out, uint16_t length)
{
  const uint8_t *src = (const uint8_t *)cal_raw;
  uint32_t size = (uint32_t)sizeof(cal_raw);
  if (out == NULL || offset >= size) return -1;
  if ((uint32_t)length > (size - offset)) length = (uint16_t)(size - offset);
  memcpy(out, src + offset, length);
  return (int)length;
}

int us_calibration_run(fpga_link_t *link, const umh_device_profile_t *profile,
                       us_cal_progress_cb_t progress, void *context,
                       umh_calibration_result_t *result)
{
  uint8_t i;
  uint16_t final_start, final_step;
  uint16_t survey_span;
  float D0_m, D_m, tilt_x = 0.0f, tilt_y = 0.0f;
  float residual = 1.0f, consistency = 999.0f, rms_pre, rms_post;
  float echo_ratio, mic_ratio;
  float mean_amp[CAL_MICS];
  int rc;

  if (link == NULL || profile == NULL || result == NULL) return -1;
  memset(result, 0, sizeof(*result));
  result->fault = UMH_FAULT_NONE;
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    cal_chan_x_mm[i] = (float)profile->coordinates[i].x_um * 0.001f;
    cal_chan_y_mm[i] = (float)profile->coordinates[i].y_um * 0.001f;
  }
  {
    float c_mm_s = profile->sound_speed_um_per_s != 0u ?
                   (float)profile->sound_speed_um_per_s * 0.001f : 343000.0f;
    float f_hz = (float)(profile->carrier_hz != 0u ? profile->carrier_hz : 40000u);
    cal_k_wave_mm = CAL_TWO_PI * f_hz / c_mm_s;
  }
  cal_code_init();
  cal_frame_sequence = 0u;
  cal_block_expected = 0u;
  cal_report(progress, context, US_CAL_WAIT, 0u);

  rc = cal_survey_collect(link);
  if (rc != 0) {
    result->quality_flags |= CAL_Q_MEASURE;
    result->fault = UMH_FAULT_CAL_MIC_SILENT;
    goto fail;
  }
  cal_report(progress, context, US_CAL_SOLVE, 5u);
  cal_arrival_gate = cal_find_arrival();

  {
    float arrival_samples = (float)cal_arrival_gate * (float)CAL_SURVEY_STEP
                          + 0.5f * (float)CAL_SURVEY_WIDTH;
    float arrival_s = arrival_samples * 10.0e-6f;
    float c_m_s = profile->sound_speed_um_per_s != 0u ?
                  (float)profile->sound_speed_um_per_s * 1.0e-6f : 343.0f;
    D0_m = 0.5f * c_m_s * arrival_s;
    if (D0_m < 0.15f) D0_m = 0.15f;
    if (D0_m > 3.0f) D0_m = 3.0f;
  }
  survey_span = (uint16_t)((CAL_SURVEY_GATES - 1u) * CAL_SURVEY_STEP + CAL_SURVEY_WIDTH);
  {
    /* Four contiguous 400 us gates centred on the detected echo peak. */
    int32_t peak_samples = (int32_t)cal_arrival_gate * CAL_SURVEY_STEP;
    int32_t span = (int32_t)((CAL_FINAL_GATES - 1u) * CAL_FINAL_WIDTH + CAL_FINAL_WIDTH);
    int32_t start = peak_samples - (int32_t)CAL_FINAL_WIDTH;
    int32_t latest = (int32_t)survey_span - span;
    if (start > latest) start = latest;
    if (start < 0) start = 0;
    final_start = (uint16_t)start;
    final_step = CAL_FINAL_WIDTH;
  }
  cal_report(progress, context, US_CAL_MEASURE, 0u);
  rc = cal_final_collect(link, final_start, final_step, CAL_FINAL_WIDTH, progress, context);
  if (rc != 0) {
    result->quality_flags |= CAL_Q_MEASURE;
    result->fault = UMH_FAULT_CAL_MIC_SILENT;
    goto fail;
  }
  cal_report(progress, context, US_CAL_SOLVE, 20u);

  cal_decode_final();
  cal_pose_search(D0_m, &D_m, &tilt_x, &tilt_y);
  cal_fit_arho(D_m * 1000.0f, tilt_x, tilt_y, &residual, &consistency);
  cal_report(progress, context, US_CAL_SOLVE, 80u);
  /* Gauge-fix the channel phases and build the direct correction candidate.
   * Because the four microphones are coplanar with the aperture, the mirror
   * tilt solution is nearly as good; keep both candidates and let the
   * hardware echo-power check choose the one that actually focuses on the
   * wall. */
  rms_pre = cal_gauge(cal_phi, cal_fit_deg);
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    float correction_deg = -cal_fit_deg[i];
    int32_t q = (int32_t)lroundf(correction_deg * (256.0f / 360.0f)) % 256;
    if (q < 0) q += 256;
    cal_corr_a[i] = (uint8_t)q;
  }
  memcpy(cal_corr_b, cal_corr_a, CAL_CHANNELS);
  rms_post = 0.0f;
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    float applied = (float)cal_corr_a[i] * (360.0f / 256.0f);
    float err;
    while (applied > 180.0f) applied -= 360.0f;
    err = cal_fit_deg[i] + applied;
    rms_post += err * err;
  }
  rms_post = sqrtf(rms_post / (float)CAL_CHANNELS);
  {
    float mx = -tilt_x, my = -tilt_y;
    if (fabsf(mx - tilt_x) > 0.01f || fabsf(my - tilt_y) > 0.01f) {
      float mirror_residual, mirror_consistency;
      cal_fit_arho(D_m * 1000.0f, mx, my, &mirror_residual, &mirror_consistency);
      (void)cal_gauge(cal_phi, cal_fit_deg);
      for (i = 0u; i < CAL_CHANNELS; ++i) {
        float correction_deg = -cal_fit_deg[i];
        int32_t q = (int32_t)lroundf(correction_deg * (256.0f / 360.0f)) % 256;
        if (q < 0) q += 256;
        cal_corr_b[i] = (uint8_t)q;
      }
    }
  }

  result->rms_before_deg = rms_pre;
  result->rms_after_deg = rms_post;
  result->mic_consistency_deg = consistency;
  result->residual = residual;
  result->distance_m = D_m;
  result->tilt_x_deg = tilt_x;
  result->tilt_y_deg = tilt_y;
  result->used_gate_start = final_start;
  result->used_gate_count = CAL_FINAL_GATES;
  result->used_gate_width = CAL_FINAL_WIDTH;

  for (i = 0u; i < CAL_MICS; ++i) mean_amp[i] = 0.0f;
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    for (uint8_t mic = 0u; mic < CAL_MICS; ++mic)
      mean_amp[mic] += sqrtf(cal_zre[i][mic] * cal_zre[i][mic] + cal_zim[i][mic] * cal_zim[i][mic]);
  }
  for (i = 0u; i < CAL_MICS; ++i) mean_amp[i] /= (float)CAL_CHANNELS;
  mic_ratio = 0.0f;
  {
    float mn = mean_amp[0], mx = mean_amp[0];
    for (i = 1u; i < CAL_MICS; ++i) { if (mean_amp[i] < mn) mn = mean_amp[i]; if (mean_amp[i] > mx) mx = mean_amp[i]; }
    mic_ratio = (mn > 1.0e-9f) ? (mx / mn) : 999.0f;
  }
  {
    float sig = 0.0f, noise = 0.0f;
    uint8_t gate, pattern, mic;
    for (gate = 0u; gate < CAL_FINAL_GATES; ++gate)
      for (pattern = 0u; pattern < CAL_CHANNELS; ++pattern)
        for (mic = 0u; mic < CAL_MICS; ++mic) {
          float iv = (float)cal_raw[gate][pattern][mic][0];
          float qv = (float)cal_raw[gate][pattern][mic][1];
          sig += iv * iv + qv * qv;
        }
    sig /= (float)(CAL_FINAL_GATES * CAL_CHANNELS * CAL_MICS);
    for (gate = 0u; gate < CAL_FINAL_GATES; ++gate)
      noise += cal_final_noise[gate] / (float)CAL_MICS;
    noise /= (float)CAL_FINAL_GATES;
    {
      float net_sig = sig - noise;
      if (net_sig < 1.0f) net_sig = 1.0f;
      echo_ratio = sqrtf(net_sig / (noise + 1.0e-9f));
    }
  }  result->echo_ratio = echo_ratio;
  result->mic_ratio = mic_ratio;
  if (echo_ratio < CAL_MIN_ECHO_RATIO) result->quality_flags |= CAL_Q_ECHO;
  if (mic_ratio > CAL_MAX_MIC_RATIO) result->quality_flags |= CAL_Q_MIC_RATIO;
  if (residual > CAL_MAX_RESIDUAL) result->quality_flags |= CAL_Q_RESIDUAL;
  if (consistency > CAL_MAX_MIC_CONSIST) result->quality_flags |= CAL_Q_CONSISTENCY;
  if (rms_pre > CAL_MAX_PRERMS) result->quality_flags |= CAL_Q_PRERMS;
  if (fabsf(tilt_x) > CAL_MAX_TILT || fabsf(tilt_y) > CAL_MAX_TILT)
    result->quality_flags |= CAL_Q_TILT;
#if !CAL_QUALITY_RELAXED
  if (result->quality_flags != 0u) {
    result->fault = UMH_FAULT_CAL_QUALITY;
    goto fail;
  }
#endif
#if CAL_QUALITY_RELAXED
  /* Bring-up: no hardware array-gain check, use the model candidate A. */
  memcpy(result->phase_byte, cal_corr_a, CAL_CHANNELS);
  result->verify_gain_db = 0.0f;
  cal_report(progress, context, US_CAL_VERIFY, 0u);
#else
  {
    float gain_db = -999.0f;
    cal_report(progress, context, US_CAL_VERIFY, 0u);
    if (cal_verify(link, 2u, result->phase_byte, &gain_db) != 0) {
      result->quality_flags |= CAL_Q_VERIFY;
      result->fault = UMH_FAULT_CAL_QUALITY;
      goto fail;
    }
    result->verify_gain_db = gain_db;
    if (gain_db < CAL_MIN_VERIFY_DB && rms_pre > 20.0f) {
      result->quality_flags |= CAL_Q_VERIFY;
      result->fault = UMH_FAULT_CAL_QUALITY;
      goto fail;
    }
  }
#endif
  result->final_block_count = cal_block_expected;
  result->good_mics = 4u;
  result->progress = 100u;
  cal_report(progress, context, US_CAL_OK, 100u);
  return 0;

fail:
  result->progress = 100u;
  cal_report(progress, context, US_CAL_FAIL, 100u);
  return -2;
}
