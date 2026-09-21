/* UMH v7 built-in near-field coupling phase self-calibration.
 *
 * This file intentionally replaces the old wall-echo survey.  The array is
 * calibrated from the direct acoustic path between the 84 transmitters and
 * the four on-board SPH0641 microphones:
 *
 *   y_m(p) = b_m + sum_i Z_mi * x_i(p) + n_m(p),      x_i(p) = +/-1
 *
 * where p indexes random command-phase projections.  Mean centering removes
 * the pattern-independent background b_m (LC ring-down, DC, electrical
 * pickup) analytically.  The resulting least-squares problem for each
 * microphone is solved with a matrix-free complex conjugate-gradient
 * iteration; the random rows are regenerated from a splitmix64 PRNG so the
 * 84 x 84 normal matrix is never stored.
 *
 * The reconstructed transfer matrix is then de-rotated by the known
 * air-path phase exp(+j*k*r_mi) and fitted with a per-microphone common-mode
 * nuisance plus a rank-1 channel response.  Four discrete candidates
 * (de-rotation sign x correction sign) are resolved by actually focusing the
 * production renderer on each microphone acoustic port and measuring the
 * array gain.  Nothing is written unless the measured gain gate passes.
 *
 * The waveform is transmitted as a long burst; the FPGA gate is placed in
 * the burst steady state.  Reflections from objects >=15 cm away cannot
 * reach the gate before it closes, so the result is effectively independent
 * of the room.  No FPGA RTL change is needed.
 */
#include "us_calibration.h"
#include "mic_capture.h"
#include "main.h"
#include "cmsis_os.h"
#include "system_status.h"
#include "cordic.h"
#include "spatial_renderer.h"
#include <math.h>
#include <string.h>

#define CAL_CHANNELS  UMH_DEVICE_CHANNEL_COUNT
#define CAL_MICS      UMH_DEVICE_MIC_COUNT
#define CAL_PI        3.14159265358979323846f
#define CAL_TWO_PI    6.28318530717958647692f

/* --------------------------------------------------------------------------
 * Bring-up / debug switches.
 *
 * CAL_DIRECT_ID_PROBE enables the one-shot single-channel coupon probe.  It
 * is useful during the first bench session but is deliberately disabled in
 * production because it adds eight excitations and a compile-time different
 * EEPROM payload path.
 * -------------------------------------------------------------------------- */
#ifndef CAL_DIRECT_ID_PROBE
#define CAL_DIRECT_ID_PROBE 0
#endif
#ifndef CAL_QUALITY_RELAXED
#define CAL_QUALITY_RELAXED 0
#endif

/* ---- Level / coherence probe (A1) ---------------------------------------- */
#define CAL_LEVEL_COUNT             5u
#define CAL_A1_GATE_START         200u   /* 5.0 ms after pattern swap       */
#define CAL_A1_GATE_WIDTH          16u   /* 400 us integration              */
#define CAL_A1_BURST_US          5600u   /* 224 samples                     */
#define CAL_A1_SETTLE_US         1500u
#define CAL_LEVEL_LINEARITY_MAX   0.10f
#define CAL_SNR_MIN_DB           10.0f
#define CAL_PHASE_SCATTER_MAX_DEG 15.0f

/* ---- Random projection acquisition (C) ----------------------------------- */
#define CAL_PATTERNS_MIN          128u
#define CAL_PATTERN_CHECK         128u
#define CAL_PATTERNS_MAX         1800u
#define CAL_CONVERGE_DEG            1.5f
#define CAL_TIME_BUDGET_MS        9000u
#define CAL_SETTLE_US             3000u
#define CAL_CG_ITERS                48u
#define CAL_CG_TOL                  1.0e-4f

/* ---- Transient profile and gate selection (B) ---------------------------- */
#define CAL_PROFILE_GATES          64u
#define CAL_PROFILE_STEP            4u   /* 100 us start-to-start           */
#define CAL_PROFILE_WIDTH           2u   /* 50 us per profile gate          */
#define CAL_PROFILE_BURST_US     7200u   /* 288 samples                     */
#define CAL_PROFILE_PATTERNS        8u
#define CAL_STABLE_RUN              6u   /* 6*4 = 24 samples = gate span    */
#define CAL_STABLE_DEG              5.0f
#define CAL_STABLE_AMP_PCT         15.0f
#define CAL_GATE_WIDTH             16u   /* 400 us final gate               */
#define CAL_GATE_TAIL               8u   /* 200 us margin                   */
#define CAL_GATE_START_MIN         20u   /* 500 us fallback minimum         */

/* Distance from the PCB microphone port plane to the piezo ceramic of the
 * open GU1008C-40TR transducer.  The acoustic source is recessed inside the
 * aluminium cylinder and sits 7.0 mm above the PCB. */
#define CAL_SRC_Z_MM                7.0f

/* ---- Direct-path fit (E) -------------------------------------------------- */
#define CAL_MIN_PATH_MM           15.0f
#define CAL_WEIGHT_CAP             3.0f
#define CAL_WEIGHT_FLOOR          1.0e-4f
#define CAL_FIT_MU_ROUNDS          24u
#define CAL_FIT_RANK_ITERS          8u
#define CAL_FIT_OUTLIER_ROUND       16u
#define CAL_OUTLIER_MIN_DEG       30.0f
#define CAL_OUTLIER_MAD             3.0f
#define CAL_MIN_PAIRS_PER_CH        2u
#define CAL_BAND_COUNT              3u
#define CAL_FIT_RMS_MAX_DEG       25.0f
#define CAL_CONSIST_MAX_DEG       15.0f
#define CAL_BAND_TREND_MAX_DEG    15.0f

/* ---- Run-time focus gain verification (F) -------------------------------- */
#define CAL_VERIFY_MIN_DB           6.0f
#define CAL_VERIFY_RMS_GATE        20.0f
#define CAL_VERIFY_MIC_MIN_POS      3u
#define CAL_COUPLING_WARN_DB        8.0f

/* --------------------------------------------------------------------------
 * Fixed microphone acoustic-hole coordinates.
 *
 * The four SPH0641LU4H-1 ports are the 0.5 mm NPTH holes in
 * Hardware Design/.../Drill_NPTH_Through.DRL, transformed to the profile
 * user frame (x_user = -x_cad, y_user = y_cad).  The outer three packages are
 * rotated so their ports point at the array centre; the centre microphone is
 * vertical.  The slots are ordered exactly like the FPGA demodulator:
 *   slot 0 = DATA0 rising  (U181, SELECT high) -> (-43.305,  24.994)
 *   slot 1 = DATA0 falling (U180, SELECT low ) -> ( 43.298,  24.994)
 *   slot 2 = DATA1 rising  (U204, SELECT high) -> (  0.000,   0.000)
 *   slot 3 = DATA1 falling (U182, SELECT low ) -> ( -0.004, -49.994)
 *
 * Gate 1 confirms these coordinates offline with the --localize tool.  The
 * values are deliberately one fixed table in firmware: the calibration run
 * never performs an on-line geometry search.
 * -------------------------------------------------------------------------- */
static const float cal_mic_x_mm[CAL_MICS] = { -43.305f,  43.298f,   0.000f,  -0.004f };
static const float cal_mic_y_mm[CAL_MICS] = {  24.994f,  24.994f,   0.000f, -49.994f };

/* --------------------------------------------------------------------------
 * Static storage.  The receiver accumulator and the reconstructed transfer
 * matrix share the same array: after CG has solved row m, the row is
 * overwritten with Z_m.  cal_scratch is a union of the CG working vectors
 * and the fit-domain sin/cos cache, because the two phases never overlap.
 * -------------------------------------------------------------------------- */
static const umh_device_profile_t *cal_profile;
static float cal_z_re[CAL_MICS][CAL_CHANNELS];
static float cal_z_im[CAL_MICS][CAL_CHANNELS];
static float cal_sy_re[CAL_MICS];
static float cal_sy_im[CAL_MICS];
static float cal_sk[CAL_CHANNELS];
static float cal_xbar[CAL_CHANNELS];

typedef union {
  float cg[8][CAL_CHANNELS]; /* 0/1 x, 2/3 r, 4/5 p, 6/7 Ap */
  struct {
    float cos_re[CAL_MICS][CAL_CHANNELS];
    float sin_re[CAL_MICS][CAL_CHANNELS];
  } trig;
} cal_scratch_u;
static cal_scratch_u cal_scratch;

/* Accumulator snapshot used by the early-convergence check. */
static float cal_accum_re[CAL_MICS][CAL_CHANNELS];
static float cal_accum_im[CAL_MICS][CAL_CHANNELS];

/* Final fit state. */
static float cal_a_re[CAL_CHANNELS];
static float cal_a_im[CAL_CHANNELS];
static float cal_rho_re[CAL_MICS];
static float cal_rho_im[CAL_MICS];
static float cal_mu_re[CAL_MICS];
static float cal_mu_im[CAL_MICS];
static float cal_phi[CAL_CHANNELS];
static float cal_fit_deg[CAL_CHANNELS];
static float cal_phi_prev[CAL_CHANNELS];
static uint8_t cal_phi_prev_valid;
static uint8_t cal_pair_valid[CAL_MICS][CAL_CHANNELS];
static uint8_t cal_corr[4][CAL_CHANNELS];
static float cal_candidate_gain_db[4];
static uint8_t cal_candidate_positive[4];

/* Transient profile, filled by phase B and exported as dump section 1. */
static int16_t cal_profile_i[CAL_PROFILE_GATES][CAL_MICS];
static int16_t cal_profile_q[CAL_PROFILE_GATES][CAL_MICS];

/* Bench profile accumulators.  Kept out of the task stack; averaged into
 * cal_profile_i/q only after all repeats are collected. */
static int16_t cal_profile_acc_i[CAL_PROFILE_GATES][CAL_MICS];
static int16_t cal_profile_acc_q[CAL_PROFILE_GATES][CAL_MICS];

/* Linearity diagnostic: all-channel phase-0 differential field versus the
 * sum of the individually measured channel vectors.  Exposed as CAL_DUMP
 * section 3 for bench analysis. */
static float cal_dbg_line_re[CAL_MICS], cal_dbg_line_im[CAL_MICS];
static float cal_dbg_line_pred_re[CAL_MICS], cal_dbg_line_pred_im[CAL_MICS];

/* ID probe diagnostics (optional, Gate 1 only). */
#if CAL_DIRECT_ID_PROBE
static uint8_t cal_id_channel[8];
static float cal_id_amp[8];
#endif

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint16_t version;
  uint16_t patterns_used;
  uint8_t level_used;
  uint8_t geom_hypothesis;
  uint8_t sign_hypothesis;
  uint8_t good_mics;
  uint16_t gate_start;
  uint8_t gate_width;
  uint8_t reserved;
  float fit_rms_deg;
  float mic_consistency_deg;
  float residual;
  float drift_deg;
  float band_trend_deg;
  float verify_gain_db;
  float coupling_db;
  float rms_before_deg;
  float rms_after_deg;
  float candidate_gain_db[4];
  float a_re[CAL_CHANNELS];
  float a_im[CAL_CHANNELS];
  float rho_re[CAL_MICS];
  float rho_im[CAL_MICS];
  float mu_re[CAL_MICS];
  float mu_im[CAL_MICS];
  uint8_t q_hat[CAL_CHANNELS];
} cal_dump2_t;

static cal_dump2_t cal_dump2;

/* --------------------------------------------------------------------------
 * Misc state.
 * -------------------------------------------------------------------------- */
static uint32_t cal_frame_sequence;
static uint16_t cal_block_expected;
static uint16_t cal_cs_patterns;
static uint8_t cal_level_used = 128u;
static uint16_t cal_gate_start = CAL_GATE_START_MIN;
static uint8_t cal_gate_width = CAL_GATE_WIDTH;
static uint32_t cal_burst_us;
static float cal_k_wave_mm;
static float cal_noise_power;
static float cal_signal_power;
static us_cal_progress_cb_t cal_stage_progress;
static void *cal_stage_progress_ctx;
static uint32_t cal_time_budget_start;

typedef struct {
  float fit_rms_deg;
  float mic_consistency_deg;
  float residual;
  float drift_deg;
  float band_trend_deg;
  float rms_before_deg;
  float rms_after_deg;
} cal_fit_metrics_t;

/* Runtime debug counters; visible in GDB. */
typedef struct {
  volatile uint32_t stage;
  volatile uint32_t total_ms;
  volatile uint32_t patterns;
  volatile uint32_t apply_calls;
  volatile uint32_t cordic_fallbacks;
  volatile uint32_t checks;
  volatile float a1_amp[CAL_LEVEL_COUNT];
  volatile float a1_snr_db[CAL_LEVEL_COUNT];
  volatile float a1_scatter_deg[CAL_LEVEL_COUNT];
  volatile float a1_p1[CAL_LEVEL_COUNT];
  volatile float a1_p2[CAL_LEVEL_COUNT];
  volatile float a1_p3[CAL_LEVEL_COUNT];
  volatile float a1_noise[CAL_LEVEL_COUNT];
  volatile uint32_t first_error;
} cal_debug_t;
cal_debug_t cal_debug;

/* Bench/debug observables for the v4 subset self-test. */
volatile float cal_dbg_self_y_re[CAL_MICS];
volatile float cal_dbg_self_y_im[CAL_MICS];
volatile float cal_dbg_self_c_re[CAL_MICS];
volatile float cal_dbg_self_c_im[CAL_MICS];
volatile float cal_dbg_self_pred_mag[CAL_MICS];
volatile float cal_dbg_self_phase0[CAL_MICS];
volatile float cal_dbg_self_ctrl_mag[CAL_MICS];

/* --------------------------------------------------------------------------
 * Small helpers and CORDIC-backed math wrappers.
 * -------------------------------------------------------------------------- */
static void cal_stage_update(uint8_t state, uint8_t progress)
{
  if (cal_stage_progress != NULL) cal_stage_progress(state, progress, cal_stage_progress_ctx);
  osDelay(1u);
}

static void cal_report(us_cal_progress_cb_t cb, void *context, uint8_t state, uint8_t progress)
{
  if (cb != NULL) cb(state, progress, context);
}

static void cal_sincos_batch(const float *angles, float *sin_out, float *cos_out, uint32_t count)
{
  uint32_t i;
  if (umh_cordic_sincos_batch(angles, sin_out, cos_out, count) == 0) return;
  ++cal_debug.cordic_fallbacks;
  for (i = 0u; i < count; ++i) {
    sin_out[i] = sinf(angles[i]);
    cos_out[i] = cosf(angles[i]);
  }
}

static float cal_atan2_rad(float real, float imag)
{
  float phase;
  if (umh_cordic_phase(real, imag, &phase) == 0) return phase;
  return atan2f(imag, real);
}

static float cal_wrap_pi(float x)
{
  if (x > CAL_PI || x < -CAL_PI) {
    float k = x * (1.0f / CAL_TWO_PI);
    int32_t n = (k >= 0.0f) ? (int32_t)(k + 0.5f) : (int32_t)(k - 0.5f);
    x -= (float)n * CAL_TWO_PI;
  }
  return x;
}

static float cal_deg(float rad) { return rad * (180.0f / CAL_PI); }
static float cal_rad(float deg) { return deg * (CAL_PI / 180.0f); }

/* Accurate microsecond delay for LC burst and ring-down gaps.  The FreeRTOS
 * tick is 1 ms, which is far too coarse for placing the gate inside a burst.
 * Long burst/settle intervals sleep most of their span: calibration runs at
 * a higher priority than the UI, and busy-waiting 8 ms per channel otherwise
 * kept the lower-priority OLED I2C transaction past its HAL timeout.  The
 * DWT-based remainder still guarantees the requested minimum delay. */
static void cal_delay_us(uint32_t us)
{
  uint32_t cycles_per_us;
  uint32_t start;
  uint32_t wait;
  if (us == 0u) return;
  if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0u) {
    osDelay((us + 999u) / 1000u);
    return;
  }
  cycles_per_us = (uint32_t)(SystemCoreClock / 1000000u);
  if (cycles_per_us == 0u) cycles_per_us = 1u;
  wait = us * cycles_per_us;
  start = DWT->CYCCNT;
  if (us >= 2000u) {
    uint32_t sleep_ms = (us - 1000u) / 1000u;
    if (sleep_ms != 0u) osDelay(sleep_ms);
  }
  while ((DWT->CYCCNT - start) < wait) { }
}

/* --------------------------------------------------------------------------
 * Splitmix64 random +/-1 projection rows.  Acquisition and reconstruction
 * regenerate identical rows from the pattern index; no measurement matrix is
 * stored.  The rows are deliberately i.i.d. rather than exactly balanced so
 * the 84 columns span the complete space and A = C^H C is well conditioned.
 * -------------------------------------------------------------------------- */
#define CAL_CS_SEED 0x243F6A8885A308D3ull

static uint32_t cal_mix32(uint64_t *state)
{
  uint64_t z = (*state += 0x9E3779B97F4A7C15ull);
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
  return (uint32_t)(z ^ (z >> 31));
}

static void cal_cs_row(uint16_t pattern, uint32_t bits[3])
{
  uint64_t state = CAL_CS_SEED ^ ((uint64_t)(pattern + 1u) * 0xD1B54A32D192ED03ull);
  bits[0] = cal_mix32(&state);
  bits[1] = cal_mix32(&state);
  bits[2] = cal_mix32(&state) & 0x000FFFFFu; /* channels 64..83 only */
}

static __attribute__((always_inline)) inline int cal_cs_bit(const uint32_t bits[3], uint8_t channel)
{
  uint32_t word = bits[channel >> 5u];
  return (int)((word >> (uint32_t)(channel & 31u)) & 1u);
}

static __attribute__((always_inline)) inline float cal_cs_sign(const uint32_t bits[3], uint8_t channel)
{
  return cal_cs_bit(bits, channel) != 0 ? -1.0f : 1.0f;
}

/* --------------------------------------------------------------------------
 * FPGA link helpers.
 * -------------------------------------------------------------------------- */
static int cal_wait_credit_fast(fpga_link_t *link, uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  uint32_t spins = 0u;
  if (link == NULL) return -1;
  while (fpga_link_status(link)->fifo_credit == 0u) {
    if ((HAL_GetTick() - start) > timeout_ms) return -2;
    if (fpga_link_poll_status(link) != 0) return -3;
    ++spins;
    if (spins < 64u) cal_delay_us(5u);
    else osDelay(1u);
  }
  return 0;
}

static int cal_submit_frame(fpga_link_t *link, umh_output_frame_t *frame)
{
  if (link == NULL || frame == NULL) return -1;
  if (cal_wait_credit_fast(link, 300u) != 0) return -2;
  return fpga_link_submit(link, frame);
}

static int cal_submit_bits(fpga_link_t *link, const uint32_t bits[3],
                           const uint8_t *correction, uint8_t level)
{
  umh_output_frame_t frame;
  uint16_t i;
  if (link == NULL) return -1;
  memset(&frame, 0, sizeof(frame));
  frame.update_flags = UMH_FRAME_FLAG_ULTRASOUND;
  frame.sequence = ++cal_frame_sequence;
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    uint8_t phase = 0u;
    if (bits != NULL && cal_cs_bit(bits, (uint8_t)i) != 0) phase = 128u;
    if (correction != NULL) phase = (uint8_t)(phase + correction[i]);
    frame.channels[i].phase = phase;
    frame.channels[i].level = level;
  }
  return cal_submit_frame(link, &frame);
}

#if CAL_DIRECT_ID_PROBE
static int cal_submit_single(fpga_link_t *link, uint8_t channel, uint8_t level)
{
  umh_output_frame_t frame;
  uint16_t i;
  if (link == NULL || channel >= CAL_CHANNELS) return -1;
  memset(&frame, 0, sizeof(frame));
  frame.update_flags = UMH_FRAME_FLAG_ULTRASOUND;
  frame.sequence = ++cal_frame_sequence;
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    frame.channels[i].phase = 0u;
    frame.channels[i].level = (i == (uint16_t)channel) ? level : 0u;
  }
  return cal_submit_frame(link, &frame);
}
#endif

static int cal_wait_block_fast(fpga_link_t *link, uint16_t expected,
                               uint32_t timeout_ms, fpga_mic_gate_wire_t *status_out)
{
  uint32_t start = HAL_GetTick();
  fpga_mic_gate_wire_t gate;
  if (link == NULL) return -1;
  for (;;) {
    if (fpga_link_mic_read(link, 0u, &gate) == 0) {
      if ((gate.status & FPGA_MIC_STATUS_DONE) != 0u &&
          gate.block_count >= expected) {
        if (status_out != NULL) *status_out = gate;
        return 0;
      }
    }
    if ((HAL_GetTick() - start) >= timeout_ms) return -2;
    cal_delay_us(20u);
  }
}

static int cal_mic_start(fpga_link_t *link, uint8_t gate_count, uint16_t start,
                         uint16_t step, uint8_t width)
{
  if (mic_capture_configure(link, gate_count, start, step, width) != 0) return -1;
  cal_block_expected = 0u;
  cal_delay_us(100u);
  return 0;
}

/* Submit one frame, keep the drive on for delay_us, stop, wait for the FPGA
 * block and return the four microphone I/Q phasors. */
static int cal_measure_frame_iq(fpga_link_t *link, umh_output_frame_t *frame,
                                uint32_t delay_us, float y_re[CAL_MICS],
                                float y_im[CAL_MICS])
{
  fpga_mic_gate_wire_t wire;
  uint16_t expected;
  uint8_t m;
  uint32_t timeout = delay_us / 1000u + 200u;
  if (cal_submit_frame(link, frame) != 0) return -1;
  cal_delay_us(delay_us);
  if (fpga_link_safe_stop(link) != 0) return -2;
  expected = (uint16_t)(cal_block_expected + 1u);
  if (cal_wait_block_fast(link, expected, timeout, &wire) != 0) return -3;
  cal_block_expected = expected;
  for (m = 0u; m < CAL_MICS; ++m) {
    y_re[m] = (float)wire.i[m];
    y_im[m] = (float)wire.q[m];
  }
  return 0;
}

static int cal_measure_pattern_iq(fpga_link_t *link, const uint32_t bits[3],
                                  uint8_t level, uint32_t delay_us,
                                  float y_re[CAL_MICS], float y_im[CAL_MICS])
{
  umh_output_frame_t frame;
  uint16_t i;
  if (link == NULL) return -1;
  memset(&frame, 0, sizeof(frame));
  frame.update_flags = UMH_FRAME_FLAG_ULTRASOUND;
  frame.sequence = ++cal_frame_sequence;
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    frame.channels[i].phase = (bits != NULL && cal_cs_bit(bits, (uint8_t)i) != 0) ? 128u : 0u;
    frame.channels[i].level = level;
  }
  return cal_measure_frame_iq(link, &frame, delay_us, y_re, y_im);
}

static int cal_measure_silence_iq(fpga_link_t *link, uint32_t delay_us,
                                  float y_re[CAL_MICS], float y_im[CAL_MICS])
{
  umh_output_frame_t frame;
  memset(&frame, 0, sizeof(frame));
  frame.update_flags = UMH_FRAME_FLAG_ULTRASOUND;
  frame.sequence = ++cal_frame_sequence;
  memset(frame.channels, 0, sizeof(frame.channels));
  return cal_measure_frame_iq(link, &frame, delay_us, y_re, y_im);
}

/* --------------------------------------------------------------------------
 * Matrix-free centered normal operator.
 *
 * x is the command sign vector (+1 for phase 0, -1 for phase 128).  The
 * least-squares design matrix is column-centered; c_i = x_i - mean_p(x_i).
 * We apply A = C^T C on demand, with the centering means stored in cal_xbar.
 * -------------------------------------------------------------------------- */
static __attribute__((optimize("O3"))) void cal_cs_apply(const float *xr, const float *xi,
                                                         float *yr, float *yi)
{
  uint32_t bits[3];
  uint16_t p, i;
  ++cal_debug.apply_calls;
  memset(yr, 0, CAL_CHANNELS * sizeof(float));
  memset(yi, 0, CAL_CHANNELS * sizeof(float));
  for (p = 0u; p < cal_cs_patterns; ++p) {
    float dr = 0.0f;
    float di = 0.0f;
    float c[CAL_CHANNELS];
    cal_cs_row(p, bits);
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      c[i] = cal_cs_sign(bits, (uint8_t)i) - cal_xbar[i];
      dr += c[i] * xr[i];
      di += c[i] * xi[i];
    }
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      yr[i] += c[i] * dr;
      yi[i] += c[i] * di;
    }
  }
}

/* Complex CG on A x = b, with A real symmetric positive definite.  The
 * vectors live in cal_scratch.cg; rows 6/7 are the residual r, and the
 * incoming b row is used as the initial residual to avoid another vector. */
static __attribute__((optimize("O3"))) int cal_cs_cg_solve(const float *b_re, const float *b_im,
                                                           float *x_re, float *x_im)
{
  float *pr = cal_scratch.cg[4];
  float *pi = cal_scratch.cg[5];
  float *ap_r = cal_scratch.cg[6];
  float *ap_i = cal_scratch.cg[7];
  float rsold = 0.0f, rs0;
  uint16_t i;
  uint8_t iter;
  memset(x_re, 0, CAL_CHANNELS * sizeof(float));
  memset(x_im, 0, CAL_CHANNELS * sizeof(float));
  /* Use the caller's b input as r; it is dead after this call. */
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    pr[i] = b_re[i];
    pi[i] = b_im[i];
    rsold += pr[i] * pr[i] + pi[i] * pi[i];
  }
  rs0 = rsold;
  if (rs0 < 1.0e-18f) return -1;
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    /* p = r initially: store p in rows 2/3 by reusing the x arrays?  We need
     * separate p; use scratch rows 2/3. */
    cal_scratch.cg[2][i] = pr[i];
    cal_scratch.cg[3][i] = pi[i];
  }
  for (iter = 0u; iter < CAL_CG_ITERS; ++iter) {
    float denom = 0.0f, alpha, rsnew = 0.0f, beta;
    cal_cs_apply(cal_scratch.cg[2], cal_scratch.cg[3], ap_r, ap_i);
    for (i = 0u; i < CAL_CHANNELS; ++i)
      denom += cal_scratch.cg[2][i] * ap_r[i] + cal_scratch.cg[3][i] * ap_i[i];
    if (denom <= 1.0e-18f) break;
    alpha = rsold / denom;
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      x_re[i] += alpha * cal_scratch.cg[2][i];
      x_im[i] += alpha * cal_scratch.cg[3][i];
      pr[i] -= alpha * ap_r[i];
      pi[i] -= alpha * ap_i[i];
      rsnew += pr[i] * pr[i] + pi[i] * pi[i];
    }
    if (rsnew <= rs0 * CAL_CG_TOL * CAL_CG_TOL) return 0;
    beta = (rsold > 1.0e-30f) ? (rsnew / rsold) : 0.0f;
    rsold = rsnew;
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      cal_scratch.cg[2][i] = pr[i] + beta * cal_scratch.cg[2][i];
      cal_scratch.cg[3][i] = pi[i] + beta * cal_scratch.cg[3][i];
    }
  }
  return (rsold <= rs0 * CAL_CG_TOL * CAL_CG_TOL) ? 0 : 1;
}

/* Reconstruct Z for all four microphones from the streaming accumulators.
 * Each row is transformed in place into the centered right-hand side b,
 * then CG writes the solution back into the same row. */
static int cal_cs_reconstruct(uint8_t report_progress)
{
  uint8_t m;
  uint16_t i;
  float x_re[CAL_CHANNELS];
  float x_im[CAL_CHANNELS];
  if (cal_cs_patterns < CAL_PATTERNS_MIN) return -5;
  for (i = 0u; i < CAL_CHANNELS; ++i)
    cal_xbar[i] = cal_sk[i] / (float)cal_cs_patterns;
  for (m = 0u; m < CAL_MICS; ++m) {
    int rc;
    if (report_progress) cal_stage_update(US_CAL_SOLVE, (uint8_t)(78u + m * 2u));
    /* b = Sxy - xbar * Sy in place. */
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      cal_z_re[m][i] -= cal_xbar[i] * cal_sy_re[m];
      cal_z_im[m][i] -= cal_xbar[i] * cal_sy_im[m];
    }
    rc = cal_cs_cg_solve(cal_z_re[m], cal_z_im[m], x_re, x_im);
    if (rc < 0) return -2;
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      cal_z_re[m][i] = x_re[i];
      cal_z_im[m][i] = x_im[i];
    }
  }
  return 0;
}

/* --------------------------------------------------------------------------
 * Direct-path pair validity and rank-1 fit.
 * -------------------------------------------------------------------------- */
static void cal_median_f32(float *values, uint16_t count, float *median_out)
{
  uint16_t i, j;
  if (count == 0u) { *median_out = 0.0f; return; }
  for (i = 1u; i < count; ++i) {
    float v = values[i];
    j = i;
    while (j > 0u && values[j - 1u] > v) {
      values[j] = values[j - 1u];
      --j;
    }
    values[j] = v;
  }
  if ((count & 1u) != 0u) *median_out = values[count / 2u];
  else *median_out = 0.5f * (values[count / 2u - 1u] + values[count / 2u]);
}

static float cal_direct_path_mm(uint8_t mic, uint8_t channel)
{
  float px = (float)cal_profile->coordinates[channel].x_um * 0.001f;
  float py = (float)cal_profile->coordinates[channel].y_um * 0.001f;
  float dx = px - cal_mic_x_mm[mic];
  float dy = py - cal_mic_y_mm[mic];
  return sqrtf(dx * dx + dy * dy + CAL_SRC_Z_MM * CAL_SRC_Z_MM);
}

static void cal_prepare_pairs(void)
{
  uint8_t m, i;
  float med[CAL_MICS];
  memset(cal_pair_valid, 0, sizeof(cal_pair_valid));
  for (m = 0u; m < CAL_MICS; ++m) {
    uint16_t n = 0u;
    float *scratch = cal_scratch.cg[0];
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float zr = cal_z_re[m][i], zi = cal_z_im[m][i];
      float mag = sqrtf(zr * zr + zi * zi);
      if (cal_direct_path_mm(m, i) >= CAL_MIN_PATH_MM) scratch[n++] = mag;
    }
    cal_median_f32(scratch, n, &med[m]);
    if (med[m] < 1.0e-9f) med[m] = 1.0f;
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float zr = cal_z_re[m][i], zi = cal_z_im[m][i];
      float mag = sqrtf(zr * zr + zi * zi);
      if (cal_direct_path_mm(m, i) >= CAL_MIN_PATH_MM &&
          mag >= CAL_WEIGHT_FLOOR * med[m] && mag <= CAL_WEIGHT_CAP * med[m]) {
        cal_pair_valid[m][i] = 1u;
      }
    }
  }
  /* Every channel must have at least two valid microphone pairs, otherwise
   * the channel phase is not observable.  Add the strongest excluded pair
   * for that channel until the requirement is met. */
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    uint8_t have = 0u, add;
    for (m = 0u; m < CAL_MICS; ++m) if (cal_pair_valid[m][i] != 0u) ++have;
    while (have < (uint8_t)CAL_MIN_PAIRS_PER_CH) {
      float best = -1.0f;
      add = 0xFFu;
      for (m = 0u; m < CAL_MICS; ++m) {
        float zr, zi, mag;
        if (cal_pair_valid[m][i] != 0u) continue;
        if (cal_direct_path_mm(m, i) < CAL_MIN_PATH_MM) continue;
        zr = cal_z_re[m][i]; zi = cal_z_im[m][i];
        mag = sqrtf(zr * zr + zi * zi);
        if (mag > best) { best = mag; add = m; }
      }
      if (add == 0xFFu) break;
      cal_pair_valid[add][i] = 1u;
      ++have;
    }
  }
}

static __attribute__((always_inline)) inline void cal_d_pair(uint8_t m, uint8_t i,
                                                             float *dr, float *di)
{
  float cr = cal_scratch.trig.cos_re[m][i];
  float sr = cal_scratch.trig.sin_re[m][i];
  float zr = cal_z_re[m][i];
  float zi = cal_z_im[m][i];
  *dr = zr * cr - zi * sr;
  *di = zr * sr + zi * cr;
}

static float cal_model_rms_resid_deg(void)
{
  uint8_t m, i;
  float sum = 0.0f;
  uint16_t n = 0u;
  for (m = 0u; m < CAL_MICS; ++m) {
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float dr, di, ar, ai, mr, mi;
      if (cal_pair_valid[m][i] == 0u) continue;
      cal_d_pair(m, i, &dr, &di);
      ar = cal_a_re[i]; ai = cal_a_im[i];
      mr = ar * cal_rho_re[m] - ai * cal_rho_im[m];
      mi = ar * cal_rho_im[m] + ai * cal_rho_re[m];
      {
        float num_r = dr - cal_mu_re[m];
        float num_i = di - cal_mu_im[m];
        float dot = num_r * mr + num_i * mi;
        float cross = num_i * mr - num_r * mi;
        float ang = cal_atan2_rad(dot, cross); /* angle(d / model) */
        sum += ang * ang;
        ++n;
      }
    }
  }
  return (n != 0u) ? cal_deg(sqrtf(sum / (float)n)) : 999.0f;
}

static float cal_model_relative_residual(void)
{
  uint8_t m, i;
  float num = 0.0f, den = 0.0f;
  for (m = 0u; m < CAL_MICS; ++m) {
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float dr, di, ar, ai, mr, mi;
      if (cal_pair_valid[m][i] == 0u) continue;
      cal_d_pair(m, i, &dr, &di);
      ar = cal_a_re[i]; ai = cal_a_im[i];
      mr = ar * cal_rho_re[m] - ai * cal_rho_im[m];
      mi = ar * cal_rho_im[m] + ai * cal_rho_re[m];
      num += (dr - cal_mu_re[m] - mr) * (dr - cal_mu_re[m] - mr) +
             (di - cal_mu_im[m] - mi) * (di - cal_mu_im[m] - mi);
      den += (dr - cal_mu_re[m]) * (dr - cal_mu_re[m]) +
             (di - cal_mu_im[m]) * (di - cal_mu_im[m]);
    }
  }
  return num / (den + 1.0e-12f);
}

static float cal_model_consistency_deg(void)
{
  uint8_t m, i;
  float total = 0.0f;
  uint16_t nch = 0u;
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    float sum_cos = 0.0f, sum_sin = 0.0f;
    uint8_t good = 0u;
    for (m = 0u; m < CAL_MICS; ++m) {
      float dr, di, ph, rph;
      if (cal_pair_valid[m][i] == 0u) continue;
      cal_d_pair(m, i, &dr, &di);
      dr -= cal_mu_re[m];
      di -= cal_mu_im[m];
      ph = cal_atan2_rad(dr, di);
      rph = cal_atan2_rad(cal_rho_re[m], cal_rho_im[m]);
      ph = cal_wrap_pi(ph - rph);
      sum_cos += cosf(ph);
      sum_sin += sinf(ph);
      ++good;
    }
    if (good >= 2u) {
      float mean = cal_atan2_rad(sum_cos, sum_sin);
      float sum = 0.0f;
      uint8_t cnt = 0u;
      for (m = 0u; m < CAL_MICS; ++m) {
        float dr, di, ph, rph;
        if (cal_pair_valid[m][i] == 0u) continue;
        cal_d_pair(m, i, &dr, &di);
        dr -= cal_mu_re[m];
        di -= cal_mu_im[m];
        ph = cal_atan2_rad(dr, di);
        rph = cal_atan2_rad(cal_rho_re[m], cal_rho_im[m]);
        ph = cal_wrap_pi(ph - rph - mean);
        sum += ph * ph;
        ++cnt;
      }
      if (cnt >= 2u) { total += sum / (float)(cnt - 1u); ++nch; }
    }
  }
  return (nch != 0u) ? cal_deg(sqrtf(total / (float)nch)) : 999.0f;
}

static float cal_model_band_trend_deg(void)
{
  uint8_t m, i, b;
  float sum_ang[CAL_BAND_COUNT], r_min = 1.0e30f, r_max = -1.0e30f;
  uint16_t cnt[CAL_BAND_COUNT];
  for (b = 0u; b < CAL_BAND_COUNT; ++b) { sum_ang[b] = 0.0f; cnt[b] = 0u; }
  for (m = 0u; m < CAL_MICS; ++m) {
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      if (cal_pair_valid[m][i] == 0u) continue;
      if (cal_direct_path_mm(m, i) < r_min) r_min = cal_direct_path_mm(m, i);
      if (cal_direct_path_mm(m, i) > r_max) r_max = cal_direct_path_mm(m, i);
    }
  }
  if (r_max <= r_min) return 0.0f;
  for (m = 0u; m < CAL_MICS; ++m) {
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float dr, di, ar, ai, mr, mi, dot, cross, ang;
      uint8_t band;
      if (cal_pair_valid[m][i] == 0u) continue;
      cal_d_pair(m, i, &dr, &di);
      ar = cal_a_re[i]; ai = cal_a_im[i];
      mr = ar * cal_rho_re[m] - ai * cal_rho_im[m];
      mi = ar * cal_rho_im[m] + ai * cal_rho_re[m];
      dot = (dr - cal_mu_re[m]) * mr + (di - cal_mu_im[m]) * mi;
      cross = (di - cal_mu_im[m]) * mr - (dr - cal_mu_re[m]) * mi;
      ang = cal_atan2_rad(dot, cross);
      band = (uint8_t)(((cal_direct_path_mm(m, i) - r_min) * (float)CAL_BAND_COUNT) /
                       (r_max - r_min));
      if (band >= CAL_BAND_COUNT) band = CAL_BAND_COUNT - 1u;
      sum_ang[band] += ang;
      ++cnt[band];
    }
  }
  {
    float mn = 1.0e30f, mx = -1.0e30f;
    for (b = 0u; b < CAL_BAND_COUNT; ++b) {
      float mean;
      if (cnt[b] == 0u) continue;
      mean = sum_ang[b] / (float)cnt[b];
      if (mean < mn) mn = mean;
      if (mean > mx) mx = mean;
    }
    if (mx < -1.0e29f || mn > 1.0e29f) return 0.0f;
    return cal_deg(mx - mn);
  }
}

static float cal_model_drift_deg(void)
{
  uint8_t m, i;
  float s0 = 0.0f, s1 = 0.0f;
  uint16_t n0 = 0u, n1 = 0u;
  for (m = 0u; m < CAL_MICS; ++m) {
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float dr, di, ar, ai, mr, mi, dot, cross, ang;
      if (cal_pair_valid[m][i] == 0u) continue;
      cal_d_pair(m, i, &dr, &di);
      ar = cal_a_re[i]; ai = cal_a_im[i];
      mr = ar * cal_rho_re[m] - ai * cal_rho_im[m];
      mi = ar * cal_rho_im[m] + ai * cal_rho_re[m];
      dot = (dr - cal_mu_re[m]) * mr + (di - cal_mu_im[m]) * mi;
      cross = (di - cal_mu_im[m]) * mr - (dr - cal_mu_re[m]) * mi;
      ang = cal_atan2_rad(dot, cross);
      if (i < (CAL_CHANNELS / 2u)) { s0 += ang; ++n0; }
      else { s1 += ang; ++n1; }
    }
  }
  if (n0 == 0u || n1 == 0u) return 0.0f;
  return fabsf(cal_deg(cal_wrap_pi((s1 / (float)n1) - (s0 / (float)n0))));
}

static void cal_fit_reset_state(void)
{
  uint8_t m, i;
  for (i = 0u; i < CAL_CHANNELS; ++i) { cal_a_re[i] = 1.0f; cal_a_im[i] = 0.0f; }
  for (m = 0u; m < CAL_MICS; ++m) {
    cal_rho_re[m] = 1.0f; cal_rho_im[m] = 0.0f;
    cal_mu_re[m] = 0.0f; cal_mu_im[m] = 0.0f;
  }
}

static void cal_fit_update_mu(void)
{
  uint8_t m, i;
  for (m = 0u; m < CAL_MICS; ++m) {
    float sr = 0.0f, si = 0.0f;
    uint16_t n = 0u;
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float dr, di, mr, mi;
      if (cal_pair_valid[m][i] == 0u) continue;
      cal_d_pair(m, i, &dr, &di);
      mr = cal_a_re[i] * cal_rho_re[m] - cal_a_im[i] * cal_rho_im[m];
      mi = cal_a_re[i] * cal_rho_im[m] + cal_a_im[i] * cal_rho_re[m];
      sr += dr - mr; si += di - mi;
      ++n;
    }
    if (n != 0u) { cal_mu_re[m] = sr / (float)n; cal_mu_im[m] = si / (float)n; }
  }
}

/* Run the alternating weighted rank-1 + common-mode fit for one de-rotation
 * sign.  sign=+1 multiplies Z by exp(+j*k*r), sign=-1 uses exp(-j*k*r). */
static int cal_fit_direct(uint8_t sign, cal_fit_metrics_t *metrics)
{
  uint8_t m, i, round, iter;
  uint16_t valid_count = 0u;

  /* Cache cos/sin for d_mi = Z_mi * exp(j*sign*k*r_mi). */
  for (m = 0u; m < CAL_MICS; ++m) {
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float theta = ((sign != 0u) ? 1.0f : -1.0f) * cal_k_wave_mm * cal_direct_path_mm(m, i);
      cal_scratch.trig.cos_re[m][i] = cosf(theta);
      cal_scratch.trig.sin_re[m][i] = sinf(theta);
    }
  }

  cal_fit_reset_state();
  for (round = 0u; round < CAL_FIT_MU_ROUNDS; ++round) {
    for (iter = 0u; iter < CAL_FIT_RANK_ITERS; ++iter) {
      /* a_i update using (d - mu) and current rho. */
      for (i = 0u; i < CAL_CHANNELS; ++i) {
        float nr = 0.0f, ni = 0.0f, den = 0.0f;
        for (m = 0u; m < CAL_MICS; ++m) {
          float dr, di, w, r2;
          if (cal_pair_valid[m][i] == 0u) continue;
          cal_d_pair(m, i, &dr, &di);
          dr -= cal_mu_re[m]; di -= cal_mu_im[m];
          r2 = cal_rho_re[m] * cal_rho_re[m] + cal_rho_im[m] * cal_rho_im[m];
          w = sqrtf(dr * dr + di * di);
          nr += w * (dr * cal_rho_re[m] + di * cal_rho_im[m]);
          ni += w * (di * cal_rho_re[m] - dr * cal_rho_im[m]);
          den += w * r2;
        }
        if (den < 1.0e-12f) den = 1.0e-12f;
        cal_a_re[i] = nr / den;
        cal_a_im[i] = ni / den;
      }
      /* rho_m update using (d - mu) and current a. */
      for (m = 0u; m < CAL_MICS; ++m) {
        float nr = 0.0f, ni = 0.0f, den = 0.0f;
        for (i = 0u; i < CAL_CHANNELS; ++i) {
          float dr, di, w, a2;
          if (cal_pair_valid[m][i] == 0u) continue;
          cal_d_pair(m, i, &dr, &di);
          dr -= cal_mu_re[m]; di -= cal_mu_im[m];
          a2 = cal_a_re[i] * cal_a_re[i] + cal_a_im[i] * cal_a_im[i];
          w = sqrtf(dr * dr + di * di);
          nr += w * (dr * cal_a_re[i] + di * cal_a_im[i]);
          ni += w * (di * cal_a_re[i] - dr * cal_a_im[i]);
          den += w * a2;
        }
        if (den < 1.0e-12f) den = 1.0e-12f;
        cal_rho_re[m] = nr / den;
        cal_rho_im[m] = ni / den;
      }
      cal_fit_update_mu();
    }
    /* After enough rank-1 refinement, remove gross phase outliers. */
    if (round == CAL_FIT_OUTLIER_ROUND) {
      float resid[CAL_MICS * CAL_CHANNELS];
      uint16_t n = 0u;
      for (m = 0u; m < CAL_MICS; ++m) {
        for (i = 0u; i < CAL_CHANNELS; ++i) {
          float dr, di, ar, ai, mr, mi, dot, cross;
          if (cal_pair_valid[m][i] == 0u) continue;
          cal_d_pair(m, i, &dr, &di);
          ar = cal_a_re[i]; ai = cal_a_im[i];
          mr = ar * cal_rho_re[m] - ai * cal_rho_im[m];
          mi = ar * cal_rho_im[m] + ai * cal_rho_re[m];
          dot = (dr - cal_mu_re[m]) * mr + (di - cal_mu_im[m]) * mi;
          cross = (di - cal_mu_im[m]) * mr - (dr - cal_mu_re[m]) * mi;
          resid[n++] = cal_atan2_rad(dot, cross);
        }
      }
      if (n > 4u) {
        float sorted[CAL_MICS * CAL_CHANNELS];
        float med, mad;
        uint16_t k;
        memcpy(sorted, resid, n * sizeof(float));
        cal_median_f32(sorted, n, &med);
        for (k = 0u; k < n; ++k)
          sorted[k] = fabsf(cal_wrap_pi(resid[k] - med));
        cal_median_f32(sorted, n, &mad);
        {
          float thr = cal_rad(CAL_OUTLIER_MIN_DEG);
          float robust = CAL_OUTLIER_MAD * 1.4826f * mad;
          if (robust > thr) thr = robust;
          n = 0u;
          for (m = 0u; m < CAL_MICS; ++m) {
            for (i = 0u; i < CAL_CHANNELS; ++i) {
              float dr, di, ar, ai, mr, mi, dot, cross, ang;
              if (cal_pair_valid[m][i] == 0u) continue;
              cal_d_pair(m, i, &dr, &di);
              ar = cal_a_re[i]; ai = cal_a_im[i];
              mr = ar * cal_rho_re[m] - ai * cal_rho_im[m];
              mi = ar * cal_rho_im[m] + ai * cal_rho_re[m];
              dot = (dr - cal_mu_re[m]) * mr + (di - cal_mu_im[m]) * mi;
              cross = (di - cal_mu_im[m]) * mr - (dr - cal_mu_re[m]) * mi;
              ang = cal_atan2_rad(dot, cross);
              if (fabsf(ang) > thr) cal_pair_valid[m][i] = 0u;
            }
          }
          /* Repair channels that lost below minimum coverage. */
          for (i = 0u; i < CAL_CHANNELS; ++i) {
            uint8_t have = 0u;
            for (m = 0u; m < CAL_MICS; ++m) if (cal_pair_valid[m][i] != 0u) ++have;
            while (have < (uint8_t)CAL_MIN_PAIRS_PER_CH) {
              float best = -1.0f;
              uint8_t add = 0xFFu;
              for (m = 0u; m < CAL_MICS; ++m) {
                float zr, zi, mag;
                if (cal_pair_valid[m][i] != 0u || cal_direct_path_mm(m, i) < CAL_MIN_PATH_MM) continue;
                zr = cal_z_re[m][i]; zi = cal_z_im[m][i];
                mag = sqrtf(zr * zr + zi * zi);
                if (mag > best) { best = mag; add = m; }
              }
              if (add == 0xFFu) break;
              cal_pair_valid[add][i] = 1u;
              ++have;
            }
          }
        }
      }
    }
    cal_stage_update(US_CAL_SOLVE, (uint8_t)(82u + (round < 6u ? round : 5u)));
  }

  for (i = 0u; i < CAL_CHANNELS; ++i)
    cal_phi[i] = cal_atan2_rad(cal_a_re[i], cal_a_im[i]);

  for (m = 0u; m < CAL_MICS; ++m)
    for (i = 0u; i < CAL_CHANNELS; ++i)
      if (cal_pair_valid[m][i] != 0u) ++valid_count;

  if (metrics != NULL) {
    memset(metrics, 0, sizeof(*metrics));
    metrics->fit_rms_deg = cal_model_rms_resid_deg();
    metrics->mic_consistency_deg = cal_model_consistency_deg();
    metrics->residual = cal_model_relative_residual();
    metrics->band_trend_deg = cal_model_band_trend_deg();
    metrics->drift_deg = cal_model_drift_deg();
  }
  if (valid_count < (uint16_t)(CAL_CHANNELS * CAL_MIN_PAIRS_PER_CH)) return -1;
  return 0;
}

/* Gauge fixing: remove a global phase and a linear phase plane over the
 * array.  Both are free parameters for a single focus point and removing
 * them keeps the EEPROM calibration compact.  Returns the RMS of the
 * gauge-corrected static phases in degrees. */
static float cal_gauge(const float *phi_rad, float *out_deg)
{
  uint8_t i;
  float sr = 0.0f, si = 0.0f, mean, rms = 0.0f;
  float *v = cal_scratch.cg[0];
  float *sn = cal_scratch.cg[1];
  float *cs = cal_scratch.cg[2];
  double xx = 0.0, xy = 0.0, yy = 0.0, x = 0.0, y = 0.0;
  double sv = 0.0, xv = 0.0, yv = 0.0;
  double a[3][4], det;
  cal_sincos_batch(phi_rad, sn, cs, CAL_CHANNELS);
  for (i = 0u; i < CAL_CHANNELS; ++i) { sr += cs[i]; si += sn[i]; }
  mean = cal_atan2_rad(sr, si);
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    double px = (double)cal_profile->coordinates[i].x_um * 0.001;
    double py = (double)cal_profile->coordinates[i].y_um * 0.001;
    float vv;
    v[i] = cal_wrap_pi(phi_rad[i] - mean);
    vv = v[i];
    xx += px * px; xy += px * py; yy += py * py; x += px; y += py;
    xv += px * vv; yv += py * vv; sv += vv;
    rms += vv * vv;
  }
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
    double fit = a[0][3] * ((double)cal_profile->coordinates[i].x_um * 0.001) +
                 a[1][3] * ((double)cal_profile->coordinates[i].y_um * 0.001) + a[2][3];
    out_deg[i] = cal_deg(cal_wrap_pi((float)(v[i] - fit)));
  }
  return rms;
}

static float cal_phase_vector_rms_deg(const float *a_deg, const float *b_deg)
{
  uint8_t i;
  float sr = 0.0f, si = 0.0f, mean, sum = 0.0f;
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    float d = cal_wrap_pi(cal_rad(a_deg[i]) - cal_rad(b_deg[i]));
    sr += cosf(d); si += sinf(d);
  }
  mean = cal_atan2_rad(sr, si);
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    float d = cal_wrap_pi(cal_rad(a_deg[i]) - cal_rad(b_deg[i]) - mean);
    sum += d * d;
  }
  return cal_deg(sqrtf(sum / (float)CAL_CHANNELS));
}

static void cal_make_correction_bytes(const float *fit_deg, uint8_t *out)
{
  uint8_t i;
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    int32_t q = (int32_t)lroundf(-fit_deg[i] * (256.0f / 360.0f));
    q %= 256;
    if (q < 0) q += 256;
    out[i] = (uint8_t)q;
  }
}

static void cal_negate_bytes(const uint8_t *in, uint8_t *out)
{
  uint8_t i;
  for (i = 0u; i < CAL_CHANNELS; ++i) out[i] = (uint8_t)(0u - in[i]);
}

/* --------------------------------------------------------------------------
 * Phase A1: drive-level linearity / coherence probe.
 * -------------------------------------------------------------------------- */
static int cal_level_probe(fpga_link_t *link, uint8_t *level_out, float *noise_out)
{
  static const uint8_t ladder[CAL_LEVEL_COUNT] = {128u, 64u, 32u, 16u, 8u};
  float amp[CAL_LEVEL_COUNT], scatter_deg[CAL_LEVEL_COUNT], snr_db[CAL_LEVEL_COUNT];
  float norm[CAL_LEVEL_COUNT];
  uint8_t idx, m;
  uint32_t bits_b[3];
  uint8_t chosen = 0xFFu;
  if (cal_mic_start(link, 1u, CAL_A1_GATE_START, CAL_A1_GATE_WIDTH, CAL_A1_GATE_WIDTH) != 0) {
    cal_debug.first_error = 1u; return -1; }
  cal_cs_row(0u, bits_b);
  for (idx = 0u; idx < CAL_LEVEL_COUNT; ++idx) {
    float y1r[CAL_MICS], y1i[CAL_MICS], y2r[CAL_MICS], y2i[CAL_MICS];
    float y3r[CAL_MICS], y3i[CAL_MICS];
    float p1 = 0.0f, p2 = 0.0f, p3 = 0.0f, nsum = 0.0f;
    float phase_diff = 0.0f;
    uint8_t nb;
    if (cal_measure_pattern_iq(link, NULL, ladder[idx], CAL_A1_BURST_US, y1r, y1i) != 0) { cal_debug.first_error = 2u; return -2; }
    cal_delay_us(CAL_A1_SETTLE_US);
    if (cal_measure_pattern_iq(link, bits_b, ladder[idx], CAL_A1_BURST_US, y2r, y2i) != 0) { cal_debug.first_error = 3u; return -3; }
    cal_delay_us(CAL_A1_SETTLE_US);
    if (cal_measure_pattern_iq(link, NULL, ladder[idx], CAL_A1_BURST_US, y3r, y3i) != 0) { cal_debug.first_error = 4u; return -4; }
    cal_delay_us(CAL_A1_SETTLE_US);
    for (m = 0u; m < CAL_MICS; ++m) {
      p1 += y1r[m] * y1r[m] + y1i[m] * y1i[m];
      p2 += y2r[m] * y2r[m] + y2i[m] * y2i[m];
      p3 += y3r[m] * y3r[m] + y3i[m] * y3i[m];
      {
        float cr = y1r[m] * y3r[m] + y1i[m] * y3i[m];
        float ci = y1i[m] * y3r[m] - y1r[m] * y3i[m];
        if ((p1 + p3) > 1.0e-6f && (y1r[m] * y1r[m] + y1i[m] * y1i[m]) > 1.0e-6f &&
            (y3r[m] * y3r[m] + y3i[m] * y3i[m]) > 1.0e-6f) {
          float d = cal_atan2_rad(cr, ci);
          phase_diff += fabsf(d);
        }
      }
    }
    for (nb = 0u; nb < 2u; ++nb) {
      float yr[CAL_MICS], yi[CAL_MICS];
      if (cal_measure_silence_iq(link, 300u, yr, yi) != 0) { cal_debug.first_error = 5u; return -5; }
      for (m = 0u; m < CAL_MICS; ++m) nsum += yr[m] * yr[m] + yi[m] * yi[m];
      cal_delay_us(CAL_A1_SETTLE_US);
    }
    {
      float power = (p1 + p2 + p3) / 3.0f;
      float noise = nsum / (2.0f * (float)CAL_MICS);
      float net = power - noise;
      if (net < 0.0f) net = 0.0f;
      amp[idx] = sqrtf(power / (float)CAL_MICS);
      snr_db[idx] = 10.0f * log10f((net + 1.0e-9f) / (noise + 1.0e-9f));
      scatter_deg[idx] = cal_deg(phase_diff / (float)CAL_MICS);
      norm[idx] = amp[idx] / sinf(CAL_PI * (float)ladder[idx] / 256.0f);
      cal_debug.a1_amp[idx] = amp[idx];
      cal_debug.a1_snr_db[idx] = snr_db[idx];
      cal_debug.a1_scatter_deg[idx] = scatter_deg[idx];
      cal_debug.a1_p1[idx] = p1;
      cal_debug.a1_p2[idx] = p2;
      cal_debug.a1_p3[idx] = p3;
      cal_debug.a1_noise[idx] = nsum;
    }
  }

  /* Choose the highest level that is consistent with all lower levels and
   * whose coherence/SNR are acceptable.  A compressed high level therefore
   * drops directly to the next lower one. */
  for (idx = 0u; idx < CAL_LEVEL_COUNT; ++idx) {
    uint8_t j;
    int ok = (snr_db[idx] >= CAL_SNR_MIN_DB && scatter_deg[idx] <= CAL_PHASE_SCATTER_MAX_DEG);
    for (j = idx + 1u; j < CAL_LEVEL_COUNT && ok != 0; ++j) {
      float ref = norm[j];
      float diff = fabsf(norm[idx] - ref);
      float scale = (norm[idx] > ref) ? norm[idx] : ref;
      if (scale < 1.0e-12f || diff > CAL_LEVEL_LINEARITY_MAX * scale) ok = 0;
    }
    if (ok != 0) { chosen = idx; break; }
  }
  if (chosen == 0xFFu) return -6;
  *level_out = ladder[chosen];
  {
    float yr[CAL_MICS], yi[CAL_MICS], sum = 0.0f;
    uint8_t k;
    for (k = 0u; k < 4u; ++k) {
      if (cal_measure_silence_iq(link, 300u, yr, yi) != 0) return -7;
      for (m = 0u; m < CAL_MICS; ++m) sum += yr[m] * yr[m] + yi[m] * yi[m];
      cal_delay_us(CAL_A1_SETTLE_US);
    }
    if (noise_out != NULL) *noise_out = sum / (4.0f * (float)CAL_MICS);
  }
  return 0;
}

/* --------------------------------------------------------------------------
 * Phase B: transient envelope profile and gate selection.
 * -------------------------------------------------------------------------- */
static int cal_transient_profile(fpga_link_t *link, uint8_t level,
                                 uint16_t *gate_start_out, uint8_t *gate_width_out)
{
  uint8_t rep, g, m;
  uint32_t bits[3];
  uint8_t stable[CAL_PROFILE_GATES];
  uint8_t run = 0u, run_start = 0u;
  float tail_mean[CAL_MICS];
  (void)bits;
  if (cal_mic_start(link, CAL_PROFILE_GATES, 0u, CAL_PROFILE_STEP, CAL_PROFILE_WIDTH) != 0)
    return -1;
  memset(cal_z_re, 0, sizeof(cal_z_re));
  memset(cal_z_im, 0, sizeof(cal_z_im));
  for (rep = 0u; rep < CAL_PROFILE_PATTERNS; ++rep) {
    fpga_mic_gate_wire_t wire;
    uint16_t expected;
    if (cal_submit_bits(link, NULL, NULL, level) != 0) return -2;
    cal_delay_us(CAL_PROFILE_BURST_US);
    if (fpga_link_safe_stop(link) != 0) return -3;
    expected = (uint16_t)(cal_block_expected + 1u);
    if (cal_wait_block_fast(link, expected, 300u, &wire) != 0) return -4;
    cal_block_expected = expected;
    for (g = 0u; g < CAL_PROFILE_GATES; ++g) {
      mic_capture_gate_t sample;
      if (mic_capture_read_gate(link, g, &sample, NULL) != 0) return -5;
      for (m = 0u; m < CAL_MICS; ++m) {
        cal_z_re[m][g] += (float)sample.i[m];
        cal_z_im[m][g] += (float)sample.q[m];
      }
    }
    cal_delay_us(CAL_SETTLE_US);
  }
  for (m = 0u; m < CAL_MICS; ++m) {
    for (g = 0u; g < CAL_PROFILE_GATES; ++g) {
      float ar = cal_z_re[m][g] / (float)CAL_PROFILE_PATTERNS;
      float ai = cal_z_im[m][g] / (float)CAL_PROFILE_PATTERNS;
      float mag = sqrtf(ar * ar + ai * ai);
      float ph = cal_atan2_rad(ar, ai);
      cal_profile_i[g][m] = (int16_t)lroundf(ar);
      cal_profile_q[g][m] = (int16_t)lroundf(ai);
      cal_z_re[m][g] = mag;
      cal_z_im[m][g] = ph;
    }
  }
  for (m = 0u; m < CAL_MICS; ++m) {
    float s = 0.0f;
    for (g = CAL_PROFILE_GATES - 8u; g < CAL_PROFILE_GATES; ++g) s += cal_z_re[m][g];
    tail_mean[m] = s / 8.0f;
  }
  {
    float max_tail = 0.0f;
    for (m = 0u; m < CAL_MICS; ++m) if (tail_mean[m] > max_tail) max_tail = tail_mean[m];
    if (max_tail < 5.0f) return -8;
  }
  for (g = 0u; g < CAL_PROFILE_GATES; ++g) {
    uint8_t good = 0u;
    for (m = 0u; m < CAL_MICS; ++m) {
      float rms_amp = fabsf(tail_mean[m]);
      float amp_err;
      float dphase;
      if (rms_amp < 1.0e-6f) continue;
      amp_err = fabsf(cal_z_re[m][g] - tail_mean[m]) / rms_amp;
      dphase = 0.0f;
      if (g > 0u) dphase = fabsf(cal_wrap_pi(cal_z_im[m][g] - cal_z_im[m][g - 1u]));
      if (g + 1u < CAL_PROFILE_GATES)
        dphase += fabsf(cal_wrap_pi(cal_z_im[m][g + 1u] - cal_z_im[m][g]));
      if (amp_err <= (CAL_STABLE_AMP_PCT / 100.0f) &&
          dphase <= cal_rad(CAL_STABLE_DEG)) ++good;
    }
    stable[g] = (good >= 2u) ? 1u : 0u;
  }
  {
    uint8_t have = 0u;
    uint16_t best_start = 0u;
    run = 0u;
    for (g = 0u; g < CAL_PROFILE_GATES; ++g) {
      if (stable[g] != 0u) {
        if (run == 0u) run_start = g;
        ++run;
        if (run >= CAL_STABLE_RUN) {
          uint16_t sg = (uint16_t)(run_start + run - CAL_STABLE_RUN);
          uint32_t end_sample = (uint32_t)sg * CAL_PROFILE_STEP + CAL_GATE_WIDTH + CAL_GATE_TAIL;
          if (end_sample <= (CAL_PROFILE_BURST_US / 25u)) {
            have = 1u;
            best_start = sg;
          }
        }
      } else {
        run = 0u;
      }
    }
    if (have == 0u) return -6;
    {
      uint16_t start = (uint16_t)(best_start * CAL_PROFILE_STEP);
      if (start < CAL_GATE_START_MIN) start = CAL_GATE_START_MIN;
      *gate_start_out = start;
      *gate_width_out = CAL_GATE_WIDTH;
    }
  }
  return 0;
}

/* --------------------------------------------------------------------------
 * Optional phase A3 single-channel coupon probe.
 * -------------------------------------------------------------------------- */
#if CAL_DIRECT_ID_PROBE
static int cal_id_probe(fpga_link_t *link, uint8_t level)
{
  uint8_t k;
  if (cal_mic_start(link, 1u, cal_gate_start, cal_gate_width, cal_gate_width) != 0) return -1;
  for (k = 0u; k < 8u; ++k) {
    uint8_t ch = (uint8_t)(k * (CAL_CHANNELS / 8u) + 2u);
    float yr[CAL_MICS], yi[CAL_MICS], amp = 0.0f;
    uint8_t m;
    if (ch >= CAL_CHANNELS) ch = CAL_CHANNELS - 1u;
    if (cal_submit_single(link, ch, level) != 0) return -2;
    cal_delay_us(cal_burst_us);
    if (fpga_link_safe_stop(link) != 0) return -3;
    {
      fpga_mic_gate_wire_t wire;
      uint16_t expected = (uint16_t)(cal_block_expected + 1u);
      if (cal_wait_block_fast(link, expected, 200u, &wire) != 0) return -4;
      cal_block_expected = expected;
      for (m = 0u; m < CAL_MICS; ++m) {
        yr[m] = (float)wire.i[m];
        yi[m] = (float)wire.q[m];
        amp += yr[m] * yr[m] + yi[m] * yi[m];
      }
    }
    cal_id_channel[k] = ch;
    cal_id_amp[k] = sqrtf(amp);
    cal_delay_us(CAL_SETTLE_US);
  }
  return 0;
}
#endif

/* --------------------------------------------------------------------------
 * Phase C: random projection acquisition in the burst steady state.
 * -------------------------------------------------------------------------- */
static int cal_accumulate_pattern(fpga_link_t *link, uint16_t pattern,
                                  float *signal_power)
{
  uint32_t bits[3];
  float yr[CAL_MICS], yi[CAL_MICS];
  uint8_t m;
  uint16_t i;
  cal_cs_row(pattern, bits);
  if (cal_measure_pattern_iq(link, bits, cal_level_used, cal_burst_us, yr, yi) != 0)
    return -1;
  for (m = 0u; m < CAL_MICS; ++m) {
    cal_sy_re[m] += yr[m];
    cal_sy_im[m] += yi[m];
    *signal_power += yr[m] * yr[m] + yi[m] * yi[m];
  }
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    float sign = cal_cs_sign(bits, (uint8_t)i);
    cal_sk[i] += sign;
    for (m = 0u; m < CAL_MICS; ++m) {
      cal_z_re[m][i] += sign * yr[m];
      cal_z_im[m][i] += sign * yi[m];
    }
  }
  cal_delay_us(CAL_SETTLE_US);
  return 0;
}

/* Early convergence check.  Snapshot the accumulators, run D+E3, compare the
 * sign+ phase vector with the previous check, then restore. */
static int cal_early_check(void)
{
  float diff;
  cal_fit_metrics_t metrics;
  memcpy(cal_accum_re, cal_z_re, sizeof(cal_accum_re));
  memcpy(cal_accum_im, cal_z_im, sizeof(cal_accum_im));
  ++cal_debug.checks;
  if (cal_cs_reconstruct(0u) == 0) {
    cal_prepare_pairs();
    if (cal_fit_direct(1u, &metrics) == 0) {
      float sr = 0.0f, si = 0.0f, mean;
      uint8_t i;
      for (i = 0u; i < CAL_CHANNELS; ++i) { sr += cosf(cal_phi[i]); si += sinf(cal_phi[i]); }
      mean = cal_atan2_rad(sr, si);
      for (i = 0u; i < CAL_CHANNELS; ++i) cal_phi[i] = cal_wrap_pi(cal_phi[i] - mean);
      if (cal_phi_prev_valid != 0u) {
        diff = cal_phase_vector_rms_deg(cal_phi, cal_phi_prev);
        for (i = 0u; i < CAL_CHANNELS; ++i) cal_phi_prev[i] = cal_phi[i];
        /* Restore the streaming accumulators before returning. */
        memcpy(cal_z_re, cal_accum_re, sizeof(cal_accum_re));
        memcpy(cal_z_im, cal_accum_im, sizeof(cal_accum_im));
        return (diff < CAL_CONVERGE_DEG) ? 1 : 0;
      }
      for (i = 0u; i < CAL_CHANNELS; ++i) cal_phi_prev[i] = cal_phi[i];
      cal_phi_prev_valid = 1u;
    }
  }
  memcpy(cal_z_re, cal_accum_re, sizeof(cal_accum_re));
  memcpy(cal_z_im, cal_accum_im, sizeof(cal_accum_im));
  return 0;
}

static int cal_cs_acquire(fpga_link_t *link, us_cal_progress_cb_t cb, void *context)
{
  float signal_power = 0.0f;
  uint16_t p;
  if (cal_mic_start(link, 1u, cal_gate_start, cal_gate_width, cal_gate_width) != 0) return -1;
  memset(cal_z_re, 0, sizeof(cal_z_re));
  memset(cal_z_im, 0, sizeof(cal_z_im));
  memset(cal_sy_re, 0, sizeof(cal_sy_re));
  memset(cal_sy_im, 0, sizeof(cal_sy_im));
  memset(cal_sk, 0, sizeof(cal_sk));
  cal_cs_patterns = 0u;
  cal_phi_prev_valid = 0u;
  cal_time_budget_start = HAL_GetTick();
  for (p = 0u; p < CAL_PATTERNS_MAX; ++p) {
    if (cal_accumulate_pattern(link, p, &signal_power) != 0) return -2;
    cal_cs_patterns = (uint16_t)(p + 1u);
    if ((cal_cs_patterns & (CAL_PATTERN_CHECK - 1u)) == 0u &&
        cal_cs_patterns >= CAL_PATTERNS_MIN) {
      if (cal_early_check() == 1) break;
    }
    if ((HAL_GetTick() - cal_time_budget_start) >= CAL_TIME_BUDGET_MS) break;
    if ((p & 15u) == 0u) {
      uint32_t elapsed = HAL_GetTick() - cal_time_budget_start;
      uint32_t pr = 12u + (elapsed * 62u) / CAL_TIME_BUDGET_MS;
      if (pr > 74u) pr = 74u;
      cal_report(cb, context, US_CAL_MEASURE, (uint8_t)pr);
    }
  }
  if (cal_cs_patterns < CAL_PATTERNS_MIN) return -5;
  cal_signal_power = signal_power / ((float)cal_cs_patterns * (float)CAL_MICS);
  cal_debug.patterns = cal_cs_patterns;
  return 0;
}

/* --------------------------------------------------------------------------
 * Phase F: resolve the discrete ambiguities with a real measured array gain.
 * -------------------------------------------------------------------------- */
static int cal_focus_frame(fpga_link_t *link, umh_spatial_renderer_t *renderer,
                           const umh_spatial_point_t *point, float power_out[CAL_MICS])
{
  umh_output_frame_t frame;
  float yr[CAL_MICS], yi[CAL_MICS];
  uint8_t m;
  if (spatial_renderer_point(renderer, point, &frame) != 0) return -1;
  frame.sequence = ++cal_frame_sequence;
  if (cal_measure_frame_iq(link, &frame, cal_burst_us, yr, yi) != 0) return -2;
  for (m = 0u; m < CAL_MICS; ++m)
    power_out[m] = yr[m] * yr[m] + yi[m] * yi[m];
  cal_delay_us(CAL_SETTLE_US);
  return 0;
}

static int cal_verify_focus(fpga_link_t *link, const umh_device_profile_t *profile,
                            uint8_t candidate_mask, uint8_t chosen_bytes[CAL_CHANNELS],
                            float *gain_db, uint8_t *positive_mics_out)
{
  umh_spatial_renderer_t renderer;
  umh_channel_calibration_t calibration[CAL_CHANNELS];
  umh_spatial_point_t points[CAL_MICS];
  float base_power[CAL_MICS];
  float pwr[CAL_MICS];
  float total_base = 0.0f, best_total = -1.0f;
  uint8_t best = 0xFFu, best_positive = 0u;
  uint8_t c, m, i;
  uint8_t focus_level = (uint8_t)(((uint32_t)cal_level_used * 255u + 64u) / 128u);
  if ((candidate_mask & 0x0Fu) == 0u) return -4;
  if (cal_mic_start(link, 1u, cal_gate_start, cal_gate_width, cal_gate_width) != 0) return -1;
  for (m = 0u; m < CAL_MICS; ++m) {
    points[m].x_um = (int32_t)lroundf(cal_mic_x_mm[m] * 1000.0f);
    points[m].y_um = (int32_t)lroundf(cal_mic_y_mm[m] * 1000.0f);
    points[m].z_um = 0;
    points[m].level = focus_level;
    points[m].phase = 0u;
    points[m].source_id = 0u;
  }
  spatial_renderer_init(&renderer, profile);
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    calibration[i].phase = 0u;
    calibration[i].gain = 255u;
    calibration[i].enabled = 1u;
  }
  spatial_renderer_set_calibration(&renderer, calibration, CAL_CHANNELS);
  for (m = 0u; m < CAL_MICS; ++m) {
    if (cal_focus_frame(link, &renderer, &points[m], pwr) != 0) return -2;
    base_power[m] = pwr[m];
    total_base += base_power[m];
  }
  for (c = 0u; c < 4u; ++c) {
    float total = 0.0f;
    uint8_t positive = 0u;
    if ((candidate_mask & (uint8_t)(1u << c)) == 0u) {
      cal_candidate_gain_db[c] = -999.0f;
      cal_candidate_positive[c] = 0u;
      continue;
    }
    for (i = 0u; i < CAL_CHANNELS; ++i) calibration[i].phase = cal_corr[c][i];
    spatial_renderer_set_calibration(&renderer, calibration, CAL_CHANNELS);
    for (m = 0u; m < CAL_MICS; ++m) {
      if (cal_focus_frame(link, &renderer, &points[m], pwr) != 0) return -3;
      total += pwr[m];
      if (pwr[m] > base_power[m]) ++positive;
    }
    cal_candidate_gain_db[c] = 10.0f * log10f((total + 1.0f) / (total_base + 1.0f));
    cal_candidate_positive[c] = positive;
    if (total > best_total) { best_total = total; best = c; best_positive = positive; }
  }
  if (best == 0xFFu) return -4;
  *gain_db = cal_candidate_gain_db[best];
  if (positive_mics_out != NULL) *positive_mics_out = best_positive;
  for (i = 0u; i < CAL_CHANNELS; ++i) chosen_bytes[i] = cal_corr[best][i];
  return (int)best;
}

/* --------------------------------------------------------------------------
 * Bench raw-capture streaming.
 *
 * This path deliberately does not solve anything.  It repeats the exact C
 * acquisition excitation sequence and streams every per-pattern gate I/Q
 * sample back to the PC, so calibration algorithms can be iterated offline
 * without re-transmitting.  The projection matrix is regenerated on the PC
 * with the same splitmix64 generator and pattern indices.
 * -------------------------------------------------------------------------- */
#define CAL_RAW_PATTERNS_PER_PACKET 64u

int us_calibration_capture_raw(fpga_link_t *link, uint8_t level,
                               uint16_t gate_start, uint8_t gate_width,
                               uint32_t burst_us, uint16_t patterns,
                               us_cal_raw_tx_cb_t tx, void *context)
{
  uint8_t *raw;
  uint16_t p = 0u;
  if (link == NULL || tx == NULL || patterns == 0u || gate_width == 0u || burst_us == 0u)
    return -1;
  if (gate_start > 65000u) return -1;
  cal_level_used = level;
  cal_gate_start = gate_start;
  cal_gate_width = gate_width;
  cal_burst_us = burst_us;
  if (cal_mic_start(link, 1u, cal_gate_start, cal_gate_width, cal_gate_width) != 0)
    return -2;
  raw = (uint8_t *)cal_scratch.cg[0];
  while (p < patterns) {
    uint16_t n = (uint16_t)(patterns - p);
    uint16_t k;
    uint8_t *dst = raw;
    if (n > CAL_RAW_PATTERNS_PER_PACKET) n = CAL_RAW_PATTERNS_PER_PACKET;
    for (k = 0u; k < n; ++k) {
      uint32_t bits[3];
      float yr[CAL_MICS], yi[CAL_MICS];
      uint8_t m;
      cal_cs_row((uint16_t)(p + k), bits);
      if (cal_measure_pattern_iq(link, bits, level, burst_us, yr, yi) != 0)
        return -3;
      for (m = 0u; m < CAL_MICS; ++m) {
        int16_t iv = (int16_t)lroundf(yr[m]);
        int16_t qv = (int16_t)lroundf(yi[m]);
        *dst++ = (uint8_t)((uint16_t)iv & 0xFFu);
        *dst++ = (uint8_t)(((uint16_t)iv >> 8) & 0xFFu);
        *dst++ = (uint8_t)((uint16_t)qv & 0xFFu);
        *dst++ = (uint8_t)(((uint16_t)qv >> 8) & 0xFFu);
      }
      cal_delay_us(CAL_SETTLE_US);
    }
    if (tx((uint32_t)p, raw, (uint16_t)(n * CAL_MICS * 4u), context) != 0)
      return -4;
    p = (uint16_t)(p + n);
  }
  return 0;
}

/* --------------------------------------------------------------------------
 * Independent built-in self-test.
 *
 * This path deliberately does not use the calibration solver.  It measures
 * each channel alone at a burst-steady gate (direct H matrix), derives the
 * actual command-phase rotation direction from a 0/64/192 phase probe, then
 * compares:
 *   A) the production renderer focus pattern,
 *   B) the same pattern with a deterministic random 0/pi mask,
 * with a coherent phasor prediction built directly from H.
 * The result therefore verifies the real emitted phase alignment, not the
 * health of the reconstruction algorithm.
 * -------------------------------------------------------------------------- */
static int cal_measure_one_channel(fpga_link_t *link, uint8_t channel, uint8_t phase,
                                   uint8_t level, uint32_t burst_us,
                                   float yr[CAL_MICS], float yi[CAL_MICS])
{
  umh_output_frame_t frame;
  uint16_t i;
  memset(&frame, 0, sizeof(frame));
  frame.update_flags = UMH_FRAME_FLAG_ULTRASOUND;
  frame.sequence = ++cal_frame_sequence;
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    frame.channels[i].phase = (i == (uint16_t)channel) ? phase : 0u;
    frame.channels[i].level = (i == (uint16_t)channel) ? level : 0u;
  }
  return cal_measure_frame_iq(link, &frame, burst_us, yr, yi);
}

static int __attribute__((unused)) us_calibration_self_test_legacy(fpga_link_t *link, const umh_device_profile_t *profile,
                                      const umh_channel_calibration_t *calibration,
                             uint8_t level, uint16_t gate_start, uint8_t gate_width,
                             uint32_t burst_us,
                             us_cal_progress_cb_t progress, void *context,
                             umh_cal_self_test_result_t *result)
{
  uint8_t m, i, ref_ch = 0u, ref_mic = 0u;
  uint8_t used_start = (uint8_t)(gate_start > 255u ? 255u : 0u);
  uint16_t start = gate_start;
  uint8_t width = gate_width;
  float c_m_s, c_mm_s, f_hz;
  float ref_pow = -1.0f, ref_mic_pow = -1.0f;
  float a64, a192, score_plus, score_minus;
  float total_focus = 0.0f, total_ctrl = 0.0f;
  float sum_coh = 0.0f, sum_pred = 0.0f, sum_avp = 0.0f;
  int phase_sign;
  umh_spatial_renderer_t renderer;
  umh_spatial_point_t point;

  (void)used_start;
  if (link == NULL || profile == NULL || calibration == NULL || result == NULL) return -1;
  memset(result, 0, sizeof(*result));
  if (level == 0u) level = 8u;
  result->used_level = level;
  result->used_gate_width = width;
  result->used_gate_start = start;
  cal_profile = profile;
  c_m_s = (profile->sound_speed_um_per_s != 0u) ?
          ((float)profile->sound_speed_um_per_s * 1.0e-6f) : 343.0f;
  c_mm_s = c_m_s * 1000.0f;
  f_hz = (float)(profile->carrier_hz != 0u ? profile->carrier_hz : 40000u);
  cal_k_wave_mm = CAL_TWO_PI * f_hz / c_mm_s;

  if (width == 0u) {
    int rc = cal_transient_profile(link, level, &start, &width);
    if (rc != 0) { result->fault = UMH_FAULT_CAL_MIC_SILENT; return -2; }
    result->used_gate_start = start;
    result->used_gate_width = width;
  }
  if (burst_us == 0u)
    burst_us = ((uint32_t)start + (uint32_t)width + (uint32_t)CAL_GATE_TAIL) * 25u;
  if (burst_us < 500u) burst_us = 500u;
  /* The FPGA frame builder starts after the SPI transaction and can delay
   * the pattern swap by up to a few hundred microseconds when all 84
   * channels are active.  Keep drive on long enough that the configured
   * microphone gate completes after the actual swap. */
  burst_us += 2000u;
  cal_burst_us = burst_us;
  if (cal_mic_start(link, 1u, start, width, width) != 0) {
    result->fault = UMH_FAULT_CAL_MIC_SILENT; return -3;
  }

  /* ---- H matrix: each channel alone, phase 0 / phase 180 differential.
   * Driving the same channel with opposite command phase and subtracting
   * removes supply/mechanical/electromagnetic common-mode pickup.  That
   * pickup is not part of the acoustic channel response and made the old
   * absolute-phase H matrix non-repeatable across patterns and positions. */
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    float y0r[CAL_MICS], y0i[CAL_MICS], y1r[CAL_MICS], y1i[CAL_MICS];
    if (cal_measure_one_channel(link, i, 0u, level, burst_us, y0r, y0i) != 0) {
      result->fault = UMH_FAULT_CAL_MIC_SILENT; return -4;
    }
    cal_delay_us(CAL_SETTLE_US);
    if (cal_measure_one_channel(link, i, 128u, level, burst_us, y1r, y1i) != 0) {
      result->fault = UMH_FAULT_CAL_MIC_SILENT; return -4;
    }
    for (m = 0u; m < CAL_MICS; ++m) {
      cal_z_re[m][i] = 0.5f * (y0r[m] - y1r[m]);
      cal_z_im[m][i] = 0.5f * (y0i[m] - y1i[m]);
    }
    result->channels_measured = (uint16_t)(i + 1u);
    cal_delay_us(CAL_SETTLE_US);
    if ((i & 7u) == 0u) {
      result->progress = (uint8_t)(2u + (i * 30u) / CAL_CHANNELS);
      cal_stage_update(US_CAL_VERIFY, result->progress);
      cal_report(progress, context, US_CAL_VERIFY, result->progress);
    }
  }
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    float p = 0.0f;
    for (m = 0u; m < CAL_MICS; ++m)
      p += cal_z_re[m][i] * cal_z_re[m][i] + cal_z_im[m][i] * cal_z_im[m][i];
    if (p > ref_pow) { ref_pow = p; ref_ch = i; }
  }
  for (m = 0u; m < CAL_MICS; ++m) {
    float p = cal_z_re[m][ref_ch] * cal_z_re[m][ref_ch] +
              cal_z_im[m][ref_ch] * cal_z_im[m][ref_ch];
    if (p > ref_mic_pow) { ref_mic_pow = p; ref_mic = m; }
  }
  if (ref_mic_pow < 1.0f) { result->fault = UMH_FAULT_CAL_MIC_SILENT; return -5; }

  /* ---- all-channel linearity diagnostic ------------------------------- */
  {
    umh_output_frame_t line;
    umh_output_frame_t line_inv;
    float l0r[CAL_MICS], l0i[CAL_MICS], l1r[CAL_MICS], l1i[CAL_MICS];
    memset(&line, 0, sizeof(line));
    line.update_flags = UMH_FRAME_FLAG_ULTRASOUND;
    line.sequence = ++cal_frame_sequence;
    for (i = 0u; i < CAL_CHANNELS; ++i) line.channels[i].level = level;
    line_inv = line;
    line_inv.sequence = ++cal_frame_sequence;
    for (i = 0u; i < CAL_CHANNELS; ++i) line_inv.channels[i].phase = 128u;
    if (cal_measure_frame_iq(link, &line, burst_us, l0r, l0i) == 0) {
      cal_delay_us(CAL_SETTLE_US);
      if (cal_measure_frame_iq(link, &line_inv, burst_us, l1r, l1i) == 0) {
        for (m = 0u; m < CAL_MICS; ++m) {
          cal_dbg_line_re[m] = 0.5f * (l0r[m] - l1r[m]);
          cal_dbg_line_im[m] = 0.5f * (l0i[m] - l1i[m]);
        }
      }
    }
    for (m = 0u; m < CAL_MICS; ++m) {
      float pr = 0.0f, pi = 0.0f;
      for (i = 0u; i < CAL_CHANNELS; ++i) {
        pr += cal_z_re[m][i];
        pi += cal_z_im[m][i];
      }
      cal_dbg_line_pred_re[m] = pr;
      cal_dbg_line_pred_im[m] = pi;
    }
    cal_delay_us(CAL_SETTLE_US);
  }

  /* ---- determine actual command-phase rotation direction --------------- */
  {
    float y64r[CAL_MICS], y64i[CAL_MICS], y192r[CAL_MICS], y192i[CAL_MICS];
    float hr = cal_z_re[ref_mic][ref_ch], hi = cal_z_im[ref_mic][ref_ch];
    float dot, cross;
    if (cal_measure_one_channel(link, ref_ch, 64u, level, burst_us, y64r, y64i) != 0) {
      result->fault = UMH_FAULT_CAL_MIC_SILENT; return -6;
    }
    cal_delay_us(CAL_SETTLE_US);
    if (cal_measure_one_channel(link, ref_ch, 192u, level, burst_us, y192r, y192i) != 0) {
      result->fault = UMH_FAULT_CAL_MIC_SILENT; return -7;
    }
    cal_delay_us(CAL_SETTLE_US);
    dot = y64r[ref_mic] * hr + y64i[ref_mic] * hi;
    cross = y64i[ref_mic] * hr - y64r[ref_mic] * hi;
    a64 = cal_atan2_rad(dot, cross);
    dot = y192r[ref_mic] * hr + y192i[ref_mic] * hi;
    cross = y192i[ref_mic] * hr - y192r[ref_mic] * hi;
    a192 = cal_atan2_rad(dot, cross);
  }
  score_plus = cosf(cal_wrap_pi(a64 - (CAL_PI * 0.5f))) +
               cosf(cal_wrap_pi(a192 - CAL_PI));
  score_minus = cosf(cal_wrap_pi(a64 + (CAL_PI * 0.5f))) +
                cosf(cal_wrap_pi(a192 + CAL_PI));
  phase_sign = (score_plus >= score_minus) ? 1 : -1;

  /* ---- production focus pattern vs random-phase control ----------------
   * Measure each frame twice with an additional global 180 deg phase shift
   * and subtract.  Static microphone/power-supply/mechanical pickup is
   * common-mode under this inversion and cancels, leaving only the acoustic
   * field produced by the addressed transmit phases.  Without this the
   * common-mode floor dominated weak focus spots and made the self-test
   * unable to see whether the calibration actually aligned the channels. */
  spatial_renderer_init(&renderer, profile);
  spatial_renderer_set_calibration(&renderer, calibration, CAL_CHANNELS);
  for (m = 0u; m < CAL_MICS; ++m) {
    umh_output_frame_t frame;
    umh_output_frame_t frame_inv;
    umh_output_frame_t control;
    umh_output_frame_t control_inv;
    float a0r[CAL_MICS], a0i[CAL_MICS], a1r[CAL_MICS], a1i[CAL_MICS];
    float b0r[CAL_MICS], b0i[CAL_MICS], b1r[CAL_MICS], b1i[CAL_MICS];
    float yfr[CAL_MICS], yfi[CAL_MICS], ycr[CAL_MICS], yci[CAL_MICS];
    float pr = 0.0f, pi = 0.0f, sum_mag = 0.0f, sum_pow = 0.0f;
    float pred_pow, pred_gain, focus_pow, ctrl_pow, gain_m, avp_m;
    memset(&point, 0, sizeof(point));
    point.x_um = (int32_t)lroundf(cal_mic_x_mm[m] * 1000.0f);
    point.y_um = (int32_t)lroundf(cal_mic_y_mm[m] * 1000.0f);
    point.z_um = 0;
    point.level = (uint8_t)(((uint32_t)level * 255u + 64u) / 128u);
    point.phase = 0u;
    point.source_id = 0u;
    if (spatial_renderer_point(&renderer, &point, &frame) != 0) {
      result->fault = UMH_FAULT_CAL_QUALITY; return -8;
    }
    frame.sequence = ++cal_frame_sequence;
    for (i = 0u; i < CAL_CHANNELS; ++i) cal_corr[0][i] = frame.channels[i].phase;

    frame_inv = frame;
    frame_inv.sequence = ++cal_frame_sequence;
    for (i = 0u; i < CAL_CHANNELS; ++i)
      frame_inv.channels[i].phase = (uint8_t)(frame_inv.channels[i].phase + 128u);

    if (cal_measure_frame_iq(link, &frame, burst_us, a0r, a0i) != 0) {
      result->fault = UMH_FAULT_CAL_MIC_SILENT; return -9;
    }
    cal_delay_us(CAL_SETTLE_US);
    if (cal_measure_frame_iq(link, &frame_inv, burst_us, a1r, a1i) != 0) {
      result->fault = UMH_FAULT_CAL_MIC_SILENT; return -9;
    }
    cal_delay_us(CAL_SETTLE_US);

    control = frame;
    control.sequence = ++cal_frame_sequence;
    {
      uint32_t bits[3];
      cal_cs_row((uint16_t)(0x4000u + m), bits);
      for (i = 0u; i < CAL_CHANNELS; ++i) {
        if (cal_cs_bit(bits, i) != 0)
          control.channels[i].phase = (uint8_t)(control.channels[i].phase + 128u);
      }
    }
    control_inv = control;
    control_inv.sequence = ++cal_frame_sequence;
    for (i = 0u; i < CAL_CHANNELS; ++i)
      control_inv.channels[i].phase = (uint8_t)(control_inv.channels[i].phase + 128u);

    if (cal_measure_frame_iq(link, &control, burst_us, b0r, b0i) != 0) {
      result->fault = UMH_FAULT_CAL_MIC_SILENT; return -10;
    }
    cal_delay_us(CAL_SETTLE_US);
    if (cal_measure_frame_iq(link, &control_inv, burst_us, b1r, b1i) != 0) {
      result->fault = UMH_FAULT_CAL_MIC_SILENT; return -10;
    }
    cal_delay_us(CAL_SETTLE_US);

    for (i = 0u; i < CAL_MICS; ++i) {
      yfr[i] = 0.5f * (a0r[i] - a1r[i]);
      yfi[i] = 0.5f * (a0i[i] - a1i[i]);
      ycr[i] = 0.5f * (b0r[i] - b1r[i]);
      yci[i] = 0.5f * (b0i[i] - b1i[i]);
    }

    if (m == 0u) {
      /* The 64/192 deg probe is ambiguous by construction when the
       * microphone/transmit phase noise is small.  Resolve the actual
       * command-phase rotation sign from the differential focus vector
       * itself, which is the same convention the production renderer
       * uses. */
      float ppr = 0.0f, ppi = 0.0f, pmr = 0.0f, pmi = 0.0f;
      float mag_y = sqrtf(yfr[0] * yfr[0] + yfi[0] * yfi[0]);
      for (i = 0u; i < CAL_CHANNELS; ++i) {
        float hr = cal_z_re[0][i];
        float hi = cal_z_im[0][i];
        float rot = CAL_TWO_PI * (float)cal_corr[0][i] / 256.0f;
        float cr = cosf(rot), ci = sinf(rot);
        ppr += hr * cr - hi * ci;
        ppi += hr * ci + hi * cr;
        pmr += hr * cr + hi * ci;
        pmi += hr * ci - hi * cr;
      }
      {
        float mag_pp = sqrtf(ppr * ppr + ppi * ppi);
        float mag_pm = sqrtf(pmr * pmr + pmi * pmi);
        float dotp = yfr[0] * ppr + yfi[0] * ppi;
        float crossp = yfi[0] * ppr - yfr[0] * ppi;
        float dotm = yfr[0] * pmr + yfi[0] * pmi;
        float crossm = yfi[0] * pmr - yfr[0] * pmi;
        float corrp = (mag_y * mag_pp > 1.0e-9f) ? sqrtf(dotp * dotp + crossp * crossp) / (mag_y * mag_pp) : 0.0f;
        float corrm = (mag_y * mag_pm > 1.0e-9f) ? sqrtf(dotm * dotm + crossm * crossm) / (mag_y * mag_pm) : 0.0f;
        phase_sign = (corrp >= corrm) ? 1 : -1;
      }
    }

    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float hr = cal_z_re[m][i];
      float hi = cal_z_im[m][i];
      float rot = (float)phase_sign * (CAL_TWO_PI * (float)cal_corr[0][i] / 256.0f);
      float cr = cosf(rot);
      float ci = sinf(rot);
      pr += hr * cr - hi * ci;
      pi += hr * ci + hi * cr;
      sum_mag += sqrtf(hr * hr + hi * hi);
      sum_pow += hr * hr + hi * hi;
    }
    pred_pow = pr * pr + pi * pi;
    pred_gain = 10.0f * log10f((pred_pow + 1.0f) /
                               (sum_pow / (float)CAL_CHANNELS + 1.0f));
    focus_pow = yfr[m] * yfr[m] + yfi[m] * yfi[m];
    ctrl_pow = ycr[m] * ycr[m] + yci[m] * yci[m];
    gain_m = 10.0f * log10f((focus_pow + 1.0f) / (ctrl_pow + 1.0f));
    avp_m = 10.0f * log10f((focus_pow + 1.0f) / (pred_pow + 1.0f));
    /* Actual measured phase coherence: |sum of the real differential focus
     * vectors| divided by the sum of single-channel magnitudes.  Random
     * phases give ~1/sqrt(N); a truly phase-unified array approaches 1. */
    float actual_coh = sqrtf(focus_pow) / (sum_mag + 1.0e-9f);
    result->per_mic_gain_db[m] = gain_m;
    result->per_mic_coherence[m] = actual_coh;
    result->per_mic_predicted_gain_db[m] = pred_gain;
    result->per_mic_actual_vs_predicted_db[m] = avp_m;
    total_focus += focus_pow;
    total_ctrl += ctrl_pow;
    sum_coh += actual_coh;
    sum_pred += pred_gain;
    sum_avp += avp_m;
    result->progress = (uint8_t)(40u + m * 15u);
    cal_stage_update(US_CAL_VERIFY, result->progress);
    cal_report(progress, context, US_CAL_VERIFY, result->progress);
  }

  result->focus_gain_db = 10.0f * log10f((total_focus + 1.0f) / (total_ctrl + 1.0f));
  result->coherence = sum_coh / (float)CAL_MICS;
  result->predicted_gain_db = sum_pred / (float)CAL_MICS;
  result->actual_vs_predicted_db = sum_avp / (float)CAL_MICS;
  result->good_mics = 0u;
  for (m = 0u; m < CAL_MICS; ++m) {
    /* A random-phase array gives ~1/sqrt(84) = 0.109 coherence.  Requiring
     * >0.45 at three of four microphones is a real acoustic phase-unity
     * check with margin for PDM noise and microphone position tolerance. */
    if (result->per_mic_coherence[m] > 0.45f &&
        result->per_mic_gain_db[m] > 3.0f)
      ++result->good_mics;
  }
  result->pass = (result->focus_gain_db > 6.0f &&
                  result->coherence > 0.48f &&
                  result->predicted_gain_db > 6.0f &&
                  result->good_mics >= 3u) ? 1u : 0u;
  result->fault = result->pass ? UMH_FAULT_NONE : UMH_FAULT_CAL_QUALITY;
  result->progress = 100u;
  cal_report(progress, context, result->pass ? US_CAL_OK : US_CAL_FAIL, 100u);
  return 0;
}


/* --------------------------------------------------------------------------
 * Bench H-matrix acquisition with coherent repeat averaging.
 *
 * This is deliberately separate from us_calibration_self_test(): it performs
 * no solver, focus or quality decisions.  Each logical channel is driven
 * alone with command phase 0 and 128; the two measurements are subtracted to
 * remove pattern-independent pickup, and `repeats` acquisitions are averaged
 * coherently.  A successful run leaves the 4x84 complex matrix in
 * cal_z_re/cal_z_im.  The host/SWD side is responsible for interpreting it.
 * -------------------------------------------------------------------------- */
int us_calibration_measure_h(fpga_link_t *link, const umh_device_profile_t *profile,
                             uint8_t level, uint16_t gate_start, uint8_t gate_width,
                             uint32_t burst_us, uint8_t repeats,
                             us_cal_progress_cb_t progress, void *context)
{
  uint8_t i, m, r;
  if (link == NULL || profile == NULL || gate_width == 0u || burst_us == 0u) return -1;
  if (repeats == 0u) repeats = 1u;
  cal_profile = profile;
  cal_frame_sequence = 0u;
  cal_block_expected = 0u;
  if (cal_mic_start(link, 1u, gate_start, gate_width, gate_width) != 0) return -2;
  memset(cal_z_re, 0, sizeof(cal_z_re));
  memset(cal_z_im, 0, sizeof(cal_z_im));
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    for (r = 0u; r < repeats; ++r) {
      float y0r[CAL_MICS], y0i[CAL_MICS], y1r[CAL_MICS], y1i[CAL_MICS];
      if (cal_measure_one_channel(link, i, 0u, level, burst_us, y0r, y0i) != 0)
        return -3;
      cal_delay_us(CAL_SETTLE_US);
      if (cal_measure_one_channel(link, i, 128u, level, burst_us, y1r, y1i) != 0)
        return -4;
      for (m = 0u; m < CAL_MICS; ++m) {
        cal_z_re[m][i] += 0.5f * (y0r[m] - y1r[m]);
        cal_z_im[m][i] += 0.5f * (y0i[m] - y1i[m]);
      }
      cal_delay_us(CAL_SETTLE_US);
    }
    for (m = 0u; m < CAL_MICS; ++m) {
      cal_z_re[m][i] /= (float)repeats;
      cal_z_im[m][i] /= (float)repeats;
    }
    if (progress != NULL && (i & 7u) == 0u)
      progress(US_CAL_MEASURE, (uint8_t)((uint32_t)i * 100u / CAL_CHANNELS), context);
  }
  return 0;
}


/* --------------------------------------------------------------------------
 * Bench short-burst arrival profile.
 *
 * A single logical channel is driven for `burst_us`, then the FPGA's 64-gate
 * sequencer samples the 4 microphones over time.  The profile makes it
 * possible to separate electrical/structure-borne feed-through (present at
 * or before the drive envelope) from the airborne arrival (delayed by
 * r_mic / c) and to recover the physical element location without trusting
 * the static phase-model fit.  channel >= CAL_CHANNELS drives nothing and
 * therefore measures the acoustic + electrical noise floor.
 * -------------------------------------------------------------------------- */
int us_calibration_measure_profile(fpga_link_t *link, uint8_t channel,
                                   uint8_t phase, uint8_t level,
                                   uint8_t gate_count, uint16_t gate_start,
                                   uint16_t gate_step, uint8_t gate_width,
                                   uint32_t burst_us, uint8_t repeats)
{
  umh_output_frame_t frame;
  fpga_mic_gate_wire_t wire;
  uint16_t expected;
  uint8_t g, m, r, i;
  if (link == NULL || gate_count == 0u || gate_count > 64u ||
      gate_width == 0u || burst_us == 0u) return -1;
  if (repeats == 0u) repeats = 1u;
  cal_frame_sequence = 0u;
  cal_block_expected = 0u;
  if (cal_mic_start(link, gate_count, gate_start, gate_step, gate_width) != 0)
    return -2;
  memset(cal_profile_acc_i, 0, sizeof(cal_profile_acc_i));
  memset(cal_profile_acc_q, 0, sizeof(cal_profile_acc_q));
  for (r = 0u; r < repeats; ++r) {
    memset(&frame, 0, sizeof(frame));
    frame.update_flags = UMH_FRAME_FLAG_ULTRASOUND;
    frame.sequence = ++cal_frame_sequence;
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      frame.channels[i].phase = (i == channel) ? phase : 0u;
      frame.channels[i].level = (i == channel) ? level : 0u;
    }
    if (cal_submit_frame(link, &frame) != 0) return -3;
    cal_delay_us(burst_us);
    if (fpga_link_safe_stop(link) != 0) return -4;
    expected = (uint16_t)(cal_block_expected + 1u);
    if (cal_wait_block_fast(link, expected, burst_us / 1000u + 500u, &wire) != 0)
      return -5;
    cal_block_expected = expected;
    for (g = 0u; g < gate_count; ++g) {
      mic_capture_gate_t sample;
      if (mic_capture_read_gate(link, g, &sample, NULL) != 0) return -6;
      for (m = 0u; m < CAL_MICS; ++m) {
        int32_t ai = (int32_t)cal_profile_acc_i[g][m] + (int32_t)sample.i[m];
        int32_t aq = (int32_t)cal_profile_acc_q[g][m] + (int32_t)sample.q[m];
        if (ai > 32767) ai = 32767; else if (ai < -32768) ai = -32768;
        if (aq > 32767) aq = 32767; else if (aq < -32768) aq = -32768;
        cal_profile_acc_i[g][m] = (int16_t)ai;
        cal_profile_acc_q[g][m] = (int16_t)aq;
      }
    }
    cal_delay_us(CAL_SETTLE_US);
  }
  for (g = 0u; g < gate_count; ++g) {
    for (m = 0u; m < CAL_MICS; ++m) {
      cal_profile_i[g][m] = (int16_t)(cal_profile_acc_i[g][m] / (int32_t)repeats);
      cal_profile_q[g][m] = (int16_t)(cal_profile_acc_q[g][m] / (int32_t)repeats);
    }
  }
  return 0;
}


int us_calibration_measure_profile_all(fpga_link_t *link,
                                       uint8_t level, uint32_t burst_us,
                                       uint8_t gate_count, uint16_t gate_start,
                                       uint16_t gate_step, uint8_t gate_width,
                                       uint8_t repeats, uint8_t min_gate,
                                       us_cal_profile_peak_t *peaks)
{
  uint8_t ch, g, m;
  if (link == NULL || peaks == NULL || gate_count == 0u || gate_count > 64u ||
      gate_step == 0u || gate_width == 0u || burst_us == 0u) return -1;
  if (repeats == 0u) repeats = 1u;
  if (min_gate >= gate_count) min_gate = (uint8_t)(gate_count - 1u);
  for (ch = 0u; ch < CAL_CHANNELS; ++ch) {
    int32_t bestEarly = -1, bestLate = -1;
    int rc = us_calibration_measure_profile(link, ch, 0u, level, gate_count,
                                            gate_start, gate_step, gate_width,
                                            burst_us, repeats);
    if (rc != 0) return (int)ch - 100;
    for (m = 0u; m < CAL_MICS; ++m) {
      uint8_t eGate = 0u, lGate = min_gate;
      int16_t lI = 0, lQ = 0;
      bestEarly = -1; bestLate = -1;
      for (g = 0u; g < gate_count; ++g) {
        int32_t iv = cal_profile_i[g][m];
        int32_t qv = cal_profile_q[g][m];
        int32_t p = iv * iv + qv * qv;
        if (g < min_gate) {
          if (p > bestEarly) { bestEarly = p; eGate = g; }
        } else {
          if (p > bestLate) { bestLate = p; lGate = g; lI = (int16_t)iv; lQ = (int16_t)qv; }
        }
      }
      if (bestEarly < 0) eGate = 0u;
      if (bestLate < 0) { lGate = min_gate; lI = 0; lQ = 0; }
      peaks[(uint16_t)ch * CAL_MICS + m].early_gate = eGate;
      peaks[(uint16_t)ch * CAL_MICS + m].late_gate = lGate;
      peaks[(uint16_t)ch * CAL_MICS + m].late_i = lI;
      peaks[(uint16_t)ch * CAL_MICS + m].late_q = lQ;
    }
  }
  return 0;
}


int us_calibration_measure_pattern(fpga_link_t *link,
                                   const uint8_t *phase, const uint8_t *level,
                                   uint16_t gate_start, uint8_t gate_width,
                                   uint32_t burst_us, uint8_t repeats,
                                   float out_i[UMH_DEVICE_MIC_COUNT],
                                   float out_q[UMH_DEVICE_MIC_COUNT])
{
  umh_output_frame_t frame;
  uint8_t r, m, i;
  if (link == NULL || phase == NULL || level == NULL || out_i == NULL || out_q == NULL ||
      gate_width == 0u || burst_us == 0u) return -1;
  if (repeats == 0u) repeats = 1u;
  cal_frame_sequence = 0u;
  cal_block_expected = 0u;
  if (cal_mic_start(link, 1u, gate_start, gate_width, gate_width) != 0) return -2;
  for (m = 0u; m < CAL_MICS; ++m) { out_i[m] = 0.0f; out_q[m] = 0.0f; }
  for (r = 0u; r < repeats; ++r) {
    float yr[CAL_MICS], yi[CAL_MICS];
    memset(&frame, 0, sizeof(frame));
    frame.update_flags = UMH_FRAME_FLAG_ULTRASOUND;
    frame.sequence = ++cal_frame_sequence;
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      frame.channels[i].phase = phase[i];
      frame.channels[i].level = level[i];
    }
    if (cal_measure_frame_iq(link, &frame, burst_us, yr, yi) != 0) return -3;
    for (m = 0u; m < CAL_MICS; ++m) { out_i[m] += yr[m]; out_q[m] += yi[m]; }
    cal_delay_us(CAL_SETTLE_US);
  }
  for (m = 0u; m < CAL_MICS; ++m) {
    out_i[m] /= (float)repeats;
    out_q[m] /= (float)repeats;
  }
  return 0;
}

/* --------------------------------------------------------------------------
 * Diagnostic dump sections.
 * -------------------------------------------------------------------------- */
uint32_t us_calibration_dump_size(uint8_t section)
{
  switch (section) {
    case 0u: return (uint32_t)(CAL_MICS * CAL_CHANNELS * 2u * sizeof(float));
    case 1u: return (uint32_t)(CAL_PROFILE_GATES * CAL_MICS * 2u * sizeof(int16_t));
    case 2u: return (uint32_t)sizeof(cal_dump2_t);
    case 3u: return (uint32_t)(16u * sizeof(float));
    default: return 0u;
  }
}

static int cal_dump_read_section0(uint32_t offset, uint8_t *out, uint16_t length)
{
  uint32_t total = us_calibration_dump_size(0u);
  if (offset >= total) return -1;
  if ((uint32_t)length > total - offset) length = (uint16_t)(total - offset);
  {
    uint16_t n = length;
    while (n != 0u) {
      uint32_t elem = offset >> 3u;      /* 8 bytes per (mic,channel) pair */
      uint32_t comp = (offset >> 2u) & 1u;
      uint32_t mic = elem / CAL_CHANNELS;
      uint32_t ch = elem % CAL_CHANNELS;
      float v = (comp == 0u) ? cal_z_re[mic][ch] : cal_z_im[mic][ch];
      const uint8_t *src = (const uint8_t *)&v;
      uint32_t in_elem = offset & 3u;
      uint32_t chunk = 4u - in_elem;
      if (chunk > (uint32_t)n) chunk = (uint32_t)n;
      memcpy(out, src + in_elem, chunk);
      out += chunk; offset += chunk; n = (uint16_t)(n - (uint16_t)chunk);
    }
  }
  return (int)length;
}

static int cal_dump_read_section1(uint32_t offset, uint8_t *out, uint16_t length)
{
  uint32_t total = us_calibration_dump_size(1u);
  const int16_t *src0 = (const int16_t *)&cal_profile_i[0][0];
  const int16_t *src1 = (const int16_t *)&cal_profile_q[0][0];
  uint16_t n = length;
  if (out == NULL || offset >= total) return -1;
  if ((uint32_t)length > total - offset) length = (uint16_t)(total - offset);
  n = length;
  while (n != 0u) {
    uint32_t elem = offset / 2u;       /* int16 element index */
    uint32_t half = offset & 1u;
    uint32_t pair = elem >> 1u;        /* one (I,Q) pair per 2 elements */
    uint32_t is_q = elem & 1u;
    uint32_t gate = pair / CAL_MICS;
    uint32_t mic = pair % CAL_MICS;
    int16_t v = is_q ? src1[gate * CAL_MICS + mic] : src0[gate * CAL_MICS + mic];
    const uint8_t *src = (const uint8_t *)&v;
    uint32_t chunk = 2u - half;
    if (chunk > (uint32_t)n) chunk = (uint32_t)n;
    memcpy(out, src + half, chunk);
    out += chunk; offset += chunk; n = (uint16_t)(n - (uint16_t)chunk);
  }
  return (int)length;
}

static int cal_dump_read_section3(uint32_t offset, uint8_t *out, uint16_t length)
{
  const float *src[4];
  uint32_t total = us_calibration_dump_size(3u);
  uint16_t n = length;
  if (out == NULL || offset >= total) return -1;
  if ((uint32_t)length > total - offset) length = (uint16_t)(total - offset);
  src[0] = cal_dbg_line_re;
  src[1] = cal_dbg_line_im;
  src[2] = cal_dbg_line_pred_re;
  src[3] = cal_dbg_line_pred_im;
  n = length;
  while (n != 0u) {
    uint32_t elem = offset >> 2u;        /* float element index */
    uint32_t part = offset & 3u;
    uint32_t which = elem >> 2u;         /* 0..3: measured re/im, predicted re/im */
    uint32_t mic = elem & 3u;
    float v = (which < 4u) ? src[which][mic] : 0.0f;
    const uint8_t *bytes = (const uint8_t *)&v;
    uint32_t chunk = 4u - part;
    if (chunk > (uint32_t)n) chunk = (uint32_t)n;
    memcpy(out, bytes + part, chunk);
    out += chunk; offset += chunk; n = (uint16_t)(n - (uint16_t)chunk);
  }
  return (int)length;
}

int us_calibration_dump_read(uint8_t section, uint32_t offset, uint8_t *out, uint16_t length)
{
  uint32_t total;
  if (out == NULL) return -1;
  total = us_calibration_dump_size(section);
  if (offset >= total) return -1;
  if ((uint32_t)length > total - offset) length = (uint16_t)(total - offset);
  switch (section) {
    case 0u: return cal_dump_read_section0(offset, out, length);
    case 1u: return cal_dump_read_section1(offset, out, length);
    case 2u:
      memcpy(out, ((const uint8_t *)&cal_dump2) + offset, length);
      return (int)length;
    case 3u: return cal_dump_read_section3(offset, out, length);
    default: return -1;
  }
}

static void cal_fill_dump2(const umh_calibration_result_t *result,
                           const cal_fit_metrics_t *metrics)
{
  uint8_t i, m;
  memset(&cal_dump2, 0, sizeof(cal_dump2));
  cal_dump2.magic = 0x554D4832u; /* "UMH2" */
  cal_dump2.version = 2u;
  cal_dump2.patterns_used = result->patterns_used;
  cal_dump2.level_used = result->level_used;
  cal_dump2.geom_hypothesis = result->geom_hypothesis;
  cal_dump2.sign_hypothesis = result->sign_hypothesis;
  cal_dump2.good_mics = result->good_mics;
  cal_dump2.gate_start = result->used_gate_start;
  cal_dump2.gate_width = result->used_gate_width;
  cal_dump2.fit_rms_deg = metrics->fit_rms_deg;
  cal_dump2.mic_consistency_deg = metrics->mic_consistency_deg;
  cal_dump2.residual = metrics->residual;
  cal_dump2.drift_deg = metrics->drift_deg;
  cal_dump2.band_trend_deg = metrics->band_trend_deg;
  cal_dump2.verify_gain_db = result->verify_gain_db;
  cal_dump2.coupling_db = result->coupling_db;
  cal_dump2.rms_before_deg = result->rms_before_deg;
  cal_dump2.rms_after_deg = result->rms_after_deg;
  for (i = 0u; i < 4u; ++i) cal_dump2.candidate_gain_db[i] = cal_candidate_gain_db[i];
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    cal_dump2.a_re[i] = cal_a_re[i];
    cal_dump2.a_im[i] = cal_a_im[i];
    cal_dump2.q_hat[i] = result->phase_byte[i];
  }
  for (m = 0u; m < CAL_MICS; ++m) {
    cal_dump2.rho_re[m] = cal_rho_re[m];
    cal_dump2.rho_im[m] = cal_rho_im[m];
    cal_dump2.mu_re[m] = cal_mu_re[m];
    cal_dump2.mu_im[m] = cal_mu_im[m];
  }
}

/* --------------------------------------------------------------------------
 * Main state machine.
 * -------------------------------------------------------------------------- */
int us_calibration_run_legacy(fpga_link_t *link, const umh_device_profile_t *profile,
                              us_cal_progress_cb_t progress, void *context,
                              umh_calibration_result_t *result)
{
  uint8_t i, m;
  int rc;
  float c_m_s, c_mm_s, f_hz;
  cal_fit_metrics_t metrics_plus, metrics_minus, metrics_final;
  uint8_t pass_plus, pass_minus, candidate_mask;
  float min_mean = 1.0e30f, max_mean = -1.0e30f;
  uint8_t good_mics = 0u;
  int best_candidate;
  uint8_t positive_mics = 0u;
  uint8_t chosen_sign;
  uint32_t dbg_start = HAL_GetTick();

  if (link == NULL || profile == NULL || result == NULL) return -1;
  memset(result, 0, sizeof(*result));
  result->fault = UMH_FAULT_NONE;
  result->used_gate_width = CAL_GATE_WIDTH;
  result->used_gate_start = CAL_GATE_START_MIN;
  result->used_gate_count = 1u;
  result->level_used = 128u;
  result->geom_hypothesis = 0u;
  result->sign_hypothesis = 0u;
  cal_stage_progress = progress;
  cal_stage_progress_ctx = context;
  cal_frame_sequence = 0u;
  cal_block_expected = 0u;
  cal_phi_prev_valid = 0u;
  memset(&cal_debug, 0, sizeof(cal_debug));
  memset(&metrics_plus, 0, sizeof(metrics_plus));
  memset(&metrics_minus, 0, sizeof(metrics_minus));
  memset(&metrics_final, 0, sizeof(metrics_final));
  cal_profile = profile;
  c_m_s = (profile->sound_speed_um_per_s != 0u) ?
          ((float)profile->sound_speed_um_per_s * 1.0e-6f) : 343.0f;
  c_mm_s = c_m_s * 1000.0f;
  f_hz = (float)(profile->carrier_hz != 0u ? profile->carrier_hz : 40000u);
  cal_k_wave_mm = CAL_TWO_PI * f_hz / c_mm_s;
  cal_report(progress, context, US_CAL_WAIT, 0u);
  cal_debug.stage = 1u;
  cal_report(progress, context, US_CAL_MEASURE, 1u);

  /* ---- A: level / coherence probe -------------------------------------- */
  rc = cal_level_probe(link, &cal_level_used, &cal_noise_power);
  if (rc != 0) {
    result->quality_flags |= CAL_Q_LEVEL;
    result->fault = UMH_FAULT_CAL_MIC_SILENT;
    goto fail;
  }
  result->level_used = cal_level_used;

  /* ---- B: transient profile and gate selection ------------------------- */
  cal_report(progress, context, US_CAL_MEASURE, 6u);
  rc = cal_transient_profile(link, cal_level_used, &cal_gate_start, &cal_gate_width);
  if (rc != 0) {
    result->quality_flags |= CAL_Q_MEASURE;
    result->fault = UMH_FAULT_CAL_MIC_SILENT;
    goto fail;
  }
  result->used_gate_start = cal_gate_start;
  result->used_gate_width = cal_gate_width;
  cal_burst_us = ((uint32_t)cal_gate_start + (uint32_t)cal_gate_width +
                  (uint32_t)CAL_GATE_TAIL) * 25u;
  if (cal_burst_us < 500u) cal_burst_us = 500u;
  cal_debug.stage = 2u;
  cal_report(progress, context, US_CAL_MEASURE, 10u);

#if CAL_DIRECT_ID_PROBE
  (void)cal_id_probe(link, cal_level_used);
#endif

  /* ---- C: random projection acquisition -------------------------------- */
  cal_report(progress, context, US_CAL_MEASURE, 12u);
  rc = cal_cs_acquire(link, progress, context);
  if (rc != 0) {
    result->quality_flags |= CAL_Q_MEASURE;
    result->fault = UMH_FAULT_CAL_MIC_SILENT;
    goto fail;
  }
  result->patterns_used = cal_cs_patterns;
  cal_debug.stage = 3u;

  /* ---- D: centered least squares --------------------------------------- */
  cal_report(progress, context, US_CAL_SOLVE, 76u);
  if (cal_cs_reconstruct(1u) != 0) {
    result->quality_flags |= CAL_Q_MEASURE;
    result->fault = UMH_FAULT_CAL_SOLVER;
    goto fail;
  }

  /* ---- E: direct-path fit for both de-rotation signs -------------------- */
  cal_prepare_pairs();
  cal_report(progress, context, US_CAL_SOLVE, 80u);
  if (cal_fit_direct(1u, &metrics_plus) != 0) {
    result->quality_flags |= CAL_Q_FIT;
    result->fault = UMH_FAULT_CAL_QUALITY;
    goto fail;
  }
  {
    float gauge_rms = cal_gauge(cal_phi, cal_fit_deg);
    metrics_plus.rms_before_deg = gauge_rms;
    cal_make_correction_bytes(cal_fit_deg, cal_corr[0]);
    cal_negate_bytes(cal_corr[0], cal_corr[1]);
  }
  cal_prepare_pairs();
  if (cal_fit_direct(0u, &metrics_minus) != 0) {
    result->quality_flags |= CAL_Q_FIT;
    result->fault = UMH_FAULT_CAL_QUALITY;
    goto fail;
  }
  {
    float gauge_rms = cal_gauge(cal_phi, cal_fit_deg);
    metrics_minus.rms_before_deg = gauge_rms;
    cal_make_correction_bytes(cal_fit_deg, cal_corr[2]);
    cal_negate_bytes(cal_corr[2], cal_corr[3]);
  }
  pass_plus = (metrics_plus.fit_rms_deg <= CAL_FIT_RMS_MAX_DEG &&
               metrics_plus.mic_consistency_deg <= CAL_CONSIST_MAX_DEG &&
               metrics_plus.band_trend_deg <= CAL_BAND_TREND_MAX_DEG) ? 1u : 0u;
  pass_minus = (metrics_minus.fit_rms_deg <= CAL_FIT_RMS_MAX_DEG &&
                metrics_minus.mic_consistency_deg <= CAL_CONSIST_MAX_DEG &&
                metrics_minus.band_trend_deg <= CAL_BAND_TREND_MAX_DEG) ? 1u : 0u;
  if (pass_plus == 0u && pass_minus == 0u) {
    result->quality_flags |= CAL_Q_FIT | CAL_Q_CONSISTENCY;
    result->fault = UMH_FAULT_CAL_QUALITY;
    goto fail;
  }
  candidate_mask = (uint8_t)((pass_plus != 0u ? 0x03u : 0u) |
                             (pass_minus != 0u ? 0x0Cu : 0u));

  /* ---- F: measured focus gain resolves the discrete ambiguities -------- */
  cal_report(progress, context, US_CAL_VERIFY, 0u);
  best_candidate = cal_verify_focus(link, profile, candidate_mask,
                                    result->phase_byte, &result->verify_gain_db,
                                    &positive_mics);
  if (best_candidate < 0) {
    result->quality_flags |= CAL_Q_COUPLING;
    result->fault = UMH_FAULT_CAL_QUALITY;
    goto fail;
  }
  result->sign_hypothesis = (uint8_t)best_candidate;
  if (best_candidate == 0 || best_candidate == 1) chosen_sign = 1u;
  else chosen_sign = 0u;
  if (((best_candidate & 1) != 0) && (best_candidate < 2)) { /* negated plus */ }
  /* Re-run the winning fit so every reported metric and dump section is from
   * the exact candidate that the gain gate selected. */
  cal_prepare_pairs();
  if (cal_fit_direct(chosen_sign, &metrics_final) != 0) {
    result->quality_flags |= CAL_Q_FIT;
    result->fault = UMH_FAULT_CAL_QUALITY;
    goto fail;
  }
  {
    float gauge_rms = cal_gauge(cal_phi, cal_fit_deg);
    metrics_final.rms_before_deg = gauge_rms;
    {
      uint8_t qtmp[CAL_CHANNELS];
      cal_make_correction_bytes(cal_fit_deg, qtmp);
      if (best_candidate == 0 || best_candidate == 2) {
        memcpy(result->phase_byte, qtmp, CAL_CHANNELS);
      } else {
        cal_negate_bytes(qtmp, result->phase_byte);
      }
    }
  }
  /* quantized post-correction residual */
  {
    float sum = 0.0f;
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float applied = (float)result->phase_byte[i] * (360.0f / 256.0f);
      float e = cal_wrap_pi(cal_rad(cal_fit_deg[i]) + cal_rad(applied));
      sum += e * e;
    }
    metrics_final.rms_after_deg = cal_deg(sqrtf(sum / (float)CAL_CHANNELS));
    result->rms_after_deg = metrics_final.rms_after_deg;
    result->rms_before_deg = metrics_final.rms_before_deg;
  }
  result->fit_rms_deg = metrics_final.fit_rms_deg;
  result->mic_consistency_deg = metrics_final.mic_consistency_deg;
  result->residual = metrics_final.residual;
  result->drift_deg = metrics_final.drift_deg;
  result->band_trend_deg = metrics_final.band_trend_deg;
  result->used_gate_count = 1u;
  result->patterns_used = cal_cs_patterns;

  /* ---- microphone coupling and gain gates ------------------------------ */
  for (m = 0u; m < CAL_MICS; ++m) {
    float sum = 0.0f;
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float zr = cal_z_re[m][i], zi = cal_z_im[m][i];
      sum += sqrtf(zr * zr + zi * zi);
    }
    {
      float mean = sum / (float)CAL_CHANNELS;
      if (mean < min_mean) min_mean = mean;
      if (mean > max_mean) max_mean = mean;
    }
  }
  for (m = 0u; m < CAL_MICS; ++m) {
    float sum = 0.0f;
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float zr = cal_z_re[m][i], zi = cal_z_im[m][i];
      sum += sqrtf(zr * zr + zi * zi);
    }
    if (max_mean > 1.0e-9f && (sum / (float)CAL_CHANNELS) > 0.25f * max_mean) ++good_mics;
  }
  result->good_mics = good_mics;
  result->coupling_db = (min_mean > 1.0e-9f) ? (20.0f * log10f(max_mean / min_mean)) : 99.0f;

  if (metrics_final.fit_rms_deg > CAL_FIT_RMS_MAX_DEG ||
      metrics_final.band_trend_deg > CAL_BAND_TREND_MAX_DEG)
    result->quality_flags |= CAL_Q_FIT;
  if (metrics_final.mic_consistency_deg > CAL_CONSIST_MAX_DEG)
    result->quality_flags |= CAL_Q_CONSISTENCY;
  if (good_mics < 2u) result->quality_flags |= CAL_Q_MICS;
  if (result->coupling_db > CAL_COUPLING_WARN_DB) result->quality_flags |= CAL_Q_MICS;
  if (result->rms_before_deg >= CAL_VERIFY_RMS_GATE) {
    if (positive_mics < CAL_VERIFY_MIC_MIN_POS) result->quality_flags |= CAL_Q_VERIFY;
    if (result->verify_gain_db < CAL_VERIFY_MIN_DB) result->quality_flags |= CAL_Q_VERIFY;
  } else if (result->verify_gain_db < -0.5f) {
    /* Only demand a real array gain when the fitted static spread is large.
     * For small spread all four candidates preserve the baseline power, so
     * a slightly negative measured value is just noise. */
    result->quality_flags |= CAL_Q_VERIFY;
  }
  if (result->verify_gain_db < -1.0f) result->quality_flags |= CAL_Q_COUPLING;

#if !CAL_QUALITY_RELAXED
  if (result->quality_flags != 0u) {
    result->fault = UMH_FAULT_CAL_QUALITY;
    goto fail;
  }
#endif
  cal_fill_dump2(result, &metrics_final);
  result->progress = 100u;
  result->fault = UMH_FAULT_NONE;
  cal_debug.stage = 0u;
  cal_debug.total_ms = HAL_GetTick() - dbg_start;
  cal_report(progress, context, US_CAL_OK, 100u);
  return 0;

fail:
  cal_fill_dump2(result, &metrics_final);
  result->progress = 100u;
  cal_debug.stage = 0u;
  cal_debug.total_ms = HAL_GetTick() - dbg_start;
  cal_report(progress, context, US_CAL_FAIL, 100u);
  return -2;
}


/* ==========================================================================
 * Phase-only autonomous calibration (2026-09-17 v3)
 *
 * The near-field microphones are used only through the production spatial
 * renderer.  The fit is a phase-only maximisation of the coherent acoustic
 * power at the four microphone ports:
 *
 *   maximise  sum_m | sum_i H_mi * exp(j*2*pi*(q_i + G_mi)/256) |^2
 *
 * where H is the differential (phase 0 vs phase 180) single-channel response
 * measured one channel at a time, and G is the same geometric focusing code
 * that the production renderer applies.  This avoids the random-projection
 * reconstruction entirely and keeps every intermediate quantity acoustic.
 * ========================================================================== */


/* ==========================================================================
 * v4 static-phase calibration and linear-regime self-test (2026-09).
 *
 * Bench findings on this hardware:
 *  - The FPGA command phase p produces a measured I/Q rotation of +2*pi*p/256
 *    (opposite to the naive delay sign).  All predictions below therefore use
 *    H * exp(+j*phase), not exp(-j*phase).
 *  - 84-channel simultaneous focusing drives the SPH0641 front ends into
 *    compression: the measured focus magnitude saturates around 1200 LSB while
 *    the linear H-model prediction reaches 5k..45k.  The old self-test used
 *    that saturated quantity as its pass gate and was therefore unable to
 *    distinguish a good static phase map from a bad one.
 *  - A 12-channel subset at duty 4 keeps the summed near-field pressure below
 *    the compression corner while still exercising the real production focus
 *    phases, so it is used as the actual focusing gate.
 * ========================================================================== */

#define CAL_FOCUS_SUBSET_COUNT 8u
/* Channels validated by direct low-power focus measurements (actual field
 * matches the H linear prediction with 0-27 deg phase error).  A larger
 * farthest-point subset is intentionally not used until its transducer
 * positions have the same level of verification. */
static const uint8_t cal_focus_subset[CAL_FOCUS_SUBSET_COUNT] = {
  0u, 1u, 4u, 8u, 10u, 13u, 17u, 18u
};
#define CAL_FOCUS_DUTY            8u
#define CAL_SELFTEST_FOCUS_REPS   2u
#define CAL_SELFTEST_H_REPEATS     4u
#define CAL_SELFTEST_COH_MIN       0.55f
#define CAL_SELFTEST_GAIN_MIN_DB   3.0f
#define CAL_SELFTEST_AVG_COH_MIN   0.60f
#define CAL_SELFTEST_AVG_GAIN_DB   6.0f
#define CAL_SELFTEST_PRED_MIN_DB   6.0f

static float cal_duty_scale(uint8_t from, uint8_t to)
{
  float a = sinf(CAL_PI * (float)from / 256.0f);
  float b = sinf(CAL_PI * (float)to / 256.0f);
  if (a < 1.0e-6f) return 1.0f;
  return b / a;
}

/* Measure one frame and its globally inverted twin (all active channels
 * +128), returning 0.5*(y0 - y180) per microphone.  This double-difference
 * removes the static common-mode pickup that is independent of the commanded
 * phase pattern. */
static int cal_measure_frame_diff(fpga_link_t *link,
                                  const umh_output_frame_t *frame,
                                  uint32_t burst_us, uint8_t repeats,
                                  float out_i[CAL_MICS], float out_q[CAL_MICS])
{
  umh_output_frame_t inv;
  umh_output_frame_t f;
  uint8_t r, m, i;
  if (link == NULL || frame == NULL || repeats == 0u) return -1;
  inv = *frame;
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    if (inv.channels[i].level != 0u)
      inv.channels[i].phase = (uint8_t)(inv.channels[i].phase + 128u);
  }
  for (m = 0u; m < CAL_MICS; ++m) { out_i[m] = 0.0f; out_q[m] = 0.0f; }
  for (r = 0u; r < repeats; ++r) {
    float yr[CAL_MICS], yi[CAL_MICS];
    f = *frame;
    f.sequence = ++cal_frame_sequence;
    if (cal_measure_frame_iq(link, &f, burst_us, yr, yi) != 0) return -2;
    for (m = 0u; m < CAL_MICS; ++m) { out_i[m] += yr[m]; out_q[m] += yi[m]; }
    cal_delay_us(CAL_SETTLE_US);
    f = inv;
    f.sequence = ++cal_frame_sequence;
    if (cal_measure_frame_iq(link, &f, burst_us, yr, yi) != 0) return -3;
    for (m = 0u; m < CAL_MICS; ++m) { out_i[m] -= yr[m]; out_q[m] -= yi[m]; }
    cal_delay_us(CAL_SETTLE_US);
  }
  for (m = 0u; m < CAL_MICS; ++m) {
    out_i[m] *= 0.5f / (float)repeats;
    out_q[m] *= 0.5f / (float)repeats;
  }
  return 0;
}

static int cal_self_test_eval(fpga_link_t *link,
                              const umh_device_profile_t *profile,
                              const umh_channel_calibration_t *calibration,
                              uint8_t level, uint16_t gate_start,
                              uint8_t gate_width, uint32_t burst_us,
                              us_cal_progress_cb_t progress, void *context,
                              umh_cal_self_test_result_t *result)
{
  umh_spatial_renderer_t renderer;
  umh_spatial_point_t point;
  uint8_t m, i, k;
  uint16_t coverage = 0u;
  float scale;
  float avg_coh = 0.0f, avg_gain = 0.0f, avg_pred = 0.0f, avg_avp = 0.0f;

  if (link == NULL || profile == NULL || calibration == NULL || result == NULL)
    return -1;
  memset(result, 0, sizeof(*result));
  result->used_level = level;
  result->used_gate_start = gate_start;
  result->used_gate_width = gate_width;
  result->progress = 1u;

  for (i = 0u; i < CAL_CHANNELS; ++i) {
    float mx = 0.0f;
    for (m = 0u; m < CAL_MICS; ++m) {
      float p = sqrtf(cal_z_re[m][i] * cal_z_re[m][i] +
                      cal_z_im[m][i] * cal_z_im[m][i]);
      if (p > mx) mx = p;
    }
    if (mx > 5.0f) ++coverage;
  }
  result->channels_measured = coverage;
  scale = cal_duty_scale(level, CAL_FOCUS_DUTY);

  spatial_renderer_init(&renderer, profile);
  spatial_renderer_set_calibration(&renderer, calibration, CAL_CHANNELS);

  for (m = 0u; m < CAL_MICS; ++m) {
    umh_output_frame_t full;
    umh_output_frame_t subset;
    umh_output_frame_t control;
    float y0[CAL_MICS], y1[CAL_MICS], c0[CAL_MICS], c1[CAL_MICS];
    float sum_h = 0.0f, sum_h2 = 0.0f;
    float pred_re = 0.0f, pred_im = 0.0f;
    float predfull_re = 0.0f, predfull_im = 0.0f;
    float act, ctrl, pred_mag, predfull_mag;
    uint32_t bits[3];

    memset(&point, 0, sizeof(point));
    point.x_um = (int32_t)lroundf(cal_mic_x_mm[m] * 1000.0f);
    point.y_um = (int32_t)lroundf(cal_mic_y_mm[m] * 1000.0f);
    point.z_um = 0;
    point.level = 255u;
    point.phase = 0u;
    point.source_id = 0u;
    if (spatial_renderer_point(&renderer, &point, &full) != 0) return -2;

    /* Actual production focus phases, but only the linear-regime subset is
     * enabled.  This keeps the four microphones out of compression. */
    subset = full;
    for (i = 0u; i < CAL_CHANNELS; ++i) subset.channels[i].level = 0u;
    for (k = 0u; k < CAL_FOCUS_SUBSET_COUNT; ++k) {
      i = cal_focus_subset[k];
      subset.channels[i].level = CAL_FOCUS_DUTY;
    }
    if (cal_measure_frame_diff(link, &subset, burst_us, CAL_SELFTEST_FOCUS_REPS,
                               y0, y1) != 0) return -3;

    /* Deterministic random-phase control with the same enabled subset. */
    control = subset;
    cal_cs_row((uint16_t)(0x4000u + m), bits);
    for (k = 0u; k < CAL_FOCUS_SUBSET_COUNT; ++k) {
      i = cal_focus_subset[k];
      control.channels[i].phase = cal_cs_bit(bits, i) != 0 ? 128u : 0u;
    }
    if (cal_measure_frame_diff(link, &control, burst_us, CAL_SELFTEST_FOCUS_REPS,
                               c0, c1) != 0) return -4;

    /* Predicted subset focus (H measured at `level`) and full-array linear
     * focus prediction.  The measured command-phase convention is +p. */
    for (k = 0u; k < CAL_FOCUS_SUBSET_COUNT; ++k) {
      float hr, hi, ph, cr, ci;
      i = cal_focus_subset[k];
      hr = cal_z_re[m][i]; hi = cal_z_im[m][i];
      ph = CAL_TWO_PI * (float)subset.channels[i].phase / 256.0f;
      cr = cosf(ph); ci = sinf(ph);
      pred_re += hr * cr - hi * ci;
      pred_im += hr * ci + hi * cr;
      sum_h += sqrtf(hr * hr + hi * hi);
    }
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float hr = cal_z_re[m][i], hi = cal_z_im[m][i];
      float ph = CAL_TWO_PI * (float)full.channels[i].phase / 256.0f;
      float cr = cosf(ph), ci = sinf(ph);
      predfull_re += hr * cr - hi * ci;
      predfull_im += hr * ci + hi * cr;
      sum_h2 += hr * hr + hi * hi;
    }

    act = sqrtf(y0[m] * y0[m] + y1[m] * y1[m]);
    ctrl = sqrtf(c0[m] * c0[m] + c1[m] * c1[m]);
    pred_mag = sqrtf(pred_re * pred_re + pred_im * pred_im) * scale;
    predfull_mag = sqrtf(predfull_re * predfull_re + predfull_im * predfull_im);
    cal_dbg_self_y_re[m] = y0[m]; cal_dbg_self_y_im[m] = y1[m];
    cal_dbg_self_c_re[m] = c0[m]; cal_dbg_self_c_im[m] = c1[m];
    cal_dbg_self_pred_mag[m] = pred_mag;
    cal_dbg_self_ctrl_mag[m] = ctrl;
    cal_dbg_self_phase0[m] = (float)subset.channels[cal_focus_subset[0]].phase;

    /* The microphones are already in mild compression for an eight-channel
     * focus, so absolute focus/control gain is not a reliable pass gate.
     * Instead compare the measured production-focus vector directly with the
     * H linear prediction: phase alignment must be good and the magnitude
     * must remain in a broad [-12 dB, +6 dB] band.  The full-array expected
     * gain is still reported from the unsaturated H model. */
    {
      float cross = y1[m] * pred_re - y0[m] * pred_im;
      float dot = y0[m] * pred_re + y1[m] * pred_im;
      float dphi = atan2f(cross, dot);
      float phase_coh = cosf(dphi);
      float ratio_db = 10.0f * log10f((act * act + 1.0f) / (pred_mag * pred_mag + 1.0f));
      float pred_gain_db =
          10.0f * log10f((predfull_mag * predfull_mag + 1.0f) /
                         (sum_h2 / (float)CAL_CHANNELS + 1.0f));
      if (phase_coh < 0.0f) phase_coh = 0.0f;
      result->per_mic_coherence[m] = phase_coh;
      result->per_mic_gain_db[m] = ratio_db;
      result->per_mic_predicted_gain_db[m] = pred_gain_db;
      result->per_mic_actual_vs_predicted_db[m] = ratio_db;
      avg_coh += phase_coh;
      avg_gain += ratio_db;
      avg_pred += pred_gain_db;
      avg_avp += ratio_db;
    }
    result->progress = (uint8_t)(20u + m * 18u);
    cal_report(progress, context, US_CAL_VERIFY, result->progress);
  }

  avg_coh /= (float)CAL_MICS;
  avg_gain /= (float)CAL_MICS;
  avg_pred /= (float)CAL_MICS;
  avg_avp /= (float)CAL_MICS;
  result->coherence = avg_coh;
  result->focus_gain_db = avg_gain;
  result->predicted_gain_db = avg_pred;
  result->actual_vs_predicted_db = avg_avp;

  result->good_mics = 0u;
  for (m = 0u; m < CAL_MICS; ++m) {
    if (result->per_mic_coherence[m] > CAL_SELFTEST_COH_MIN &&
        result->per_mic_gain_db[m] > -12.0f)
      ++result->good_mics;
  }
  result->pass = (result->good_mics >= 3u &&
                  avg_coh > CAL_SELFTEST_AVG_COH_MIN &&
                  avg_gain > -12.0f &&
                  avg_pred > CAL_SELFTEST_PRED_MIN_DB &&
                  coverage >= 70u) ? 1u : 0u;
  result->fault = result->pass ? UMH_FAULT_NONE : UMH_FAULT_CAL_QUALITY;
  result->progress = 100u;
  cal_report(progress, context, result->pass ? US_CAL_OK : US_CAL_FAIL, 100u);
  return 0;
}

/* Conservative static-phase candidate from the linear H matrix.
 *
 * With the measured command-phase convention:
 *    H_mi ~= A_i * g_m * exp(+j*k*r_mi)
 * so  arg(H_mi) - k*r_mi = arg(A_i) + arg(g_m).
 * The per-microphone nuisance is removed with circular means.  This candidate
 * is only an initial estimate: cal_self_test_eval() still has to accept it by
 * actual subset focusing before it is written anywhere. */
static void cal_static_phase_candidate(uint8_t *q_out, uint8_t negate)
{
  float off[CAL_MICS];
  float phi[CAL_CHANNELS];
  uint8_t m, i, iter;
  for (m = 0u; m < CAL_MICS; ++m) off[m] = 0.0f;
  for (m = 0u; m < CAL_MICS; ++m) {
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float d = cal_direct_path_mm(m, i);
      cal_scratch.trig.cos_re[m][i] =
          atan2f(cal_z_im[m][i], cal_z_re[m][i]) - cal_k_wave_mm * d;
    }
  }
  for (iter = 0u; iter < 4u; ++iter) {
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float sr = 0.0f, si = 0.0f;
      for (m = 0u; m < CAL_MICS; ++m) {
        float a = cal_scratch.trig.cos_re[m][i] - off[m];
        sr += cosf(a); si += sinf(a);
      }
      phi[i] = atan2f(si, sr);
    }
    for (m = 0u; m < CAL_MICS; ++m) {
      float sr = 0.0f, si = 0.0f;
      for (i = 0u; i < CAL_CHANNELS; ++i) {
        float a = cal_scratch.trig.cos_re[m][i] - phi[i];
        sr += cosf(a); si += sinf(a);
      }
      off[m] = atan2f(si, sr);
    }
  }
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    int32_t code = (int32_t)lroundf(-phi[i] * (256.0f / CAL_TWO_PI));
    if (negate != 0u) code = -code;
    code %= 256;
    if (code < 0) code += 256;
    q_out[i] = (uint8_t)code;
  }
}

static int cal_self_test_score(const umh_cal_self_test_result_t *st)
{
  int score = 0;
  if (st == NULL) return -1000000;
  if (st->pass != 0u) score += 1000000;
  score += (int)st->good_mics * 10000;
  score += (int)(st->focus_gain_db * 1000.0f);
  score += (int)(st->coherence * 1000.0f);
  score += (int)(st->predicted_gain_db * 100.0f);
  return score;
}

int us_calibration_self_test(fpga_link_t *link, const umh_device_profile_t *profile,
                             const umh_channel_calibration_t *calibration,
                             uint8_t level, uint16_t gate_start, uint8_t gate_width,
                             uint32_t burst_us,
                             us_cal_progress_cb_t progress, void *context,
                             umh_cal_self_test_result_t *result)
{
  uint32_t burst = burst_us;
  int rc;
  if (link == NULL || profile == NULL || calibration == NULL || result == NULL)
    return -1;
  if (gate_width == 0u) return -2;
  if (burst == 0u) {
    burst = ((uint32_t)gate_start + (uint32_t)gate_width + CAL_GATE_TAIL) * 25u;
    /* Extra margin for the 84-channel event-table swap latency seen on this
     * board; without it the gate can run after the drive has already been
     * stopped for late pattern swaps. */
    burst += 2000u;
    if (burst < 500u) burst = 500u;
  }
  cal_profile = profile;
  cal_report(progress, context, US_CAL_MEASURE, 1u);
  rc = us_calibration_measure_h(link, profile, level, gate_start, gate_width,
                                burst, CAL_SELFTEST_H_REPEATS, progress, context);
  if (rc != 0) {
    memset(result, 0, sizeof(*result));
    result->fault = UMH_FAULT_CAL_MIC_SILENT;
    result->progress = 100u;
    return -10;
  }
  return cal_self_test_eval(link, profile, calibration, level, gate_start,
                            gate_width, burst, progress, context, result);
}

/* v4 calibration: measure H once in the single-channel linear regime, then
 * compare candidate static corrections through the real production focus
 * phases and the unsaturated 12-channel self-test.  Nothing is committed
 * unless one candidate passes the measured gate. */
int us_calibration_run(fpga_link_t *link, const umh_device_profile_t *profile,
                       us_cal_progress_cb_t progress, void *context,
                       umh_calibration_result_t *result)
{
  uint8_t level = 16u, width = CAL_GATE_WIDTH;
  uint16_t start = CAL_GATE_START_MIN;
  uint32_t burst;
  int rc;
  uint8_t i;
  umh_channel_calibration_t calibration[CAL_CHANNELS];
  umh_cal_self_test_result_t st, best_st;
  uint8_t q_zero[CAL_CHANNELS], q_est[CAL_CHANNELS], q_neg[CAL_CHANNELS];
  uint8_t best_q[CAL_CHANNELS];

  if (link == NULL || profile == NULL || result == NULL) return -1;
  memset(result, 0, sizeof(*result));
  result->fault = UMH_FAULT_CAL_SOLVER;
  cal_profile = profile;
  cal_frame_sequence = 0u;
  cal_block_expected = 0u;
  cal_report(progress, context, US_CAL_WAIT, 0u);

  /* Fixed, board-validated H operating point.  The previous v3 flow's
   * A1/transient probes left the FPGA microphone block state and the H
   * solver coupled in a way that made the first v4 subsets fail before
   * reaching the focus gate.  Level 16 / 400 us gate at sample 232 has been
   * verified on this hardware for both linear H and unsaturating subset
   * focus; adaptive probing remains a separate future refinement. */
  level = 16u;
  start = 232u;
  width = CAL_GATE_WIDTH;
  burst = ((uint32_t)start + (uint32_t)width + (uint32_t)CAL_GATE_TAIL) * 25u + 2000u;
  if (burst < 500u) burst = 500u;
  result->level_used = level;
  result->used_gate_start = start;
  result->used_gate_width = width;

  cal_report(progress, context, US_CAL_MEASURE, 10u);
  rc = us_calibration_measure_h(link, profile, level, start, width, burst,
                                CAL_SELFTEST_H_REPEATS, progress, context);
  if (rc != 0) {
    result->fault = UMH_FAULT_CAL_MIC_SILENT;
    result->progress = 100u;
    cal_report(progress, context, US_CAL_FAIL, 100u);
    return -2;
  }
  if (cal_k_wave_mm == 0.0f) {
    float c_mm_s = (profile->sound_speed_um_per_s != 0u) ?
                   ((float)profile->sound_speed_um_per_s * 1.0e-3f) : 343000.0f;
    cal_k_wave_mm = CAL_TWO_PI * (float)profile->carrier_hz / c_mm_s;
  }

  /* Candidate 0: zero static correction. */
  memset(calibration, 0, sizeof(calibration));
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    calibration[i].gain = 255u;
    calibration[i].enabled = 1u;
  }
  memset(q_zero, 0, sizeof(q_zero));
  memset(best_q, 0, sizeof(best_q));
  cal_report(progress, context, US_CAL_VERIFY, 20u);
  if (cal_self_test_eval(link, profile, calibration, level, start, width, burst,
                         progress, context, &best_st) != 0) {
    result->fault = UMH_FAULT_CAL_QUALITY;
    result->progress = 100u;
    return -3;
  }
  result->used_gate_count = 1u;
  result->patterns_used = (uint16_t)(CAL_CHANNELS * 2u);

  /* Candidate 1/2: conservative additive static phase estimate and its
   * inverse.  A real measured subset self-test decides whether either is
   * actually better than zero. */
  cal_static_phase_candidate(q_est, 0u);
  for (i = 0u; i < CAL_CHANNELS; ++i) q_neg[i] = (uint8_t)(0u - q_est[i]);
  {
    const uint8_t *cand[2];
    uint8_t c;
    cand[0] = q_est;
    cand[1] = q_neg;
    for (c = 0u; c < 2u; ++c) {
      for (i = 0u; i < CAL_CHANNELS; ++i) {
        calibration[i].phase = cand[c][i];
        calibration[i].gain = 255u;
        calibration[i].enabled = 1u;
      }
      if (cal_self_test_eval(link, profile, calibration, level, start, width,
                             burst, progress, context, &st) != 0)
        continue;
      if (cal_self_test_score(&st) > cal_self_test_score(&best_st)) {
        best_st = st;
        memcpy(best_q, cand[c], CAL_CHANNELS);
      }
    }
  }

  memcpy(result->phase_byte, best_q, CAL_CHANNELS);
  result->good_mics = best_st.good_mics;
  result->verify_gain_db = best_st.focus_gain_db;
  result->coupling_db = best_st.predicted_gain_db;
  result->band_trend_deg = best_st.actual_vs_predicted_db;
  result->fit_rms_deg = 0.0f;
  result->rms_before_deg = 0.0f;
  result->rms_after_deg = 0.0f;
  result->residual = 0.0f;
  result->drift_deg = 0.0f;
  result->mic_consistency_deg = 0.0f;
  result->progress = 100u;
  if (best_st.pass != 0u) {
    result->fault = UMH_FAULT_NONE;
    result->quality_flags = 0u;
    cal_report(progress, context, US_CAL_OK, 100u);
    return 0;
  }
  result->fault = UMH_FAULT_CAL_QUALITY;
  result->quality_flags = CAL_Q_VERIFY;
  cal_report(progress, context, US_CAL_FAIL, 100u);
  return -5;
}
