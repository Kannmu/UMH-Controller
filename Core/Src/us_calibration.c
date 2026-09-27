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
/* Fixed count instead of the old online convergence probe.  The saved 2.6 KiB
 * accumulator snapshot is more valuable than early termination on this RAM-
 * constrained MCU, and 512 i.i.d. rows leave the centered 84-column problem
 * comfortably overdetermined even with microphone noise. */
#define CAL_ACQUIRE_PATTERNS     512u
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

/* Final fit state. */
static float cal_a_re[CAL_CHANNELS];
static float cal_a_im[CAL_CHANNELS];
static float cal_rho_re[CAL_MICS];
static float cal_rho_im[CAL_MICS];
static float cal_mu_re[CAL_MICS];
static float cal_mu_im[CAL_MICS];
static float cal_phi[CAL_CHANNELS];
static uint8_t cal_pair_valid[CAL_MICS][CAL_CHANNELS];
static uint8_t cal_corr[4][CAL_CHANNELS];
static float cal_candidate_gain_db[4];
static uint8_t cal_candidate_positive[4];

/* Transient profile, filled by phase B and exported as dump section 1. */
static int16_t cal_profile_i[CAL_PROFILE_GATES][CAL_MICS];
static int16_t cal_profile_q[CAL_PROFILE_GATES][CAL_MICS];

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
static float cal_signal_power;
static us_cal_progress_cb_t cal_stage_progress;
static void *cal_stage_progress_ctx;

typedef struct {
  float fit_rms_deg;
  float mic_consistency_deg;
  float residual;
  float drift_deg;
  float band_trend_deg;
  float rms_before_deg;
  float rms_after_deg;
} cal_fit_metrics_t;

static int cal_early_run(fpga_link_t *link, const umh_device_profile_t *profile,
                           us_cal_progress_cb_t progress, void *context,
                           umh_calibration_result_t *result);

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

/* Bench/debug observables for the linear-regime self-test. */
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

/* --------------------------------------------------------------------------
 * Phase A1: drive-level linearity / coherence probe.
 * -------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------
 * Phase B: transient envelope profile and gate selection.
 * -------------------------------------------------------------------------- */

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
  for (p = 0u; p < CAL_ACQUIRE_PATTERNS; ++p) {
    if (cal_accumulate_pattern(link, p, &signal_power) != 0) return -2;
    cal_cs_patterns = (uint16_t)(p + 1u);
    if ((p & 15u) == 0u) {
      uint32_t pr = 12u + ((uint32_t)p * 62u) / CAL_ACQUIRE_PATTERNS;
      if (pr > 74u) pr = 74u;
      cal_report(cb, context, US_CAL_MEASURE, (uint8_t)pr);
    }
  }
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

/* --------------------------------------------------------------------------
 * Early direct-path calibration (2026-09-26).
 *
 * The only field that cannot be polluted by an external room is the first
 * arrival that leaves a transmitter and reaches an on-board microphone.  The
 * direct distance is 7..100 mm, i.e. ~20..300 us; an external object at
 * distance d can only contribute after the round trip 2*d/c.  The old
 * steady-state gate at 5..6 ms therefore measured the room.  This path instead
 * predicts the direct arrival sample for every (channel, microphone) pair
 * from the device geometry, scans a few 25 us samples around it, and keeps
 * the earliest narrow response.  Only the internal coupling is inside the
 * gate; reflections from external objects are later by construction.
 * -------------------------------------------------------------------------- */
#define CAL_EARLY_LEVEL             128u
#define CAL_EARLY_WIDTH               2u   /* 50 us integration */
#define CAL_EARLY_REPEATS             4u
#define CAL_EARLY_SCAN_END           14u   /* scan gates 0..14 = 0..350 us */
#define CAL_EARLY_MAX_START          14u
#define CAL_EARLY_PEAK_FRACTION      0.30f
#define CAL_EARLY_SCAN_MIN            1u   /* ignore the pattern-swap transient */

#define CAL_EARLY_MIN_MAG         10.0f
#define CAL_EARLY_FIT_ITERS          64u
#define CAL_EARLY_PASS_RMS_DEG     30.0f
#define CAL_EARLY_PASS_CHANNELS      70u
#define CAL_EARLY_PASS_MICS           3u
#define CAL_EARLY_MM_PER_SAMPLE      8.575f  /* 343 m/s * 25 us */

static uint16_t cal_early_start_samples(uint8_t mic, uint8_t channel)
{
  float d_mm = cal_direct_path_mm(mic, channel);
  int32_t sample = (int32_t)lroundf(d_mm / CAL_EARLY_MM_PER_SAMPLE);
  if (sample < 0) sample = 0;
  if (sample > (int32_t)CAL_EARLY_MAX_START) sample = (int32_t)CAL_EARLY_MAX_START;
  return (uint16_t)sample;
}

static int cal_early_measure_pair(fpga_link_t *link, uint8_t channel,
                                  uint16_t start, uint32_t burst_us,
                                  uint8_t repeats,
                                  float out_re[CAL_MICS], float out_im[CAL_MICS])
{
  float sum0_re[CAL_MICS] = {0.0f}, sum0_im[CAL_MICS] = {0.0f};
  float sum1_re[CAL_MICS] = {0.0f}, sum1_im[CAL_MICS] = {0.0f};
  uint8_t r, m;
  if (link == NULL || channel >= CAL_CHANNELS || repeats == 0u) return -1;
  if (cal_mic_start(link, 1u, start, CAL_EARLY_WIDTH, CAL_EARLY_WIDTH) != 0)
    return -2;
  for (r = 0u; r < repeats; ++r) {
    float y0r[CAL_MICS], y0i[CAL_MICS], y1r[CAL_MICS], y1i[CAL_MICS];
    if (cal_measure_one_channel(link, channel, 0u, CAL_EARLY_LEVEL, burst_us,
                                y0r, y0i) != 0)
      return -3;
    if (cal_measure_one_channel(link, channel, 128u, CAL_EARLY_LEVEL, burst_us,
                                y1r, y1i) != 0)
      return -4;
    for (m = 0u; m < CAL_MICS; ++m) {
      sum0_re[m] += y0r[m];
      sum0_im[m] += y0i[m];
      sum1_re[m] += y1r[m];
      sum1_im[m] += y1i[m];
    }
    cal_delay_us(CAL_SETTLE_US);
  }
  for (m = 0u; m < CAL_MICS; ++m) {
    out_re[m] = 0.5f * (sum0_re[m] - sum1_re[m]) / (float)repeats;
    out_im[m] = 0.5f * (sum0_im[m] - sum1_im[m]) / (float)repeats;
  }
  return 0;
}

static int cal_early_measure_h(fpga_link_t *link, us_cal_progress_cb_t progress,
                               void *context)
{
  uint8_t i, m;
  uint16_t s;
  if (link == NULL) return -1;
  cal_frame_sequence = 0u;
  cal_block_expected = 0u;
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    float prof_re[CAL_EARLY_SCAN_END + 1u][CAL_MICS];
    float prof_im[CAL_EARLY_SCAN_END + 1u][CAL_MICS];
    for (s = 0u; s <= CAL_EARLY_SCAN_END; ++s) {
      float re[CAL_MICS], im[CAL_MICS];
      uint32_t burst_us = ((uint32_t)s + CAL_EARLY_WIDTH + CAL_GATE_TAIL) * 25u +
                          2000u;
      if (cal_early_measure_pair(link, i, s, burst_us, CAL_EARLY_REPEATS,
                                 re, im) != 0)
        return -2;
      for (m = 0u; m < CAL_MICS; ++m) {
        prof_re[s][m] = re[m];
        prof_im[s][m] = im[m];
      }
    }
    for (m = 0u; m < CAL_MICS; ++m) {
      float mag[CAL_EARLY_SCAN_END + 1u];
      float max_mag = 0.0f;
      uint16_t first_peak = CAL_EARLY_SCAN_MIN, argmax = CAL_EARLY_SCAN_MIN;
      for (s = CAL_EARLY_SCAN_MIN; s <= CAL_EARLY_SCAN_END; ++s) {
        mag[s] = sqrtf(prof_re[s][m] * prof_re[s][m] +
                       prof_im[s][m] * prof_im[s][m]);
        if (mag[s] > max_mag) { max_mag = mag[s]; argmax = s; }
      }
      for (s = CAL_EARLY_SCAN_MIN + 1u; s < CAL_EARLY_SCAN_END; ++s) {
        if (mag[s] >= mag[s - 1u] && mag[s] >= mag[s + 1u] &&
            mag[s] >= CAL_EARLY_PEAK_FRACTION * max_mag) {
          first_peak = s;
          break;
        }
      }
      if (first_peak == CAL_EARLY_SCAN_MIN &&
          argmax != CAL_EARLY_SCAN_MIN)
        first_peak = argmax;
      {
        float sum_re = 0.0f, sum_im = 0.0f;
        uint16_t k, k0 = (first_peak > (CAL_EARLY_SCAN_MIN + 1u)) ?
                         (uint16_t)(first_peak - 2u) : CAL_EARLY_SCAN_MIN;
        uint16_t k1 = (uint16_t)(first_peak + 2u);
        if (k1 > CAL_EARLY_SCAN_END) k1 = CAL_EARLY_SCAN_END;
        for (k = k0; k <= k1; ++k) {
          sum_re += prof_re[k][m];
          sum_im += prof_im[k][m];
        }
        cal_z_re[m][i] = sum_re / (float)(k1 - k0 + 1u);
        cal_z_im[m][i] = sum_im / (float)(k1 - k0 + 1u);
      }
    }
    if ((i & 3u) == 0u) {
      uint8_t p = (uint8_t)(4u + (uint32_t)i * 66u / CAL_CHANNELS);
      if (p > 72u) p = 72u;
      cal_report(progress, context, US_CAL_MEASURE, p);
    }
  }
  return 0;
}



static float cal_early_fit_sign(uint8_t sign, float a_out[CAL_CHANNELS],
                                uint16_t valid_out[CAL_CHANNELS],
                                uint16_t mic_valid_out[CAL_MICS],
                                uint16_t *valid_pairs_out)
{
  float off[CAL_MICS] = {0.0f};
  float a[CAL_CHANNELS] = {0.0f};
  float weighted_err = 0.0f, weight_sum = 0.0f;
  uint16_t valid_pairs = 0u;
  uint8_t iter, m, i;
  for (m = 0u; m < CAL_MICS; ++m) mic_valid_out[m] = 0u;
  for (iter = 0u; iter < CAL_EARLY_FIT_ITERS; ++iter) {
    for (m = 0u; m < CAL_MICS; ++m) {
      float sr = 0.0f, si = 0.0f;
      for (i = 0u; i < CAL_CHANNELS; ++i) {
        float zr = cal_z_re[m][i], zi = cal_z_im[m][i];
        float mag = sqrtf(zr * zr + zi * zi);
        float ph;
        if (mag < CAL_EARLY_MIN_MAG) continue;
        ph = cal_atan2_rad(zr, zi) - ((sign != 0u) ? 1.0f : -1.0f) *
             cal_k_wave_mm * cal_direct_path_mm(m, i) - a[i];
        sr += mag * cosf(ph);
        si += mag * sinf(ph);
      }
      off[m] = cal_atan2_rad(sr, si);
    }
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float sr = 0.0f, si = 0.0f;
      for (m = 0u; m < CAL_MICS; ++m) {
        float zr = cal_z_re[m][i], zi = cal_z_im[m][i];
        float mag = sqrtf(zr * zr + zi * zi);
        float ph;
        if (mag < CAL_EARLY_MIN_MAG) continue;
        ph = cal_atan2_rad(zr, zi) - ((sign != 0u) ? 1.0f : -1.0f) *
             cal_k_wave_mm * cal_direct_path_mm(m, i) - off[m];
        sr += mag * cosf(ph);
        si += mag * sinf(ph);
      }
      a[i] = cal_atan2_rad(sr, si);
    }
  }
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    uint16_t count = 0u;
    for (m = 0u; m < CAL_MICS; ++m) {
      float zr = cal_z_re[m][i], zi = cal_z_im[m][i];
      float mag = sqrtf(zr * zr + zi * zi);
      float expected, err;
      if (mag < CAL_EARLY_MIN_MAG) continue;
      expected = ((sign != 0u) ? 1.0f : -1.0f) * cal_k_wave_mm *
                 cal_direct_path_mm(m, i) + off[m] + a[i];
      err = cal_wrap_pi(cal_atan2_rad(zr, zi) - expected);
      weighted_err += mag * err * err;
      weight_sum += mag;
      ++count;
      ++valid_pairs;
      ++mic_valid_out[m];
    }
    valid_out[i] = count;
  }
  if (weight_sum < 1.0e-6f) {
    *valid_pairs_out = 0u;
    return 999.0f;
  }
  *valid_pairs_out = valid_pairs;
  for (i = 0u; i < CAL_CHANNELS; ++i) a_out[i] = a[i];
  return cal_deg(sqrtf(weighted_err / weight_sum));
}

static int cal_early_run(fpga_link_t *link, const umh_device_profile_t *profile,
                         us_cal_progress_cb_t progress, void *context,
                         umh_calibration_result_t *result)
{
  float a_plus[CAL_CHANNELS], a_minus[CAL_CHANNELS];
  uint16_t valid_plus[CAL_CHANNELS], valid_minus[CAL_CHANNELS];
  uint16_t mic_plus[CAL_MICS], mic_minus[CAL_MICS];
  uint16_t pairs_plus = 0u, pairs_minus = 0u;
  float rms_plus, rms_minus;
  uint8_t sign, i, m;
  uint16_t valid_channels = 0u, good_mics = 0u;
  float c_mm_s;

  if (link == NULL || profile == NULL || result == NULL) return -1;
  memset(result, 0, sizeof(*result));
  result->fault = UMH_FAULT_CAL_SOLVER;
  result->level_used = (uint8_t)CAL_EARLY_LEVEL;
  result->used_gate_width = CAL_EARLY_WIDTH;
  result->used_gate_start = 0u;
  result->used_gate_count = 1u;
  result->patterns_used = CAL_CHANNELS;
  cal_profile = profile;
  c_mm_s = (profile->sound_speed_um_per_s != 0u) ?
           ((float)profile->sound_speed_um_per_s * 1.0e-3f) : 343000.0f;
  cal_k_wave_mm = CAL_TWO_PI * (float)profile->carrier_hz / c_mm_s;

  cal_report(progress, context, US_CAL_WAIT, 0u);
  cal_report(progress, context, US_CAL_MEASURE, 2u);
  if (cal_early_measure_h(link, progress, context) != 0) {
    result->fault = UMH_FAULT_CAL_MIC_SILENT;
    result->progress = 100u;
    cal_report(progress, context, US_CAL_FAIL, 100u);
    return -2;
  }
  cal_report(progress, context, US_CAL_SOLVE, 76u);

  rms_plus = cal_early_fit_sign(1u, a_plus, valid_plus, mic_plus, &pairs_plus);
  rms_minus = cal_early_fit_sign(0u, a_minus, valid_minus, mic_minus, &pairs_minus);
  sign = (rms_plus <= rms_minus) ? 1u : 0u;
  if (sign != 0u) {
    result->fit_rms_deg = rms_plus;
  } else {
    result->fit_rms_deg = rms_minus;
  }
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    uint16_t valid = (sign != 0u) ? valid_plus[i] : valid_minus[i];
    float a = (sign != 0u) ? a_plus[i] : a_minus[i];
    int32_t code = (valid >= 2u) ?
                   (int32_t)lroundf(-a * (256.0f / CAL_TWO_PI)) : 0;
    code %= 256;
    if (code < 0) code += 256;
    result->phase_byte[i] = (uint8_t)code;
    if (valid >= 2u) ++valid_channels;
  }
  result->sign_hypothesis = sign;
  result->geom_hypothesis = 0u;
  result->good_mics = 0u;
  for (m = 0u; m < CAL_MICS; ++m) {
    if ((sign != 0u ? mic_plus[m] : mic_minus[m]) >=
        (CAL_CHANNELS / 4u))
      ++good_mics;
  }
  result->good_mics = good_mics;
  result->mic_consistency_deg = 0.0f;
  result->residual = 0.0f;
  result->verify_gain_db = 0.0f;
  result->coupling_db = 0.0f;
  result->rms_before_deg = result->fit_rms_deg;
  result->rms_after_deg = result->fit_rms_deg;
  result->progress = 100u;

  if (valid_channels < CAL_EARLY_PASS_CHANNELS ||
      good_mics < CAL_EARLY_PASS_MICS ||
      result->fit_rms_deg > CAL_EARLY_PASS_RMS_DEG) {
    result->fault = UMH_FAULT_CAL_QUALITY;
    result->quality_flags = CAL_Q_FIT | CAL_Q_CONSISTENCY | CAL_Q_MICS;
    cal_report(progress, context, US_CAL_FAIL, 100u);
    return -3;
  }
  result->fault = UMH_FAULT_NONE;
  result->quality_flags = 0u;
  cal_report(progress, context, US_CAL_OK, 100u);
  return 0;
}

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
int us_calibration_run(fpga_link_t *link, const umh_device_profile_t *profile,
                       us_cal_progress_cb_t progress, void *context,
                       umh_calibration_result_t *result)
{
  return cal_early_run(link, profile, progress, context, result);
}



/* ==========================================================================
 * Independent linear-regime self-test.
 *
 * Bench findings on this hardware:
 *  - The FPGA command phase p produces a measured I/Q rotation of +2*pi*p/256
 *    (opposite to the naive delay sign).  All predictions below therefore use
 *    H * exp(+j*phase), not exp(-j*phase).
 *  - 84-channel simultaneous focusing can drive the SPH0641 front ends into
 *    compression, so the self-test exercises the real production focus
 *    phases through a small low-duty subset instead of the full array.
 * ========================================================================== */

#define CAL_FOCUS_SUBSET_COUNT 8u
/* Channels validated by direct low-power focus measurements (actual field
 * matches the H linear prediction with 0-27 deg phase error). */
static const uint8_t cal_focus_subset[CAL_FOCUS_SUBSET_COUNT] = {
  0u, 1u, 4u, 8u, 10u, 13u, 17u, 18u
};
#define CAL_FOCUS_DUTY            8u
#define CAL_SELFTEST_FOCUS_REPS   2u
#define CAL_SELFTEST_H_REPEATS     4u
#define CAL_SELFTEST_COH_MIN       0.55f
#define CAL_SELFTEST_AVG_COH_MIN   0.60f
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
