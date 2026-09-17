/* UMH v7 four-microphone echo self-calibration.
 *
 * Measurement chain (compressed sensing, 2026-09):
 *   1. A coarse then a fine echo survey locates the wall echo.  Both surveys
 *      transmit balanced +/-1 random phase patterns; a dedicated all-on probe
 *      is stored for the FMAC matched filter that finds the echo leading
 *      edge.  Balanced patterns cancel the common-mode direct coupling and
 *      the LC tank ring-down in each projection.
 *   2. The final acquisition transmits M pseudo-random +/-1 phase
 *      patterns (phase 0 / 128), drawn i.i.d. so the measurement matrix spans the complete 84-dimensional channel space.  The FPGA stays gate-locked to the pattern
 *      swap and captures one short 40 kHz I/Q gate per pattern for each of
 *      the four microphones.  This is the compressed-sensing measurement:
 *          y[p][m] = sum_i c[p][i] * z[i][m] + n[p][m]
 *      where c is the random measurement matrix.  Every new random row is an
 *      independent linear projection, so rank(C) grows to full rank in the
 *      84-dimensional channel space; extra rows average down noise.
 *   3. Streaming accumulation forms b = C^H y only.  The normal matrix
 *      A = C^H C is never stored: the solver evaluates A x = C^H (C x)
 *      on demand, regenerating the same deterministic PRNG rows.  The
 *      reconstruction is the convex least-squares problem
 *          min_x 0.5 ||C x - y||^2  (  + lambda ||x||_1 )
 *      solved by conjugate gradients when M >= 84, or FISTA/L1 when the
 *      measurement is deliberately under-sampled.
 *   4. A wall-image model with unknown distance and two tilt angles is
 *      fitted together with a per-microphone common-mode nuisance mu_m that absorbs residual LC ring at the gate; the per-channel static phase is obtained from a rank-1
 *      a_i * rho_j fit.  Alternating the fit with a pose refinement on the mu-corrected data removes the ring/pose coupling.  The CORDIC unit accelerates every sin/cos/atan2/
 *      sqrt in the model; the FMAC performs the probe matched filter.
 *      A final one-pattern array-gain check chooses the correction sign.
 */
#include "us_calibration.h"
#include "mic_capture.h"
#include "main.h"
#include "cmsis_os.h"
#include "system_status.h"
#include "cordic.h"
#include "fmac.h"
#include <math.h>
#include <string.h>

#define CAL_CHANNELS  UMH_DEVICE_CHANNEL_COUNT
#define CAL_MICS      UMH_DEVICE_MIC_COUNT
#define CAL_PI        3.14159265358979323846f
#define CAL_TWO_PI    6.28318530717958647692f

/* --------------------------------------------------------------------------
 * Survey / acquisition timing.  The series LC tank has Q ~= 20..50 at
 * 40 kHz, so each random pattern is transmitted long enough for the tank to
 * settle but stopped well before the wall echo returns.  The settle time
 * after the pattern keeps the next projection free of previous ringing.
 * -------------------------------------------------------------------------- */
#define CAL_SURVEY_GATES       64u
#define CAL_SURVEY_COARSE_STEP 16u   /* 400 us, covers 0.15..4.3 m          */
#define CAL_SURVEY_FINE_STEP    4u   /* 100 us around the detected echo     */
#define CAL_SURVEY_WIDTH        8u   /* 200 us integration window           */
#define CAL_SURVEY_PATTERNS     8u   /* balanced random projections          */
#define CAL_SURVEY_NOISE_BLOCKS 2u
#define CAL_SURVEY_BURST_US    700u  /* survey probe, stopped before echo    */

/* Final compressed-sensing acquisition.  The burst is shortened when the
 * measured echo is close, and the quiet gap is deliberately long compared
 * with the LC tank ring-down so every projection starts from a clean tank.
 * With Q ~= 20..50 the 2.5 ms gap gives 6.5..16 time constants of decay. */
#define CAL_CS_BURST_MIN_US    400u
#define CAL_CS_BURST_MAX_US   1400u
#define CAL_CS_BURST_MARGIN_US 300u  /* stop this long before echo onset     */
#define CAL_CS_RING_GUARD_US   300u  /* ignore the transmit interval         */
#define CAL_CS_SETTLE_US      2500u  /* post-gate quiet time                  */
#define CAL_CS_GATE_WIDTH        8u  /* 200 us final echo gate               */
#define CAL_CS_GATE_LATE_SAMPLES 8u  /* start 200 us after the detected onset */
#define CAL_CS_NOISE_BLOCKS      4u
#define CAL_CS_MIN_PATTERNS     96u  /* > CAL_CHANNELS guarantees full rank   */
#define CAL_CS_MAX_PATTERNS   2048u
#define CAL_CS_TIME_BUDGET_MS 10000u
#define CAL_CS_CG_ITERS        28u
#define CAL_CS_CG_TOL          1.0e-4f
#define CAL_CS_FISTA_ITERS     64u
#define CAL_CS_POWER_ITERS      6u
#define CAL_FIT_MU_ROUNDS       4u   /* outer rank-1 + common-mode iterations */
#define CAL_FIT_RANK_ITERS      8u   /* rank-1 alternating iterations per round  */
#define CAL_CS_L1_SIGMA        2.5f
#define CAL_DEFAULT_DISTANCE_MM 300.0f
#define CAL_FMAC_TAPS           8u

/* Solver budgets.  The previous firmware could iterate the local pose search
 * without a bound; these limits turn a pathological fit into a controlled
 * failure instead of an indefinitely busy UI. */
#define CAL_SOLVE_TIME_BUDGET_MS 15000u
#define CAL_POSE_GRID_STEP_DEG   15.0f
#define CAL_POSE_GRID_RANGE       3
#define CAL_POSE_LOCAL_PASSES      8u

#ifndef CAL_QUALITY_RELAXED
/* Bring-up switch: 1 completes the flow and writes EEPROM even when the
 * environment is not a clean plane; set to 0 for final accuracy gates. */
#define CAL_QUALITY_RELAXED 0
#endif

#define CAL_MIN_ECHO_RATIO   2.5f
#define CAL_MAX_MIC_RATIO    2.5f
#define CAL_MAX_RESIDUAL     1.0e-2f
#define CAL_MAX_MIC_CONSIST  12.0f
#define CAL_MAX_PRERMS       90.0f
#define CAL_MAX_TILT         55.0f
#define CAL_MIN_VERIFY_DB    3.0f
#define CAL_SURVEY_MIN_SNR   2.5f

/* Runtime debug counters.  Kept as plain globals so GDB can read them live;
 * they also make it possible to prove where a calibration run spends time. */
typedef struct {
  volatile uint32_t stage;
  volatile uint32_t stage_ms[8];
  volatile uint32_t total_ms;
  volatile uint32_t patterns;
  volatile uint32_t pose_evals;
  volatile uint32_t apply_calls;
  volatile uint32_t cordic_fallbacks;
} cal_debug_t;
cal_debug_t cal_debug;

/* Microphone coordinates from the 2026-09-02 pick-and-place file, ordered
 * by the FPGA demodulator slots:
 *   slot 0 DATA0 rising (U181, SELECT high), slot 1 DATA0 falling (U180),
 *   slot 2 DATA1 rising (U204, SELECT high), slot 3 DATA1 falling (U182). */
static const float cal_mic_x_mm[CAL_MICS] = { 25.000f, -50.000f,   0.000f, 25.000f};
static const float cal_mic_y_mm[CAL_MICS] = {-43.301f,   0.000f,   0.000f, 43.301f};

static const uint8_t cal_pair_j[6] = {0u, 0u, 0u, 1u, 1u, 2u};
static const uint8_t cal_pair_k[6] = {1u, 2u, 3u, 2u, 3u, 3u};

static float cal_chan_x_mm[CAL_CHANNELS];
static float cal_chan_y_mm[CAL_CHANNELS];
static float cal_path[CAL_CHANNELS][CAL_MICS];
static float cal_zre[CAL_MICS][CAL_CHANNELS];
static float cal_zim[CAL_MICS][CAL_CHANNELS];
static float cal_a_re[CAL_CHANNELS];
static float cal_a_im[CAL_CHANNELS];
static float cal_phi[CAL_CHANNELS];
static float cal_fit_deg[CAL_CHANNELS];
static float cal_rho_re[CAL_MICS];
static float cal_rho_im[CAL_MICS];
/* Common-mode nuisance per microphone.  The CS reconstruction may see
 * z_true + alpha (alpha = residual LC ring picked up by the gate) or, with
 * an exactly balanced code, z_true - mean(z_true).  The rank-1 model is
 * extended with this unknown per-microphone complex offset so the fit can
 * absorb it and still recover the relative channel phases. */
static float cal_mu_re[CAL_MICS];
static float cal_mu_im[CAL_MICS];
static float cal_survey_power[CAL_SURVEY_GATES];
static float cal_survey_noise[CAL_SURVEY_GATES];
static uint8_t cal_corr_a[CAL_CHANNELS];
static uint8_t cal_corr_b[CAL_CHANNELS];

/* Work area shared between the survey, the compressed-sensing acquisition and
 * the solver.  The union deliberately reuses the storage that the previous
 * firmware generation used for its fixed 84-pattern raw capture, so the new
 * algorithm does not increase the static RAM footprint. */
typedef struct {
  float b_re[CAL_MICS][CAL_CHANNELS];
  float b_im[CAL_MICS][CAL_CHANNELS];
  float solve[8u][CAL_CHANNELS];
} cal_cs_work_t;

static cal_cs_work_t cal_cs_work;

/* Named solver / scratch slices inside cal_cs_work.solve. */
#define CAL_SCR_BASE (&cal_cs_work.solve[0][0])
#define CAL_SCR(k)   (CAL_SCR_BASE + (size_t)(k) * CAL_CHANNELS)

static uint32_t cal_frame_sequence;
static uint16_t cal_block_expected;
static float cal_k_wave_mm;      /* 2*pi*f/c in rad/mm */
static uint16_t cal_cs_patterns;
static float cal_noise_power;    /* mean |I+jQ|^2 of a silent gate sample */
static float cal_signal_power;   /* same, averaged over all projections */
static float cal_cs_ridge;       /* Tikhonov term added by cal_cs_apply */
static float cal_cs_lmax;
static uint32_t cal_active_burst_us = CAL_SURVEY_BURST_US;
static uint32_t cal_solve_deadline_ms;
static us_cal_progress_cb_t cal_stage_progress;
static void *cal_stage_progress_ctx;
static uint8_t cal_fit_progress_base = 90u;

/* Called by long solver loops to keep the UI/wdt alive and to refresh the
 * progress display.  The callback itself is cheap (it only writes GUI state),
 * but osDelay(1) also gives the lower-priority UI task a scheduling slot. */
static void cal_stage_update(uint8_t state, uint8_t progress)
{
  if (cal_stage_progress != NULL) cal_stage_progress(state, progress, cal_stage_progress_ctx);
  osDelay(1u);
}

static int cal_solve_expired(void)
{
  if (cal_solve_deadline_ms == 0u) return 0;
  return (int32_t)(cal_solve_deadline_ms - HAL_GetTick()) <= 0;
}

/* --------------------------------------------------------------------------
 * Hardware-accelerated math wrappers.  The CORDIC batch helpers return a
 * negative value if the peripheral self-test fails; every wrapper keeps a
 * scalar libm fallback so a configuration problem can never hang the
 * calibration state machine.
 * -------------------------------------------------------------------------- */
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

static void cal_phase_batch(const float *real, const float *imag, float *phase_rad, uint32_t count)
{
  uint32_t i;
  if (umh_cordic_phase_batch(real, imag, phase_rad, count) == 0) return;
  ++cal_debug.cordic_fallbacks;
  for (i = 0u; i < count; ++i) phase_rad[i] = atan2f(imag[i], real[i]);
}

static void cal_sqrt_batch(const float *values, float *roots, uint32_t count)
{
  uint32_t i;
  if (umh_cordic_sqrt_batch(values, roots, count) == 0) return;
  ++cal_debug.cordic_fallbacks;
  for (i = 0u; i < count; ++i) roots[i] = sqrtf(values[i]);
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

/* --------------------------------------------------------------------------
 * Deterministic Bernoulli projection matrices.
 *
 *  - cal_cs_row() is the raw i.i.d. random row used by the compressed-sensing
 *    acquisition.  It is NOT balanced, so the 84 columns span the complete
 *    channel space (rank 84) and A = C^H C is well conditioned.
 *  - cal_balanced_row() forces exactly 42 (+1) / 42 (-1) entries.  The survey
 *    uses it because a zero-mean row cancels the common-mode LC ring-down and
 *    direct coupling in the echo-power profile; the rank deficiency of the
 *    balanced ensemble is irrelevant there.  The common-mode component that
 *    the balanced survey cannot see is handled by the mu-nuisance term in the
 *    rank-1 fit for the CS data.
 *
 * Both generators are stateless and bit-exact, so acquisition and
 * reconstruction regenerate the identical matrix without storing it.
 * -------------------------------------------------------------------------- */
#define CAL_CS_SEED 0x243F6A8885A308D3ull

static uint32_t cal_mix32(uint64_t *state)
{
  uint64_t z = (*state += 0x9E3779B97F4A7C15ull);
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
  return (uint32_t)(z ^ (z >> 31));
}

static void cal_cs_raw_bits(uint16_t pattern, uint32_t bits[3])
{
  uint64_t state = CAL_CS_SEED ^ ((uint64_t)(pattern + 1u) * 0xD1B54A32D192ED03ull);
  bits[0] = cal_mix32(&state);
  bits[1] = cal_mix32(&state);
  bits[2] = cal_mix32(&state) & 0x000FFFFFu;   /* only channels 64..83 */
}

static void cal_cs_row(uint16_t pattern, uint32_t bits[3])
{
  cal_cs_raw_bits(pattern, bits);
}

static void cal_balanced_row(uint16_t pattern, uint32_t bits[3])
{
  int ones, need, i;
  cal_cs_raw_bits(pattern, bits);
  ones = __builtin_popcount(bits[0]) + __builtin_popcount(bits[1]) + __builtin_popcount(bits[2]);
  need = 42 - ones;
  if (need > 0) {
    for (i = 0; i < (int)CAL_CHANNELS && need > 0; ++i) {
      if (((bits[i >> 5] >> (uint32_t)(i & 31)) & 1u) == 0u) {
        bits[i >> 5] |= (1u << (uint32_t)(i & 31));
        --need;
      }
    }
  } else if (need < 0) {
    need = -need;
    for (i = 0; i < (int)CAL_CHANNELS && need > 0; ++i) {
      if (((bits[i >> 5] >> (uint32_t)(i & 31)) & 1u) != 0u) {
        bits[i >> 5] &= ~(1u << (uint32_t)(i & 31));
        --need;
      }
    }
  }
}

static __attribute__((always_inline)) inline int cal_cs_bit(const uint32_t bits[3], uint8_t channel)
{
  uint32_t word = bits[channel >> 5u];
  return (int)((word >> (uint32_t)(channel & 31u)) & 1u);
}

/* --------------------------------------------------------------------------
 * FPGA link helpers.
 * -------------------------------------------------------------------------- */
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

/* Accurate microsecond delay for the LC burst / ring-down timing.  The
 * FreeRTOS tick is too coarse (1 ms), so use the DWT cycle counter when it is
 * available and fall back to the RTOS tick otherwise. */
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
  while ((DWT->CYCCNT - start) < wait) { }
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
  if (cal_wait_credit(link, 300u) != 0) return -2;
  return fpga_link_submit(link, &frame);
}

static int cal_submit_silence(fpga_link_t *link)
{
  return cal_submit_bits(link, NULL, NULL, 0u);
}

static uint32_t cal_block_timeout_ms(uint16_t start, uint16_t step, uint8_t count, uint8_t width)
{
  uint64_t samples = (uint64_t)start + (uint64_t)(count - 1u) * (uint64_t)step + (uint64_t)width;
  return (uint32_t)((samples * 25ull) / 1000ull) + 200u;
}

/* --------------------------------------------------------------------------
 * Compressed-sensing operator A x = C^H (C x).
 * The random rows are regenerated on the fly, so only O(84) state is kept
 * instead of M x 84 measurement-matrix storage.
 * -------------------------------------------------------------------------- */
static __attribute__((optimize("O3"))) void cal_cs_apply(const float *xr, const float *xi, float *yr, float *yi, uint16_t patterns)
{
  uint32_t bits[3];
  ++cal_debug.apply_calls;
  uint16_t p, i;
  memset(yr, 0, CAL_CHANNELS * sizeof(float));
  memset(yi, 0, CAL_CHANNELS * sizeof(float));
  for (p = 0u; p < patterns; ++p) {
    float vr = 0.0f;
    float vi = 0.0f;
    cal_cs_row(p, bits);
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      if (cal_cs_bit(bits, (uint8_t)i) != 0) { vr -= xr[i]; vi -= xi[i]; }
      else { vr += xr[i]; vi += xi[i]; }
    }
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      if (cal_cs_bit(bits, (uint8_t)i) != 0) { yr[i] -= vr; yi[i] -= vi; }
      else { yr[i] += vr; yi[i] += vi; }
    }
  }
  if (cal_cs_ridge != 0.0f) {
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      yr[i] += cal_cs_ridge * xr[i];
      yi[i] += cal_cs_ridge * xi[i];
    }
  }
}

static __attribute__((optimize("O3"))) float cal_cs_lmax_est(uint16_t patterns)
{
  float *ur = CAL_SCR(0);
  float *ui = CAL_SCR(1);
  float *vr = CAL_SCR(2);
  float *vi = CAL_SCR(3);
  float norm = 0.0f;
  uint8_t k;
  uint16_t i;
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    ur[i] = ((i & 1u) != 0u) ? 1.0f : -1.0f;
    ui[i] = ((i & 2u) != 0u) ? 0.5f : -0.5f;
  }
  for (i = 0u; i < CAL_CHANNELS; ++i) norm += ur[i] * ur[i] + ui[i] * ui[i];
  norm = sqrtf(norm);
  if (norm < 1.0e-12f) return 0.0f;
  for (i = 0u; i < CAL_CHANNELS; ++i) { ur[i] /= norm; ui[i] /= norm; }
  for (k = 0u; k < CAL_CS_POWER_ITERS; ++k) {
    float rn = 0.0f;
    cal_cs_apply(ur, ui, vr, vi, patterns);
    for (i = 0u; i < CAL_CHANNELS; ++i) rn += vr[i] * vr[i] + vi[i] * vi[i];
    rn = sqrtf(rn);
    if (rn < 1.0e-12f) return 0.0f;
    norm = rn;
    for (i = 0u; i < CAL_CHANNELS; ++i) { ur[i] = vr[i] / rn; ui[i] = vi[i] / rn; }
  }
  return norm;
}

/* Conjugate-gradient solve of (A + ridge I) x = b for one microphone.
 * b, x are complex vectors of length 84. Returns 0 on convergence. */
static __attribute__((optimize("O3"))) int cal_cs_cg_solve(const float *br, const float *bi, float *xr, float *xi,
                           uint16_t patterns)
{
  float *rr = CAL_SCR(2);
  float *ri = CAL_SCR(3);
  float *pr = CAL_SCR(4);
  float *pi = CAL_SCR(5);
  float *ap_r = CAL_SCR(6);
  float *ap_i = CAL_SCR(7);
  float rsold = 0.0f;
  float rs0;
  uint16_t i;
  uint8_t iter;
  memset(xr, 0, CAL_CHANNELS * sizeof(float));
  memset(xi, 0, CAL_CHANNELS * sizeof(float));
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    rr[i] = br[i]; ri[i] = bi[i];
    pr[i] = rr[i]; pi[i] = ri[i];
    rsold += rr[i] * rr[i] + ri[i] * ri[i];
  }
  rs0 = rsold;
  if (rs0 < 1.0e-18f) return -1;
  for (iter = 0u; iter < CAL_CS_CG_ITERS; ++iter) {
    float denom = 0.0f, alpha, rsnew = 0.0f, beta;
    cal_cs_apply(pr, pi, ap_r, ap_i, patterns);
    for (i = 0u; i < CAL_CHANNELS; ++i)
      denom += pr[i] * ap_r[i] + pi[i] * ap_i[i];
    if (denom <= 1.0e-18f) break;
    alpha = rsold / denom;
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      xr[i] += alpha * pr[i];
      xi[i] += alpha * pi[i];
      rr[i] -= alpha * ap_r[i];
      ri[i] -= alpha * ap_i[i];
    }
    for (i = 0u; i < CAL_CHANNELS; ++i) rsnew += rr[i] * rr[i] + ri[i] * ri[i];
    if (rsnew <= rs0 * CAL_CS_CG_TOL * CAL_CS_CG_TOL) return 0;
    beta = rsnew / rsold;
    rsold = rsnew;
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      pr[i] = rr[i] + beta * pr[i];
      pi[i] = ri[i] + beta * pi[i];
    }
  }
  return (rsold <= rs0 * CAL_CS_CG_TOL * CAL_CS_CG_TOL) ? 0 : 1;
}

/* FISTA / accelerated proximal gradient for the under-sampled case.
 * Solves min 0.5 ||C x - y||^2 + lambda ||x||_1 via the normal equations. */
static __attribute__((optimize("O3"))) int cal_cs_fista_solve(const float *br, const float *bi, float *xr, float *xi,
                              uint16_t patterns, float lmax)
{
  float *zr = CAL_SCR(2);
  float *zi = CAL_SCR(3);
  float *gr = CAL_SCR(4);
  float *gi = CAL_SCR(5);
  float *nr = CAL_SCR(6);
  float *ni = CAL_SCR(7);
  float sigma = sqrtf(cal_noise_power * 0.5f);
  float eta;
  float threshold;
  float t = 1.0f;
  uint8_t iter;
  uint16_t i;
  if (lmax <= 1.0e-9f) return -1;
  eta = 1.0f / lmax;
  threshold = eta * CAL_CS_L1_SIGMA * sigma * sqrtf((float)patterns);
  memset(xr, 0, CAL_CHANNELS * sizeof(float));
  memset(xi, 0, CAL_CHANNELS * sizeof(float));
  memcpy(zr, xr, CAL_CHANNELS * sizeof(float));
  memcpy(zi, xi, CAL_CHANNELS * sizeof(float));
  for (iter = 0u; iter < CAL_CS_FISTA_ITERS; ++iter) {
    float tnew, beta;
    cal_cs_apply(zr, zi, gr, gi, patterns);
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float vr = zr[i] - eta * (gr[i] - br[i]);
      float vi = zi[i] - eta * (gi[i] - bi[i]);
      float mag = sqrtf(vr * vr + vi * vi);
      if (mag <= threshold) { nr[i] = 0.0f; ni[i] = 0.0f; }
      else {
        float scale = (mag - threshold) / mag;
        nr[i] = vr * scale;
        ni[i] = vi * scale;
      }
    }
    tnew = 0.5f * (1.0f + sqrtf(1.0f + 4.0f * t * t));
    beta = (t - 1.0f) / tnew;
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      zr[i] = nr[i] + beta * (nr[i] - xr[i]);
      zi[i] = ni[i] + beta * (ni[i] - xi[i]);
    }
    memcpy(xr, nr, CAL_CHANNELS * sizeof(float));
    memcpy(xi, ni, CAL_CHANNELS * sizeof(float));
    t = tnew;
  }
  return 0;
}

static int cal_cs_reconstruct(void)
{
  uint8_t m;
  float lmax;
  cal_cs_ridge = (cal_cs_patterns < 128u) ? 0.01f * (float)cal_cs_patterns : 0.0f;
  if (cal_cs_patterns >= CAL_CHANNELS) {
    /* Over-complete: CG solves the normal equations directly and FISTA
     * never runs, so skip the six power iterations that only estimate
     * lmax for the under-sampled path. */
    lmax = 1.0f;
    cal_cs_lmax = lmax;
  } else {
    lmax = cal_cs_lmax_est(cal_cs_patterns);
    if (lmax <= 1.0e-9f) return -1;
    cal_cs_lmax = lmax;
  }
  for (m = 0u; m < CAL_MICS; ++m) {
    int rc;
    if (cal_solve_expired() != 0) return -3;
    if (cal_cs_patterns >= CAL_CHANNELS)
      rc = cal_cs_cg_solve(cal_cs_work.b_re[m], cal_cs_work.b_im[m],
                           cal_zre[m], cal_zim[m], cal_cs_patterns);
    else
      rc = cal_cs_fista_solve(cal_cs_work.b_re[m], cal_cs_work.b_im[m],
                              cal_zre[m], cal_zim[m], cal_cs_patterns, lmax);
    if (rc < 0) return -2;
    cal_stage_update(US_CAL_SOLVE, (uint8_t)(77u + m * 3u));
  }
  return 0;
}

/* --------------------------------------------------------------------------
 * Survey: locate the echo.
 * -------------------------------------------------------------------------- */
static int cal_survey_scan(fpga_link_t *link, uint16_t step, uint16_t start,
                           uint8_t width)
{
  uint8_t p, g, m, b;
  uint32_t timeout = cal_block_timeout_ms(start, step, CAL_SURVEY_GATES, width);
  fpga_mic_gate_wire_t wire;
  mic_capture_gate_t sample;
  if (mic_capture_configure(link, CAL_SURVEY_GATES, start, step, width) != 0) return -1;
  cal_block_expected = 0u;
  osDelay(1u);
  memset(cal_survey_power, 0, sizeof(cal_survey_power));
  memset(cal_survey_noise, 0, sizeof(cal_survey_noise));
  for (b = 0u; b < CAL_SURVEY_NOISE_BLOCKS; ++b) {
    uint16_t expected;
    if (cal_submit_silence(link) != 0) return -2;
    expected = (uint16_t)(cal_block_expected + 1u);
    if (mic_capture_wait_block(link, expected, timeout, &wire) != 0) return -3;
    cal_block_expected = expected;
    for (g = 0u; g < CAL_SURVEY_GATES; ++g) {
      if (mic_capture_read_gate(link, g, &sample, NULL) != 0) return -4;
      for (m = 0u; m < CAL_MICS; ++m) {
        float iv = (float)sample.i[m];
        float qv = (float)sample.q[m];
        cal_survey_noise[g] += iv * iv + qv * qv;
      }
    }
  }
  for (g = 0u; g < CAL_SURVEY_GATES; ++g)
    cal_survey_noise[g] /= (float)CAL_SURVEY_NOISE_BLOCKS;

  for (p = 0u; p < CAL_SURVEY_PATTERNS; ++p) {
    uint32_t bits[3];
    uint16_t expected;
    cal_balanced_row(p, bits);
    if (cal_submit_bits(link, bits, NULL, 128u) != 0) return -5;
    cal_delay_us(CAL_SURVEY_BURST_US);
    if (fpga_link_safe_stop(link) != 0) return -6;
    expected = (uint16_t)(cal_block_expected + 1u);
    if (mic_capture_wait_block(link, expected, timeout, &wire) != 0) return -7;
    cal_block_expected = expected;
    for (g = 0u; g < CAL_SURVEY_GATES; ++g) {
      if (mic_capture_read_gate(link, g, &sample, NULL) != 0) return -8;
      for (m = 0u; m < CAL_MICS; ++m) {
        float iv = (float)sample.i[m];
        float qv = (float)sample.q[m];
        cal_survey_power[g] += iv * iv + qv * qv;
      }
    }
    cal_delay_us(CAL_CS_SETTLE_US);
  }
  return 0;
}

static uint16_t cal_survey_find_peak(const float *power, const float *noise, uint8_t patterns,
                                     uint16_t step, uint8_t width, uint32_t skip_samples,
                                     float *snr_out, uint8_t *found_out)
{
  float net[CAL_SURVEY_GATES];
  float smooth[CAL_SURVEY_GATES];
  float peak = 0.0f, noise_ref, snr, threshold;
  uint8_t g, peak_idx = 0u, skip_gate, edge;
  if (found_out != NULL) *found_out = 0u;
  for (g = 0u; g < CAL_SURVEY_GATES; ++g) {
    float expected = (float)patterns * noise[g];
    net[g] = power[g] - expected;
    if (net[g] < 0.0f) net[g] = 0.0f;
  }
  for (g = 0u; g < CAL_SURVEY_GATES; ++g) {
    float prev = net[g > 0u ? g - 1u : 0u];
    float next = net[g + 1u < CAL_SURVEY_GATES ? g + 1u : g];
    smooth[g] = 0.25f * prev + 0.5f * net[g] + 0.25f * next;
  }
  skip_gate = (uint8_t)((skip_samples + (uint32_t)step - 1u) / (uint32_t)step);
  if (skip_gate >= CAL_SURVEY_GATES) skip_gate = CAL_SURVEY_GATES - 1u;
  for (g = skip_gate; g < CAL_SURVEY_GATES; ++g) {
    if (smooth[g] > peak) { peak = smooth[g]; peak_idx = g; }
  }
  noise_ref = (float)patterns * noise[peak_idx] + 1.0f;
  snr = peak / noise_ref;
  if (snr_out != NULL) *snr_out = snr;
  if (peak < 1.0f || snr < CAL_SURVEY_MIN_SNR) return 0u;
  threshold = 0.25f * peak;
  edge = skip_gate;
  for (g = skip_gate; g <= peak_idx; ++g) {
    if (smooth[g] >= threshold) { edge = g; break; }
  }
  if (found_out != NULL) *found_out = 1u;
  return (uint16_t)((uint32_t)edge * (uint32_t)step + (uint32_t)width / 2u);
}

/* FMAC matched filter on the accumulated, ring-cancelled echo-power profile.
 * A boxcar template followed by a half-amplitude leading-edge detector is
 * more robust than a raw peak search when the plane is small and the echo is
 * weak or asymmetric.  The profile is already a sufficient statistic built
 * from balanced random projections; the LC common-mode ring-down cancels in
 * every projection before the power accumulation. */
static uint16_t cal_fmac_refine(const float *power, const float *noise, uint8_t patterns,
                                uint16_t step, uint32_t skip_samples, uint8_t *found_out)
{
  int16_t profile[CAL_SURVEY_GATES];
  int16_t filtered[CAL_SURVEY_GATES];
  int16_t taps[CAL_FMAC_TAPS];
  float net[CAL_SURVEY_GATES];
  float max_net = 0.0f;
  float noise_sum = 0.0f;
  float noise_avg;
  int32_t maxval = 0;
  uint8_t g, k, skip_gate, edge;
  if (found_out != NULL) *found_out = 0u;
  for (g = 0u; g < CAL_SURVEY_GATES; ++g) {
    float value = power[g] - (float)patterns * noise[g];
    if (value < 0.0f) value = 0.0f;
    net[g] = value;
    noise_sum += (float)patterns * noise[g];
    if (value > max_net) max_net = value;
  }
  noise_avg = noise_sum / (float)CAL_SURVEY_GATES;
  /* Reject a spurious edge when the profile is only noise. */
  if (max_net <= 1.0e-9f || max_net < 3.0f * noise_avg) return 0u;
  for (g = 0u; g < CAL_SURVEY_GATES; ++g) {
    float scaled = net[g] * (30000.0f / max_net);
    if (scaled > 32767.0f) scaled = 32767.0f;
    profile[g] = (int16_t)scaled;
  }
  for (k = 0u; k < CAL_FMAC_TAPS; ++k) taps[k] = 1024;
  if (umh_fmac_fir_q15(taps, CAL_FMAC_TAPS, profile, CAL_SURVEY_GATES, filtered) != 0) {
    /* CPU fallback with the identical FIR equation. */
    for (g = 0u; g < CAL_SURVEY_GATES; ++g) {
      int32_t acc = 0;
      for (k = 0u; k < CAL_FMAC_TAPS; ++k) {
        int32_t idx = (int32_t)g - (int32_t)k;
        if (idx >= 0) acc += (int32_t)profile[idx] * 1024;
      }
      filtered[g] = (int16_t)(acc >> 15);
    }
  }
  skip_gate = (uint8_t)((skip_samples + (uint32_t)step - 1u) / (uint32_t)step);
  if (skip_gate >= CAL_SURVEY_GATES) skip_gate = CAL_SURVEY_GATES - 1u;
  for (g = skip_gate; g < CAL_SURVEY_GATES; ++g) {
    if ((int32_t)filtered[g] > maxval) maxval = filtered[g];
  }
  if (maxval <= 8) return 0u;
  edge = skip_gate;
  while (edge < CAL_SURVEY_GATES && (int32_t)filtered[edge] < (maxval / 2)) ++edge;
  if (edge >= CAL_SURVEY_GATES) return 0u;
  {
    int32_t onset = (int32_t)edge - (int32_t)CAL_FMAC_TAPS / 2;
    if (onset < 0) onset = 0;
    if (found_out != NULL) *found_out = 1u;
    return (uint16_t)((uint32_t)onset * (uint32_t)step + (uint32_t)step / 2u);
  }
}
static void cal_pick_gate(uint32_t arrival_samples, uint16_t *start_out, uint8_t *width_out)
{
  uint32_t skip = (cal_active_burst_us + CAL_CS_RING_GUARD_US + 24u) / 25u;
  /* Sampling 200 us after the leading edge keeps the echo inside the gate
   * while giving the LC ring an extra exponential decay before acquisition;
   * the 2048-pattern coherent projection makes up for the small SNR loss. */
  int32_t start = (int32_t)arrival_samples - (int32_t)CAL_CS_GATE_WIDTH / 2
                  + (int32_t)CAL_CS_GATE_LATE_SAMPLES;
  if (start < (int32_t)skip) start = (int32_t)skip;
  if (start > 65000) start = 65000;
  *start_out = (uint16_t)start;
  *width_out = CAL_CS_GATE_WIDTH;
}

static int cal_cs_measure_noise(fpga_link_t *link, uint16_t start, uint8_t width, float *noise_out)
{
  uint8_t b, m;
  uint16_t expected;
  fpga_mic_gate_wire_t wire;
  float sum = 0.0f;
  uint32_t timeout = ((uint32_t)start + (uint32_t)width) * 25u / 1000u + 100u;
  if (mic_capture_configure(link, 1u, start, width, width) != 0) return -1;
  cal_block_expected = 0u;
  osDelay(1u);
  for (b = 0u; b < CAL_CS_NOISE_BLOCKS; ++b) {
    if (cal_submit_silence(link) != 0) return -2;
    expected = (uint16_t)(cal_block_expected + 1u);
    if (mic_capture_wait_block(link, expected, timeout, &wire) != 0) return -3;
    cal_block_expected = expected;
    for (m = 0u; m < CAL_MICS; ++m) {
      float iv = (float)wire.i[m];
      float qv = (float)wire.q[m];
      sum += iv * iv + qv * qv;
    }
    cal_delay_us(200u);
  }
  *noise_out = sum / ((float)CAL_CS_NOISE_BLOCKS * (float)CAL_MICS);
  return 0;
}

/* --------------------------------------------------------------------------
 * Compressed-sensing acquisition.
 * -------------------------------------------------------------------------- */
static int cal_cs_acquire(fpga_link_t *link, uint16_t start, uint8_t width,
                          uint32_t burst_us, us_cal_progress_cb_t cb, void *context)
{
  uint32_t budget_start = HAL_GetTick();
  uint32_t timeout = ((uint32_t)start + (uint32_t)width) * 25u / 1000u + 100u;
  uint16_t p, i;
  uint8_t m;
  float sig = 0.0f;
  if (burst_us < CAL_CS_BURST_MIN_US) burst_us = CAL_CS_BURST_MIN_US;
  if (burst_us > CAL_CS_BURST_MAX_US) burst_us = CAL_CS_BURST_MAX_US;

  if (mic_capture_configure(link, 1u, start, width, width) != 0) return -1;
  cal_block_expected = 0u;
  osDelay(1u);
  memset(cal_cs_work.b_re, 0, sizeof(cal_cs_work.b_re));
  memset(cal_cs_work.b_im, 0, sizeof(cal_cs_work.b_im));
  cal_cs_patterns = 0u;

  for (p = 0u; p < CAL_CS_MAX_PATTERNS; ++p) {
    uint32_t bits[3];
    uint16_t expected;
    fpga_mic_gate_wire_t wire;
    float y_re[CAL_MICS];
    float y_im[CAL_MICS];
    if (p >= CAL_CS_MIN_PATTERNS &&
        (HAL_GetTick() - budget_start) >= CAL_CS_TIME_BUDGET_MS)
      break;
    if (cal_solve_expired()) return -6;    /* the whole run is over budget  */
    cal_cs_row(p, bits);
    if (cal_submit_bits(link, bits, NULL, 128u) != 0) return -2;
    cal_delay_us(burst_us);
    if (fpga_link_safe_stop(link) != 0) return -3;
    expected = (uint16_t)(cal_block_expected + 1u);
    if (mic_capture_wait_block(link, expected, timeout, &wire) != 0) return -4;
    cal_block_expected = expected;

    for (m = 0u; m < CAL_MICS; ++m) {
      y_re[m] = (float)wire.i[m];
      y_im[m] = (float)wire.q[m];
      sig += y_re[m] * y_re[m] + y_im[m] * y_im[m];
    }
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float sign = (cal_cs_bit(bits, (uint8_t)i) != 0) ? -1.0f : 1.0f;
      for (m = 0u; m < CAL_MICS; ++m) {
        cal_cs_work.b_re[m][i] += sign * y_re[m];
        cal_cs_work.b_im[m][i] += sign * y_im[m];
      }
    }
    cal_cs_patterns = (uint16_t)(p + 1u);
    cal_delay_us(CAL_CS_SETTLE_US);
    if ((p & 15u) == 0u) {
      uint32_t elapsed = HAL_GetTick() - budget_start;
      uint32_t pr = 12u + (elapsed * 62u) / CAL_CS_TIME_BUDGET_MS;
      if (pr > 74u) pr = 74u;
      cal_report(cb, context, US_CAL_MEASURE, (uint8_t)pr);
      osDelay(1u);
    }
  }
  if (cal_cs_patterns < CAL_CS_MIN_PATTERNS) return -5;
  cal_signal_power = sig / ((float)cal_cs_patterns * (float)CAL_MICS);
  cal_report(cb, context, US_CAL_MEASURE, 75u);
  osDelay(1u);
  return 0;
}
/* --------------------------------------------------------------------------
 * Wall-image phase model and rank-1 channel fit.
 * -------------------------------------------------------------------------- */
static __attribute__((optimize("O3"))) void cal_paths(float D_mm, float tilt_x_deg, float tilt_y_deg)
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
      CAL_SCR_BASE[i * CAL_MICS + mic] = dx * dx + dy * dy + dz * dz;
    }
  }
  cal_sqrt_batch(CAL_SCR_BASE, &cal_path[0][0], CAL_CHANNELS * CAL_MICS);
}

#define CAL_POSE_RANK_ITERS 3u

/* --------------------------------------------------------------------------
 * Wall-image model cache and rank-1 fit.
 *
 * For a fixed (D, tilt_x, tilt_y), cal_paths() provides the geometric phase
 * of every (channel, microphone) path.  The old solver recomputed sin/cos
 * inside every rank-1 iteration and every partial update, which generated
 * thousands of CORDIC transactions per pose and dominated the solve time.
 * The cache below stores the complex rotation exp(-j*phi) instead: two
 * 84-point CORDIC sincos calls per microphone fill a now-free scratch area,
 * and the rank-1 iterations then use only four multiplies per element.
 * -------------------------------------------------------------------------- */
static __attribute__((optimize("O3"))) void cal_model_phasor_cache(void)
{
  uint8_t mic, i;
  for (mic = 0u; mic < CAL_MICS; ++mic) {
    float *ang = cal_fit_deg;
    float *sn = cal_a_re;
    float *cs = cal_a_im;
    float *cr = CAL_SCR(mic);
    float *ci = CAL_SCR(4u + mic);
    for (i = 0u; i < CAL_CHANNELS; ++i)
      ang[i] = cal_k_wave_mm * cal_path[i][mic];
    cal_sincos_batch(ang, sn, cs, CAL_CHANNELS);
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      cr[i] = cs[i];
      ci[i] = -sn[i];
    }
  }
}

/* D(m,i) = (z(m,i) - mu_m) * exp(-j*phi_i,m).  The complex rotation is
 * cached in solve[0..3] (real) and solve[4..7] (imag). */
#define CAL_CACHE_D(mic_, i_, dr_, di_) do { \
    float cr_ = CAL_SCR(mic_)[i_]; \
    float ci_ = CAL_SCR(4u + (mic_))[i_]; \
    float zr_ = cal_zre[mic_][i_] - cal_mu_re[mic_]; \
    float zi_ = cal_zim[mic_][i_] - cal_mu_im[mic_]; \
    (dr_) = zr_ * cr_ - zi_ * ci_; \
    (di_) = zr_ * ci_ + zi_ * cr_; \
  } while (0)

/* Remove the current common-mode estimate from the reconstructed data.  The
 * model fit solves in the offset-corrected domain already; the data itself is
 * corrected so that the pose cost and later rank-1 re-fits see the same
 * offset-free signal. */
static void cal_subtract_mu(void)
{
  uint8_t mic, i;
  for (mic = 0u; mic < CAL_MICS; ++mic) {
    if (cal_mu_re[mic] == 0.0f && cal_mu_im[mic] == 0.0f) continue;
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      cal_zre[mic][i] -= cal_mu_re[mic];
      cal_zim[mic][i] -= cal_mu_im[mic];
    }
    cal_mu_re[mic] = 0.0f;
    cal_mu_im[mic] = 0.0f;
  }
}

/* --------------------------------------------------------------------------
 * Rank-1 residual pose search.
 *
 * After the common-mode offset has been removed, the residual of the rank-1
 * model a_i * rho_m * exp(j*k*path_i,m) is a sharp objective: the true wall
 * pose makes the model fit all four microphones simultaneously.  The pose
 * search uses the cheap unweighted rank-1 power iteration below; the final
 * weighted fit in cal_fit_arho() recovers the channel phases themselves.
 * -------------------------------------------------------------------------- */
static __attribute__((optimize("O3"))) float cal_pose_rank1_residual(float D_mm, float tilt_x, float tilt_y)
{
  uint8_t i, m, iter;
  float num = 0.0f, den = 0.0f;
  ++cal_debug.pose_evals;
  cal_paths(D_mm, tilt_x, tilt_y);
  cal_model_phasor_cache();
  for (m = 0u; m < CAL_MICS; ++m) {
    cal_rho_re[m] = 1.0f;
    cal_rho_im[m] = 0.0f;
  }
  for (iter = 0u; iter < CAL_POSE_RANK_ITERS; ++iter) {
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float sr = 0.0f, si = 0.0f, dd = 0.0f;
      for (m = 0u; m < CAL_MICS; ++m) {
        float dr, di;
        float rr = cal_rho_re[m], ri = cal_rho_im[m];
        CAL_CACHE_D(m, i, dr, di);
        sr += dr * rr + di * ri;
        si += di * rr - dr * ri;
        dd += rr * rr + ri * ri;
      }
      if (dd < 1.0e-12f) dd = 1.0e-12f;
      cal_a_re[i] = sr / dd;
      cal_a_im[i] = si / dd;
    }
    for (m = 0u; m < CAL_MICS; ++m) {
      float sr = 0.0f, si = 0.0f, dd = 0.0f;
      for (i = 0u; i < CAL_CHANNELS; ++i) {
        float dr, di;
        float ar = cal_a_re[i], ai = cal_a_im[i];
        CAL_CACHE_D(m, i, dr, di);
        sr += dr * ar + di * ai;
        si += di * ar - dr * ai;
        dd += ar * ar + ai * ai;
      }
      if (dd < 1.0e-12f) dd = 1.0e-12f;
      cal_rho_re[m] = sr / dd;
      cal_rho_im[m] = si / dd;
    }
  }
  for (m = 0u; m < CAL_MICS; ++m) {
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float dr, di, mr, mi, er, ei;
      CAL_CACHE_D(m, i, dr, di);
      mr = cal_a_re[i] * cal_rho_re[m] - cal_a_im[i] * cal_rho_im[m];
      mi = cal_a_re[i] * cal_rho_im[m] + cal_a_im[i] * cal_rho_re[m];
      er = dr - mr;
      ei = di - mi;
      num += er * er + ei * ei;
      den += dr * dr + di * di;
    }
  }
  return num / (den + 1.0e-12f);
}

static __attribute__((optimize("O3"))) void cal_pose_local_residual(float *D_m, float *tilt_x, float *tilt_y)
{
  float D = *D_m, tx = *tilt_x, ty = *tilt_y;
  float best = cal_pose_rank1_residual(D * 1000.0f, tx, ty);
  float step_t = 5.0f;
  float step_d = 0.03f;
  uint8_t phase;
  for (phase = 0u; phase < 4u; ++phase) {
    uint8_t pass = 0u;
    int improved = 1;
    while (improved != 0 && pass < CAL_POSE_LOCAL_PASSES && cal_solve_expired() == 0) {
      int k;
      improved = 0;
      ++pass;
      for (k = 0; k < 6; ++k) {
        float cd = D, cx = tx, cy = ty, r;
        if (k == 0) cd = D + step_d;
        else if (k == 1) cd = D - step_d;
        else if (k == 2) cx = tx + step_t;
        else if (k == 3) cx = tx - step_t;
        else if (k == 4) cy = ty + step_t;
        else cy = ty - step_t;
        if (cd < 0.12f || cd > 3.0f) continue;
        if (cx < -CAL_MAX_TILT || cx > CAL_MAX_TILT) continue;
        if (cy < -CAL_MAX_TILT || cy > CAL_MAX_TILT) continue;
        r = cal_pose_rank1_residual(cd * 1000.0f, cx, cy);
        if (r < best) { best = r; D = cd; tx = cx; ty = cy; improved = 1; }
      }
    }
    step_t *= 0.5f;
    step_d *= 0.5f;
    cal_stage_update(US_CAL_SOLVE, (uint8_t)(82u + phase * 2u));
  }
  *D_m = D;
  *tilt_x = tx;
  *tilt_y = ty;
}

static __attribute__((optimize("O3"))) void cal_pose_search_residual(float D0_m, float *D_m, float *tilt_x, float *tilt_y)
{
  static const float factors[5] = {0.80f, 0.90f, 1.00f, 1.10f, 1.20f};
  float best = 1.0e30f;
  float best_d = *D_m, best_x = *tilt_x, best_y = *tilt_y;
  uint8_t di;
  if (D0_m < 0.12f) D0_m = 0.30f;
  if (D0_m > 3.0f) D0_m = 3.0f;
  for (di = 0u; di < 5u; ++di) {
    int8_t ti, ui;
    float D = D0_m * factors[di];
    if (D < 0.12f) D = 0.12f;
    if (D > 3.0f) D = 3.0f;
    for (ti = -CAL_POSE_GRID_RANGE; ti <= CAL_POSE_GRID_RANGE; ++ti) {
      float tx = (float)ti * CAL_POSE_GRID_STEP_DEG;
      for (ui = -CAL_POSE_GRID_RANGE; ui <= CAL_POSE_GRID_RANGE; ++ui) {
        float ty = (float)ui * CAL_POSE_GRID_STEP_DEG;
        float r = cal_pose_rank1_residual(D * 1000.0f, tx, ty);
        if (r < best) { best = r; best_d = D; best_x = tx; best_y = ty; }
      }
    }
    cal_stage_update(US_CAL_SOLVE, (uint8_t)(80u + di * 2u));
    if (cal_solve_expired() != 0) break;
  }
  *D_m = best_d;
  *tilt_x = best_x;
  *tilt_y = best_y;
  cal_pose_local_residual(D_m, tilt_x, tilt_y);
}

static __attribute__((optimize("O3"))) void cal_fit_arho(float D_mm, float tilt_x, float tilt_y,
                         float *residual, float *consistency_deg)
{
  float *ar = &cal_path[0][0];               /* scratch: cache already built */
  float *ai = ar + CAL_CHANNELS;
  float *den_acc = ai + CAL_CHANNELS;
  uint8_t i, j, iter, round;
  float numerr = 0.0f, denz = 0.0f;
  cal_paths(D_mm, tilt_x, tilt_y);
  for (j = 0u; j < CAL_MICS; ++j) {
    cal_mu_re[j] = 0.0f;
    cal_mu_im[j] = 0.0f;
    cal_rho_re[j] = 1.0f;
    cal_rho_im[j] = 0.0f;
  }
  cal_model_phasor_cache();
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    cal_a_re[i] = 1.0f;
    cal_a_im[i] = 0.0f;
  }

  for (round = 0u; round < CAL_FIT_MU_ROUNDS; ++round) {
    for (iter = 0u; iter < CAL_FIT_RANK_ITERS; ++iter) {
      uint8_t mic;
      for (i = 0u; i < CAL_CHANNELS; ++i) {
        ar[i] = 0.0f;
        ai[i] = 0.0f;
        den_acc[i] = 0.0f;
      }
      /* a_i = sum_j w_ij d_ij conj(rho_j) / sum_j w_ij |rho_j|^2 */
      for (mic = 0u; mic < CAL_MICS; ++mic) {
        float rr = cal_rho_re[mic], ri = cal_rho_im[mic];
        float r2 = rr * rr + ri * ri;
        for (i = 0u; i < CAL_CHANNELS; ++i) {
          float dr, di, w;
          CAL_CACHE_D(mic, i, dr, di);
          w = sqrtf(dr * dr + di * di);
          ar[i] += w * (dr * rr + di * ri);
          ai[i] += w * (di * rr - dr * ri);
          den_acc[i] += w * r2;
        }
      }
      for (i = 0u; i < CAL_CHANNELS; ++i) {
        float den = den_acc[i];
        if (den < 1.0e-12f) den = 1.0e-12f;
        cal_a_re[i] = ar[i] / den;
        cal_a_im[i] = ai[i] / den;
      }
      /* rho_j = sum_i w_ij d_ij conj(a_i) / sum_i w_ij |a_i|^2 */
      for (mic = 0u; mic < CAL_MICS; ++mic) {
        float sr = 0.0f, si = 0.0f, sw = 0.0f;
        for (i = 0u; i < CAL_CHANNELS; ++i) {
          float dr, di, w, aa2, aar, aai;
          CAL_CACHE_D(mic, i, dr, di);
          w = sqrtf(dr * dr + di * di);
          aar = cal_a_re[i];
          aai = cal_a_im[i];
          aa2 = aar * aar + aai * aai;
          sr += w * (dr * aar + di * aai);
          si += w * (di * aar - dr * aai);
          sw += w * aa2;
        }
        if (sw < 1.0e-12f) sw = 1.0e-12f;
        cal_rho_re[mic] = sr / sw;
        cal_rho_im[mic] = si / sw;
      }
    }

    /* Absorb the common-mode component left in the data.  The update is
     * mu_m += mean_i( (d_i,m - a_i rho_m) * exp(+j phi_i,m) ).  The
     * rotation is available from the cache, so no extra CORDIC calls are
     * needed for the nuisance fit. */
    for (j = 0u; j < CAL_MICS; ++j) {
      float sumr = 0.0f, sumi = 0.0f;
      for (i = 0u; i < CAL_CHANNELS; ++i) {
        float dr, di, er, ei, mr, mi, cr, ci;
        CAL_CACHE_D(j, i, dr, di);
        mr = cal_a_re[i] * cal_rho_re[j] - cal_a_im[i] * cal_rho_im[j];
        mi = cal_a_re[i] * cal_rho_im[j] + cal_a_im[i] * cal_rho_re[j];
        er = dr - mr;
        ei = di - mi;
        cr = CAL_SCR(j)[i];
        ci = CAL_SCR(4u + j)[i];
        sumr += er * cr + ei * ci;
        sumi += ei * cr - er * ci;
      }
      cal_mu_re[j] += sumr / (float)CAL_CHANNELS;
      cal_mu_im[j] += sumi / (float)CAL_CHANNELS;
    }
    cal_stage_update(US_CAL_SOLVE, (uint8_t)(cal_fit_progress_base + round));
  }

  /* Residual of the weighted rank-1 model at the final mu. */
  for (j = 0u; j < CAL_MICS; ++j) {
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float dr, di, mr, mi, er, ei;
      CAL_CACHE_D(j, i, dr, di);
      mr = cal_a_re[i] * cal_rho_re[j] - cal_a_im[i] * cal_rho_im[j];
      mi = cal_a_re[i] * cal_rho_im[j] + cal_a_im[i] * cal_rho_re[j];
      er = dr - mr;
      ei = di - mi;
      numerr += er * er + ei * ei;
      denz += dr * dr + di * di;
    }
  }
  if (residual != NULL) *residual = numerr / (denz + 1.0e-12f);

  /* Publish a_i phase for the gauge fixer. */
  cal_phase_batch(cal_a_re, cal_a_im, cal_phi, CAL_CHANNELS);

  if (consistency_deg == NULL) return;
  {
    float pair_cos[6], pair_sin[6];
    float sum = 0.0f;
    uint8_t pair;
    for (pair = 0u; pair < 6u; ++pair) { pair_cos[pair] = 0.0f; pair_sin[pair] = 0.0f; }
    /* Cross-microphone phase difference consistency.  The phases are read
     * from the cached D matrix; cal_phi is left untouched because it holds
     * the reconstructed channel phase a_i. */
    for (pair = 0u; pair < 6u; ++pair) {
      uint8_t m0 = cal_pair_j[pair], m1 = cal_pair_k[pair];
      float *ph0 = cal_fit_deg;
      float *ph1 = cal_a_re;
      float *diff = cal_a_im;
      cal_phase_batch(CAL_SCR(m0), CAL_SCR(4u + m0), ph0, CAL_CHANNELS);
      cal_phase_batch(CAL_SCR(m1), CAL_SCR(4u + m1), ph1, CAL_CHANNELS);
      for (i = 0u; i < CAL_CHANNELS; ++i)
        diff[i] = cal_wrap_pi(ph0[i] - ph1[i]);
      cal_sincos_batch(diff, ph1, ph0, CAL_CHANNELS);
      for (i = 0u; i < CAL_CHANNELS; ++i) {
        pair_cos[pair] += ph0[i];
        pair_sin[pair] += ph1[i];
      }
    }
    for (pair = 0u; pair < 6u; ++pair) {
      uint8_t m0 = cal_pair_j[pair], m1 = cal_pair_k[pair];
      float *ph0 = cal_fit_deg;
      float *ph1 = cal_a_re;
      float *diff = cal_a_im;
      float mean = cal_atan2_rad(pair_cos[pair], pair_sin[pair]);
      cal_phase_batch(CAL_SCR(m0), CAL_SCR(4u + m0), ph0, CAL_CHANNELS);
      cal_phase_batch(CAL_SCR(m1), CAL_SCR(4u + m1), ph1, CAL_CHANNELS);
      for (i = 0u; i < CAL_CHANNELS; ++i)
        diff[i] = cal_wrap_pi(ph0[i] - ph1[i] - mean);
      for (i = 0u; i < CAL_CHANNELS; ++i) sum += diff[i] * diff[i];
    }
    *consistency_deg = cal_deg(sqrtf(sum / (float)(6u * CAL_CHANNELS)));
  }
}
static float cal_gauge(const float *phi_rad, float *out_deg)
{
  uint8_t i;
  float sr = 0.0f, si = 0.0f, mean, rms = 0.0f;
  float *v = CAL_SCR(0);
  float *sn = CAL_SCR(1);
  float *cs = CAL_SCR(2);
  double xx = 0.0, xy = 0.0, yy = 0.0, x = 0.0, y = 0.0;
  double sv = 0.0, xv = 0.0, yv = 0.0;
  double a[3][4], det;
  cal_sincos_batch(phi_rad, sn, cs, CAL_CHANNELS);
  for (i = 0u; i < CAL_CHANNELS; ++i) { sr += cs[i]; si += sn[i]; }
  mean = cal_atan2_rad(sr, si);
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    double px = cal_chan_x_mm[i], py = cal_chan_y_mm[i];
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
    double fit = a[0][3] * cal_chan_x_mm[i] + a[1][3] * cal_chan_y_mm[i] + a[2][3];
    out_deg[i] = cal_deg(cal_wrap_pi((float)(v[i] - fit)));
  }
  return rms;
}

/* --------------------------------------------------------------------------
 * Array-gain verification (strict mode only).
 * -------------------------------------------------------------------------- */
#if !CAL_QUALITY_RELAXED
static int cal_measure_gate_power(fpga_link_t *link, const uint8_t *correction, float *power_out)
{
  fpga_mic_gate_wire_t wire;
  uint16_t expected;
  float power = 0.0f;
  uint8_t m;
  if (cal_submit_bits(link, NULL, correction, 128u) != 0) return -1;
  cal_delay_us(cal_active_burst_us);
  if (fpga_link_safe_stop(link) != 0) return -2;
  expected = (uint16_t)(cal_block_expected + 1u);
  if (mic_capture_wait_block(link, expected, 200u, &wire) != 0) return -3;
  cal_block_expected = expected;
  cal_delay_us(CAL_CS_SETTLE_US);
  for (m = 0u; m < CAL_MICS; ++m) {
    float iv = (float)wire.i[m];
    float qv = (float)wire.q[m];
    power += iv * iv + qv * qv;
  }
  *power_out = power;
  return 0;
}

static int cal_verify(fpga_link_t *link, uint8_t candidate_count,
                      uint8_t *chosen, float *gain_db)
{
  uint8_t negated[CAL_CHANNELS];
  uint8_t best_candidate = 0u, best_negated = 0u;
  float p0 = 0.0f, best_power = -1.0f;
  uint8_t c, i;
  if (cal_measure_gate_power(link, NULL, &p0) != 0) return -1;
  if (p0 < 1.0f || candidate_count == 0u) return -2;
  for (c = 0u; c < candidate_count; ++c) {
    const uint8_t *cand = (c == 0u) ? cal_corr_a : cal_corr_b;
    float power;
    if (cal_measure_gate_power(link, cand, &power) != 0) return -3;
    if (power > best_power) { best_power = power; best_candidate = c; best_negated = 0u; }
    for (i = 0u; i < CAL_CHANNELS; ++i) negated[i] = (uint8_t)(0u - cand[i]);
    if (cal_measure_gate_power(link, negated, &power) != 0) return -4;
    if (power > best_power) { best_power = power; best_candidate = c; best_negated = 1u; }
  }
  memcpy(chosen, (best_candidate == 0u) ? cal_corr_a : cal_corr_b, CAL_CHANNELS);
  if (best_negated != 0u) {
    for (i = 0u; i < CAL_CHANNELS; ++i) chosen[i] = (uint8_t)(0u - chosen[i]);
  }
  *gain_db = 10.0f * log10f((best_power + 1.0f) / (p0 + 1.0f));
  return 0;
}
#endif

/* --------------------------------------------------------------------------
 * Raw dump for offline diagnostics: interleaved float32 b vectors,
 * layout [mic][channel][I,Q].  It is the compressed-sensing sufficient
 * statistic, not the old per-pattern gate capture.
 * -------------------------------------------------------------------------- */
uint32_t us_calibration_raw_size(void)
{
  return (uint32_t)(sizeof(cal_cs_work.b_re) + sizeof(cal_cs_work.b_im));
}

int us_calibration_raw_read(uint32_t offset, uint8_t *out, uint16_t length)
{
  uint32_t size = us_calibration_raw_size();
  uint32_t block = (uint32_t)sizeof(cal_cs_work.b_re);
  uint16_t total;
  if (out == NULL || offset >= size) return -1;
  if ((uint32_t)length > (size - offset)) length = (uint16_t)(size - offset);
  total = length;
  while (length != 0u) {
    const uint8_t *src;
    uint32_t chunk;
    if (offset < block) {
      src = (const uint8_t *)&cal_cs_work.b_re + offset;
      chunk = block - offset;
    } else {
      src = (const uint8_t *)&cal_cs_work.b_im + (offset - block);
      chunk = size - offset;
    }
    if (chunk > (uint32_t)length) chunk = (uint32_t)length;
    memcpy(out, src, chunk);
    out += chunk;
    offset += chunk;
    length = (uint16_t)(length - (uint16_t)chunk);
  }
  return (int)total;
}

/* --------------------------------------------------------------------------
 * Main state machine.
 * -------------------------------------------------------------------------- */
int us_calibration_run(fpga_link_t *link, const umh_device_profile_t *profile,
                       us_cal_progress_cb_t progress, void *context,
                       umh_calibration_result_t *result)
{
  uint8_t i, m;
  uint16_t coarse_arrival = 0u, fine_start = 0u, fine_arrival = 0u, fmac_arrival = 0u;
  uint16_t final_start = 0u;
  uint8_t final_width = CAL_CS_GATE_WIDTH;
  uint32_t default_samples, skip_abs, skip_rel;
  float D0_search, D_m = CAL_DEFAULT_DISTANCE_MM / 1000.0f;
  float tilt_x = 0.0f, tilt_y = 0.0f;
  float residual = 1.0f, consistency = 999.0f, rms_pre, rms_post;
  float echo_ratio, mic_ratio = 999.0f;
  float mean_amp[CAL_MICS], max_amp = 0.0f;
  float c_m_s, c_mm_s, f_hz;
  uint8_t found = 0u, fmac_found = 0u, good_mics = 0u, round;
  int32_t fine_center;
  uint32_t dbg_t0, dbg_part;
  int rc;

  if (link == NULL || profile == NULL || result == NULL) return -1;
  memset(result, 0, sizeof(*result));
  result->fault = UMH_FAULT_NONE;
  dbg_t0 = HAL_GetTick();
  for (i = 0u; i < 8u; ++i) cal_debug.stage_ms[i] = 0u;
  cal_debug.stage = 1u;
  cal_debug.total_ms = 0u;
  cal_debug.patterns = 0u;
  cal_debug.pose_evals = 0u;
  cal_debug.apply_calls = 0u;
  cal_debug.cordic_fallbacks = 0u;
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    cal_chan_x_mm[i] = (float)profile->coordinates[i].x_um * 0.001f;
    cal_chan_y_mm[i] = (float)profile->coordinates[i].y_um * 0.001f;
  }
  c_m_s = (profile->sound_speed_um_per_s != 0u) ?
          ((float)profile->sound_speed_um_per_s * 1.0e-6f) : 343.0f;
  c_mm_s = c_m_s * 1000.0f;
  f_hz = (float)(profile->carrier_hz != 0u ? profile->carrier_hz : 40000u);
  cal_k_wave_mm = CAL_TWO_PI * f_hz / c_mm_s;
  cal_frame_sequence = 0u;
  cal_block_expected = 0u;
  cal_stage_progress = progress;
  cal_stage_progress_ctx = context;
  cal_solve_deadline_ms = 0u;
  cal_active_burst_us = CAL_SURVEY_BURST_US;
  cal_report(progress, context, US_CAL_WAIT, 0u);

  default_samples = (uint32_t)((2.0f * (CAL_DEFAULT_DISTANCE_MM / 1000.0f) / c_m_s) / 25.0e-6f + 0.5f);
  skip_abs = (CAL_SURVEY_BURST_US + CAL_CS_RING_GUARD_US + 24u) / 25u;

  /* ---- step 1: coarse and fine echo survey ---------------------------- */
  cal_report(progress, context, US_CAL_MEASURE, 1u);
  rc = cal_survey_scan(link, CAL_SURVEY_COARSE_STEP, 0u, CAL_SURVEY_WIDTH);
  if (rc != 0) {
    result->quality_flags |= CAL_Q_MEASURE;
    result->fault = UMH_FAULT_CAL_MIC_SILENT;
    goto fail;
  }
  coarse_arrival = cal_survey_find_peak(cal_survey_power, cal_survey_noise, CAL_SURVEY_PATTERNS,
                                        CAL_SURVEY_COARSE_STEP, CAL_SURVEY_WIDTH,
                                        skip_abs, NULL, &found);
  if (coarse_arrival == 0u) coarse_arrival = (uint16_t)default_samples;

  fine_center = (int32_t)coarse_arrival - 32 * (int32_t)CAL_SURVEY_FINE_STEP;
  if (fine_center < 0) fine_center = 0;
  if (fine_center > 40000) fine_center = 40000;
  fine_start = (uint16_t)fine_center;
  cal_report(progress, context, US_CAL_MEASURE, 5u);
  rc = cal_survey_scan(link, CAL_SURVEY_FINE_STEP, fine_start, CAL_SURVEY_WIDTH);
  if (rc != 0) {
    result->quality_flags |= CAL_Q_MEASURE;
    result->fault = UMH_FAULT_CAL_MIC_SILENT;
    goto fail;
  }
  skip_rel = ((uint32_t)skip_abs > (uint32_t)fine_start) ? ((uint32_t)skip_abs - (uint32_t)fine_start) : 0u;
  fine_arrival = cal_survey_find_peak(cal_survey_power, cal_survey_noise, CAL_SURVEY_PATTERNS,
                                      CAL_SURVEY_FINE_STEP, CAL_SURVEY_WIDTH,
                                      skip_rel, NULL, &found);
  fmac_arrival = cal_fmac_refine(cal_survey_power, cal_survey_noise, CAL_SURVEY_PATTERNS,
                                     CAL_SURVEY_FINE_STEP, skip_rel, &fmac_found);
  {
    uint16_t arrival_rel = 0u;
    uint32_t arrival_abs;
    uint32_t arrival_us, burst_us;
    if (fmac_found != 0u && found != 0u)
      arrival_rel = (uint16_t)(((uint32_t)fmac_arrival + (uint32_t)fine_arrival) / 2u);
    else if (fmac_found != 0u) {
      arrival_rel = fmac_arrival;
      found = 1u;   /* FMAC matched-filter detection counts as a valid echo */
    } else if (found != 0u)
      arrival_rel = fine_arrival;
    if (arrival_rel != 0u) arrival_abs = (uint32_t)fine_start + (uint32_t)arrival_rel;
    else { found = 0u; arrival_abs = default_samples; }

    /* Stop the drive before the first echo returns.  The survey gives the
     * absolute arrival sample, so a close wall automatically gets a shorter
     * burst while a far wall can use a longer one for full LC settling. */
    arrival_us = arrival_abs * 25u;
    burst_us = (arrival_us > CAL_CS_BURST_MARGIN_US) ?
               (arrival_us - CAL_CS_BURST_MARGIN_US) : CAL_CS_BURST_MIN_US;
    if (burst_us < CAL_CS_BURST_MIN_US) burst_us = CAL_CS_BURST_MIN_US;
    if (burst_us > CAL_CS_BURST_MAX_US) burst_us = CAL_CS_BURST_MAX_US;
    cal_active_burst_us = burst_us;
    cal_pick_gate(arrival_abs, &final_start, &final_width);
  }
  {
    float arrival_s = ((float)final_start + 0.5f * (float)final_width) * 25.0e-6f;
    D0_search = 0.5f * c_m_s * arrival_s;
    if (D0_search < 0.12f) D0_search = 0.12f;
    if (D0_search > 3.0f) D0_search = 3.0f;
  }

  cal_debug.stage = 2u;
  cal_debug.stage_ms[1] = HAL_GetTick() - dbg_t0;
  /* ---- step 2: silent floor and compressed-sensing projections -------- */
  cal_report(progress, context, US_CAL_SOLVE, 10u);
  if (cal_cs_measure_noise(link, final_start, final_width, &cal_noise_power) != 0) {
    result->quality_flags |= CAL_Q_MEASURE;
    result->fault = UMH_FAULT_CAL_MIC_SILENT;
    goto fail;
  }
  cal_report(progress, context, US_CAL_MEASURE, 12u);
  if (cal_cs_acquire(link, final_start, final_width, cal_active_burst_us, progress, context) != 0) {
    result->quality_flags |= CAL_Q_MEASURE;
    result->fault = UMH_FAULT_CAL_MIC_SILENT;
    goto fail;
  }

  cal_debug.stage = 3u;
  cal_debug.stage_ms[2] = HAL_GetTick() - dbg_t0;
  cal_debug.patterns = cal_cs_patterns;
  /* ---- step 3: convex reconstruction, pose and channel phases --------- */
  cal_solve_deadline_ms = HAL_GetTick() + CAL_SOLVE_TIME_BUDGET_MS;
  cal_report(progress, context, US_CAL_SOLVE, 76u);
  dbg_part = HAL_GetTick();
  if (cal_cs_reconstruct() != 0) {
    result->quality_flags |= CAL_Q_MEASURE;
    result->fault = UMH_FAULT_CAL_SOLVER;
    goto fail;
  }
  cal_debug.stage_ms[5] = HAL_GetTick() - dbg_part;

  {
    float net = cal_signal_power - cal_noise_power;
    if (net < 0.0f) net = 0.0f;
    echo_ratio = sqrtf(net / (cal_noise_power + 1.0e-9f));
  }

  /* Pose and channel phase are jointly ambiguous with the common-mode
   * nuisance that the gate sees (LC ring residual).  Alternate between the
   * rank-1 + mu fit, removing the estimated common mode from the data, and
   * refining the wall pose on the corrected data.  The first full pose
   * search may itself be biased when the ring residual is large, so a large
   * first mu triggers a fresh full search instead of a local refinement. */
  /* Estimate and remove the common-mode offset at the echo distance; the
   * rank-1 residual pose search then runs on the offset-free data. */
  dbg_part = HAL_GetTick();
  cal_fit_progress_base = 77u;
  cal_stage_update(US_CAL_SOLVE, 77u);
  cal_fit_arho(D0_search * 1000.0f, 0.0f, 0.0f, &residual, &consistency);
  if (cal_solve_expired()) { result->quality_flags |= CAL_Q_MEASURE; result->fault = UMH_FAULT_CAL_SOLVER; goto fail; }
  cal_subtract_mu();
  cal_stage_update(US_CAL_SOLVE, 79u);
  cal_pose_search_residual(D0_search, &D_m, &tilt_x, &tilt_y);
  if (cal_solve_expired()) { result->quality_flags |= CAL_Q_MEASURE; result->fault = UMH_FAULT_CAL_SOLVER; goto fail; }
  cal_fit_progress_base = 90u;
  for (round = 0u; round < 3u; ++round) {
    cal_stage_update(US_CAL_SOLVE, (uint8_t)(cal_fit_progress_base + round));
    cal_fit_arho(D_m * 1000.0f, tilt_x, tilt_y, &residual, &consistency);
    if (cal_solve_expired()) { result->quality_flags |= CAL_Q_MEASURE; result->fault = UMH_FAULT_CAL_SOLVER; goto fail; }
    cal_subtract_mu();
    if (round < 2u) cal_pose_local_residual(&D_m, &tilt_x, &tilt_y);
  }
  cal_stage_update(US_CAL_SOLVE, 96u);
  cal_debug.stage_ms[6] = HAL_GetTick() - dbg_part;
  cal_debug.stage = 5u;
  cal_debug.stage_ms[4] = HAL_GetTick() - dbg_t0;
  /* Microphone amplitude metrics are measured after the common-mode
   * correction so a residual LC ring offset cannot distort mic_ratio. */
  max_amp = 0.0f;
  cal_debug.stage = 4u;
  cal_debug.stage_ms[3] = HAL_GetTick() - dbg_t0;
  for (m = 0u; m < CAL_MICS; ++m) {
    float sum = 0.0f;
    for (i = 0u; i < CAL_CHANNELS; ++i)
      sum += sqrtf(cal_zre[m][i] * cal_zre[m][i] + cal_zim[m][i] * cal_zim[m][i]);
    mean_amp[m] = sum / (float)CAL_CHANNELS;
    if (mean_amp[m] > max_amp) max_amp = mean_amp[m];
  }
  for (m = 0u; m < CAL_MICS; ++m) {
    if (max_amp > 1.0e-9f && mean_amp[m] > 0.25f * max_amp) ++good_mics;
  }
  {
    float mn = mean_amp[0], mx = mean_amp[0];
    for (m = 1u; m < CAL_MICS; ++m) {
      if (mean_amp[m] < mn) mn = mean_amp[m];
      if (mean_amp[m] > mx) mx = mean_amp[m];
    }
    mic_ratio = (mn > 1.0e-9f) ? (mx / mn) : 999.0f;
  }

  /* Gauge-fix the channel phases and build both mirror candidates. */
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
  result->echo_ratio = echo_ratio;
  result->mic_ratio = mic_ratio;
  result->used_gate_start = final_start;
  result->used_gate_count = 1u;
  result->used_gate_width = final_width;
  result->final_block_count = cal_cs_patterns;
  result->good_mics = good_mics;
  result->reserved = (uint8_t)(cal_cs_patterns >= CAL_CHANNELS ? CAL_CHANNELS : cal_cs_patterns);

  if (found == 0u) result->quality_flags |= CAL_Q_ECHO;
  if (echo_ratio < CAL_MIN_ECHO_RATIO) result->quality_flags |= CAL_Q_ECHO;
  if (mic_ratio > CAL_MAX_MIC_RATIO) result->quality_flags |= CAL_Q_MIC_RATIO;
  if (residual > CAL_MAX_RESIDUAL) result->quality_flags |= CAL_Q_RESIDUAL;
  if (consistency > CAL_MAX_MIC_CONSIST) result->quality_flags |= CAL_Q_CONSISTENCY;
  if (rms_pre > CAL_MAX_PRERMS) result->quality_flags |= CAL_Q_PRERMS;
  if (fabsf(tilt_x) > CAL_MAX_TILT || fabsf(tilt_y) > CAL_MAX_TILT)
    result->quality_flags |= CAL_Q_TILT;
  if (cal_cs_patterns < CAL_CHANNELS || good_mics < 2u)
    result->quality_flags |= CAL_Q_MEASURE;

#if !CAL_QUALITY_RELAXED
  if (result->quality_flags != 0u) {
    result->fault = UMH_FAULT_CAL_QUALITY;
    goto fail;
  }
#endif
#if CAL_QUALITY_RELAXED
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
  cal_debug.stage = 0u;
  cal_debug.total_ms = HAL_GetTick() - dbg_t0;
  result->progress = 100u;
  cal_report(progress, context, US_CAL_OK, 100u);
  return 0;

fail:
  cal_debug.stage = 0u;
  cal_debug.total_ms = HAL_GetTick() - dbg_t0;
  result->progress = 100u;
  cal_report(progress, context, US_CAL_FAIL, 100u);
  return -2;
}
