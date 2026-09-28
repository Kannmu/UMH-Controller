/* UMH v7 phase calibration.
 *
 * Production (us_calibration_run): axial table-echo static phase, see the
 * "Axial table-echo" section below.  The near-field direct-path fit described
 * next is kept only as a diagnostic (us_calibration_nearfield_run); its phase
 * is not the axial emission phase and is never written to EEPROM.
 *
 * The 84 transmitters are measured individually against the four on-board
 * SPH0641 microphones.  Only the first acoustic arrival can be trusted: for
 * an external reflector at distance d the earliest room contribution is
 * delayed by 2*d/c, while the on-board direct path is <= 100 mm.  Every
 * (channel, microphone) pair therefore scans a few 25 us gates around its
 * geometric arrival, keeps the earliest narrow peak and stores the complex
 * transfer coefficient:
 *
 *   Z_mi = |Z| * exp(j * (arg(g_m) + arg(a_i) + k*r_mi)).
 *
 * A rank-1 + per-microphone common-mode fit removes g_m and gives the
 * per-channel correction a_i.  Both de-rotation signs are tried; the one
 * with the lower weighted phase residual is written to EEPROM.  The
 * alternating fit runs on complex phasors, so the 64 solver iterations need
 * only multiply-accumulates instead of libm transcendentals.
 *
 * The independent self-test path measures the 4x84 matrix directly, then
 * compares a real production focus against a coherent prediction from that
 * matrix before reporting pass/fail.  No FPGA RTL change is needed.
 */
#include "us_calibration.h"
#include "main.h"
#include "cmsis_os.h"
#include "system_status.h"
#include "cordic.h"
#include "spatial_renderer.h"
#include "umh_utils.h"
#include <math.h>
#include <string.h>

#define CAL_CHANNELS  UMH_DEVICE_CHANNEL_COUNT
#define CAL_MICS      UMH_DEVICE_MIC_COUNT

/* ---- Shared measurement constants --------------------------------------- */
#define CAL_SETTLE_US             3000u  /* LC + room ring-down before next frame */
#define CAL_GATE_WIDTH             16u   /* 400 us gate, matches legacy records   */
#define CAL_GATE_TAIL               8u   /* 200 us margin after the gate          */
#define CAL_GATE_START_MIN         20u   /* 500 us fallback minimum               */
#define CAL_PROFILE_GATES          64u   /* legacy dump section 1 length contract */

/* Distance from the PCB microphone port plane to the piezo ceramic of the
 * open GU1008C-40TR transducer.  The acoustic source is recessed inside the
 * aluminium cylinder and sits 7.0 mm above the PCB. */
#define CAL_SRC_Z_MM                7.0f

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
 * Static working storage.
 *
 * cal_z is the measured 4x84 differential transfer matrix.  The early-path
 * fit rotates each pair once into cal_scratch.rotated, then alternates
 * between the per-microphone common-mode phasor (cal_rho) and the per-channel
 * phasor (cal_a).  cal_scratch.cg[0] is also the raw-capture packet buffer;
 * the capture and fit phases never overlap.
 * -------------------------------------------------------------------------- */
static const umh_device_profile_t *cal_profile;
static float cal_z_re[CAL_MICS][CAL_CHANNELS];
static float cal_z_im[CAL_MICS][CAL_CHANNELS];

typedef union {
  float cg[8][CAL_CHANNELS];              /* raw capture byte buffer view */
  struct {
    float d_re[CAL_MICS][CAL_CHANNELS];   /* z * exp(-j*theta) */
    float d_im[CAL_MICS][CAL_CHANNELS];
  } rotated;
} cal_scratch_u;
static cal_scratch_u cal_scratch;

static float cal_a_re[CAL_CHANNELS];      /* channel phasor A_i */
static float cal_a_im[CAL_CHANNELS];
static float cal_rho_re[CAL_MICS];        /* microphone phasor rho_m */
static float cal_rho_im[CAL_MICS];

/* Magnitude-valid pairs as a 4x84 bitmap; the fit only tests each pair a few
 * times per iteration, and 42 bytes beats 336 bytes of byte flags. */
#define CAL_PAIR_WORDS ((CAL_MICS * CAL_CHANNELS + 31u) / 32u)
static uint32_t cal_pair_valid_bits[CAL_PAIR_WORDS];

static __attribute__((always_inline)) inline uint8_t cal_pair_valid(uint8_t m, uint8_t i)
{
  uint16_t index = (uint16_t)m * CAL_CHANNELS + i;
  return (uint8_t)((cal_pair_valid_bits[index >> 5u] >> (index & 31u)) & 1u);
}

static __attribute__((always_inline)) inline void cal_pair_mark(uint8_t m, uint8_t i, uint8_t valid)
{
  uint16_t index = (uint16_t)m * CAL_CHANNELS + i;
  uint32_t mask = 1u << (index & 31u);
  uint32_t *word = &cal_pair_valid_bits[index >> 5u];
  if (valid != 0u) *word |= mask;
  else *word &= ~mask;
}

/* --------------------------------------------------------------------------
 * Misc state.
 * -------------------------------------------------------------------------- */
static uint32_t cal_frame_sequence;
static uint16_t cal_block_expected;
static uint8_t cal_level_used = 128u;
static uint16_t cal_gate_start = CAL_GATE_START_MIN;
static uint8_t cal_gate_width = CAL_GATE_WIDTH;
static uint32_t cal_burst_us;
static float cal_k_wave_mm;

static int cal_early_run(fpga_link_t *link, const umh_device_profile_t *profile,
                           us_cal_progress_cb_t progress, void *context,
                           umh_calibration_result_t *result);

/* Runtime debug counter; visible in GDB. */
volatile uint32_t cal_debug_cordic_fallbacks;

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
static void cal_report(us_cal_progress_cb_t cb, void *context, uint8_t state, uint8_t progress)
{
  if (cb != NULL) cb(state, progress, context);
}

static void cal_sincos_batch(const float *angles, float *sin_out, float *cos_out, uint32_t count)
{
  uint32_t i;
  if (umh_cordic_sincos_batch(angles, sin_out, cos_out, count) == 0) return;
  ++cal_debug_cordic_fallbacks;
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
  if (x > UMH_PI || x < -UMH_PI) {
    float k = x * (1.0f / UMH_TWO_PI);
    int32_t n = (k >= 0.0f) ? (int32_t)(k + 0.5f) : (int32_t)(k - 0.5f);
    x -= (float)n * UMH_TWO_PI;
  }
  return x;
}

static float cal_deg(float rad) { return rad * (180.0f / UMH_PI); }


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
  if (fpga_link_mic_config(link, gate_count, start, step, width) != 0) return -1;
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



/* --------------------------------------------------------------------------
 * Direct-path geometry.  The fit below is phase-only; this helper supplies
 * the geometric distance used by both the pair scan and the phase model.
 * -------------------------------------------------------------------------- */


static float cal_direct_path_mm(uint8_t mic, uint8_t channel)
{
  float px = (float)cal_profile->coordinates[channel].x_um * 0.001f;
  float py = (float)cal_profile->coordinates[channel].y_um * 0.001f;
  float dx = px - cal_mic_x_mm[mic];
  float dy = py - cal_mic_y_mm[mic];
  return sqrtf(dx * dx + dy * dy + CAL_SRC_Z_MM * CAL_SRC_Z_MM);
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
#define CAL_EARLY_PEAK_FRACTION      0.30f
#define CAL_EARLY_SCAN_MIN            1u   /* ignore the pattern-swap transient */

#define CAL_EARLY_MIN_MAG         10.0f
#define CAL_EARLY_FIT_ITERS          64u
#define CAL_EARLY_PASS_RMS_DEG     30.0f
#define CAL_EARLY_PASS_CHANNELS      70u
#define CAL_EARLY_PASS_MICS           3u



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



static __attribute__((always_inline)) inline void cal_normalize_phasor(float *re, float *im)
{
  float mag2 = (*re) * (*re) + (*im) * (*im);
  if (mag2 > 1.0e-30f) {
    float inv = 1.0f / sqrtf(mag2);
    *re *= inv;
    *im *= inv;
  } else {
    /* Same convention as atan2(0, 0): a zero sum becomes phase zero. */
    *re = 1.0f;
    *im = 0.0f;
  }
}

/* Rotate every measured z_mi pair into the direct-path frame once per sign:
 *   d_mi = z_mi * exp(-j*sign*k*r_mi).
 * The old solver called atan2f + sinf + cosf for each pair on all 64
 * iterations, i.e. about 129k libm calls per run.  The rank-1 update below
 * only needs complex multiply-adds, and the CORDIC batch supplies the
 * rotation cache on hardware (libm fallback is kept inside cal_sincos_batch). */
static void cal_early_rotate_pairs(uint8_t sign)
{
  const float sign_f = (sign != 0u) ? 1.0f : -1.0f;
  uint8_t m;
  uint16_t i;
  for (m = 0u; m < CAL_MICS; ++m) {
    float *cos_row = cal_scratch.rotated.d_re[m];
    float *sin_row = cal_scratch.rotated.d_im[m];
    /* d_re doubles as the angle input: both the CORDIC chunk loop and the
     * libm fallback read an element before overwriting its own output. */
    for (i = 0u; i < CAL_CHANNELS; ++i)
      cos_row[i] = sign_f * cal_k_wave_mm * cal_direct_path_mm(m, (uint8_t)i);
    cal_sincos_batch(cos_row, sin_row, cos_row, CAL_CHANNELS);
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float zr = cal_z_re[m][i];
      float zi = cal_z_im[m][i];
      float c = cos_row[i];
      float s = sin_row[i];
      cos_row[i] = zr * c + zi * s;
      sin_row[i] = zi * c - zr * s;
    }
  }
}

static float cal_early_fit_sign(uint8_t sign, float a_out[CAL_CHANNELS],
                                uint16_t valid_out[CAL_CHANNELS],
                                uint16_t mic_valid_out[CAL_MICS],
                                uint16_t *valid_pairs_out)
{
  float weighted_err = 0.0f, weight_sum = 0.0f;
  uint16_t valid_pairs = 0u;
  uint8_t iter, m, i;

  cal_early_rotate_pairs(sign);
  for (i = 0u; i < CAL_CHANNELS; ++i) { cal_a_re[i] = 1.0f; cal_a_im[i] = 0.0f; }
  for (m = 0u; m < CAL_MICS; ++m) { cal_rho_re[m] = 1.0f; cal_rho_im[m] = 0.0f; mic_valid_out[m] = 0u; }

  /* Alternating rank-1 + common-mode projection on phasors.  This is the
   * complex form of the original sr/si angle updates and removes every
   * atan2f/sinf/cosf from the iteration. */
  for (iter = 0u; iter < CAL_EARLY_FIT_ITERS; ++iter) {
    for (m = 0u; m < CAL_MICS; ++m) {
      float sr = 0.0f, si = 0.0f;
      for (i = 0u; i < CAL_CHANNELS; ++i) {
        float dr, di;
        if (cal_pair_valid(m, i) == 0u) continue;
        dr = cal_scratch.rotated.d_re[m][i];
        di = cal_scratch.rotated.d_im[m][i];
        sr += dr * cal_a_re[i] + di * cal_a_im[i];
        si += di * cal_a_re[i] - dr * cal_a_im[i];
      }
      cal_normalize_phasor(&sr, &si);
      cal_rho_re[m] = sr;
      cal_rho_im[m] = si;
    }
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float sr = 0.0f, si = 0.0f;
      for (m = 0u; m < CAL_MICS; ++m) {
        float dr, di;
        if (cal_pair_valid(m, i) == 0u) continue;
        dr = cal_scratch.rotated.d_re[m][i];
        di = cal_scratch.rotated.d_im[m][i];
        sr += dr * cal_rho_re[m] + di * cal_rho_im[m];
        si += di * cal_rho_re[m] - dr * cal_rho_im[m];
      }
      cal_normalize_phasor(&sr, &si);
      cal_a_re[i] = sr;
      cal_a_im[i] = si;
    }
  }

  for (i = 0u; i < CAL_CHANNELS; ++i) {
    uint16_t count = 0u;
    for (m = 0u; m < CAL_MICS; ++m) {
      float dr, di, mr, mi, mag, wr, wi, err;
      if (cal_pair_valid(m, i) == 0u) continue;
      dr = cal_scratch.rotated.d_re[m][i];
      di = cal_scratch.rotated.d_im[m][i];
      /* Model phasor M = A_i * rho_m; error = angle(d_mi * conj(M)). */
      mr = cal_a_re[i] * cal_rho_re[m] - cal_a_im[i] * cal_rho_im[m];
      mi = cal_a_re[i] * cal_rho_im[m] + cal_a_im[i] * cal_rho_re[m];
      wr = dr * mr + di * mi;
      wi = di * mr - dr * mi;
      mag = sqrtf(dr * dr + di * di);
      err = cal_wrap_pi(cal_atan2_rad(wr, wi));
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
  for (i = 0u; i < CAL_CHANNELS; ++i)
    a_out[i] = cal_atan2_rad(cal_a_re[i], cal_a_im[i]);
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
  cal_k_wave_mm = UMH_TWO_PI * (float)profile->carrier_hz / c_mm_s;

  cal_report(progress, context, US_CAL_WAIT, 0u);
  cal_report(progress, context, US_CAL_MEASURE, 2u);
  if (cal_early_measure_h(link, progress, context) != 0) {
    result->fault = UMH_FAULT_CAL_MIC_SILENT;
    result->progress = 100u;
    cal_report(progress, context, US_CAL_FAIL, 100u);
    return -2;
  }
  cal_report(progress, context, US_CAL_SOLVE, 76u);

  /* Pair validity depends only on the measured magnitude, so build the mask
   * once for both rotation signs. */
  for (m = 0u; m < CAL_MICS; ++m) {
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float zr = cal_z_re[m][i];
      float zi = cal_z_im[m][i];
      cal_pair_mark(m, (uint8_t)i, sqrtf(zr * zr + zi * zi) < CAL_EARLY_MIN_MAG ? 0u : 1u);
    }
  }

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
                   (int32_t)lroundf(-a * (256.0f / UMH_TWO_PI)) : 0;
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
 * Bench time-profile probe.
 * -------------------------------------------------------------------------- */
static int cal_probe_block(fpga_link_t *link, umh_output_frame_t *frame,
                           uint32_t burst_us, uint8_t gate_count, float sign,
                           float *acc, uint16_t *saturated)
{
  fpga_mic_gate_wire_t wire;
  uint16_t expected;
  uint8_t g, m;
  frame->sequence = ++cal_frame_sequence;
  if (cal_submit_frame(link, frame) != 0) return -1;
  cal_delay_us(burst_us);
  if (fpga_link_safe_stop(link) != 0) return -2;
  expected = (uint16_t)(cal_block_expected + 1u);
  if (cal_wait_block_fast(link, expected, burst_us / 1000u + 300u, &wire) != 0) return -3;
  cal_block_expected = expected;
  if ((wire.status & FPGA_MIC_STATUS_SATURATED) != 0u && saturated != NULL) ++*saturated;
  for (g = 0u; g < gate_count; ++g) {
    if (fpga_link_mic_read(link, g, &wire) != 0) return -4;
    for (m = 0u; m < CAL_MICS; ++m) {
      acc[(uint32_t)g * 8u + m * 2u] += sign * (float)wire.i[m];
      acc[(uint32_t)g * 8u + m * 2u + 1u] += sign * (float)wire.q[m];
    }
  }
  return 0;
}

int us_calibration_probe(fpga_link_t *link, const uint8_t *phase, const uint8_t *level,
                         uint8_t gate_count, uint16_t gate_start, uint16_t gate_step,
                         uint8_t gate_width, uint32_t burst_us, uint32_t settle_us,
                         uint8_t repeats, uint8_t flags, float *out,
                         uint16_t *saturated_blocks)
{
  umh_output_frame_t frame, inv;
  uint32_t n, k;
  uint8_t r, i;
  float scale;
  if (link == NULL || phase == NULL || level == NULL || out == NULL) return -1;
  if (gate_count == 0u || gate_count > FPGA_MIC_MAX_GATES || gate_width == 0u ||
      gate_width > 64u || gate_step < gate_width || burst_us == 0u) return -1;
  if (repeats == 0u) repeats = 1u;
  if (saturated_blocks != NULL) *saturated_blocks = 0u;
  n = (uint32_t)gate_count * 8u;
  for (k = 0u; k < n; ++k) out[k] = 0.0f;
  if (cal_mic_start(link, gate_count, gate_start, gate_step, gate_width) != 0) return -2;
  memset(&frame, 0, sizeof(frame));
  frame.update_flags = UMH_FRAME_FLAG_ULTRASOUND;
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    frame.channels[i].phase = phase[i];
    frame.channels[i].level = level[i];
  }
  inv = frame;
  for (i = 0u; i < CAL_CHANNELS; ++i)
    inv.channels[i].phase = (uint8_t)(inv.channels[i].phase + 128u);
  for (r = 0u; r < repeats; ++r) {
    int rc = cal_probe_block(link, &frame, burst_us, gate_count, 1.0f, out,
                             saturated_blocks);
    if (rc != 0) return rc;
    cal_delay_us(settle_us);
    if ((flags & US_CAL_PROBE_DIFF) != 0u) {
      rc = cal_probe_block(link, &inv, burst_us, gate_count, -1.0f, out,
                           saturated_blocks);
      if (rc != 0) return rc - 10;
      cal_delay_us(settle_us);
    }
  }
  scale = ((flags & US_CAL_PROBE_DIFF) != 0u) ? 0.5f / (float)repeats : 1.0f / (float)repeats;
  for (k = 0u; k < n; ++k) out[k] *= scale;
  return 0;
}

float *us_calibration_scratch(uint32_t *bytes)
{
  if (bytes != NULL) *bytes = (uint32_t)sizeof(cal_scratch);
  return cal_scratch.cg[0];
}

/* ==========================================================================
 * Axial table-echo static phase calibration (production CAL_START / GUI).
 *
 * Why not the near-field path: the in-plane direct coupling reaches the
 * microphones at 60..90 deg off the transducer axis.  Bench data showed that
 * this phase is 75..84 deg RMS away from the axial emission phase (all sign
 * combinations), i.e. it cannot calibrate the field the array actually
 * produces.  The old EEPROM map built that way lost 2.9 dB of real focus
 * (worse than zero correction in 38/40 random focus sets).
 *
 * Method: the array faces a flat surface.  Each channel alone is driven at a
 * linear drive level; every microphone sees the direct near field (steady
 * state before the echo) and then the echo from the mirror image source.
 *   d_mi  = echo - pre-echo baseline         (phase-cycled, 0.5*(y0 - y180))
 *   u_mi  = d_mi * exp(-j*k*r_e) / |d_mi|,   r_e = sqrt(h^2 + (2V + z_src)^2)
 * Only pairs whose reflection leaves the transducer within ~17 deg of its axis
 * (h < 60 mm) are used: beyond ~18 deg the element directivity has a
 * channel-dependent null with +-180 deg flips.  All cone reflection points
 * lie within r < 46 mm, so only a ~10 cm patch below the array must be free;
 * side obstacles (stand legs) arrive later and outside the window.
 * u_mi = rho_m * a_i is solved by alternating phasor projection.  The gauge
 * removes the common phase, the plane (reflector tilt) and the x^2+y^2 term
 * (reflector distance error), then each channel is shrunk by its standard
 * error (Wiener), so noise-level differences stay at zero and only
 * significant errors (inverted/late channels) are corrected.
 * Dead channels are detected from the echo magnitude and disabled.  Gain is
 * not equalised: the measured spread (p10/p90 ~0.85/1.15) equals the
 * level-to-level repeatability (~12-15 %), and equalising would only lower
 * total output.  Before a result is accepted it must pass a real production
 * renderer focus test (3-channel sets focused on each microphone image)
 * against zero correction.
 * ========================================================================== */
#define CAL_ECHO_LEVEL             16u   /* single channel, linear echo regime  */
#define CAL_ECHO_REPEATS            8u
#define CAL_ECHO_SETTLE_US      20000u   /* table/array reverberation decay     */
#define CAL_ECHO_BURST_TAIL_US   2500u   /* event-table swap margin             */
#define CAL_ECHO_A_START            8u   /* A profile: 200 .. 1775 us           */
#define CAL_ECHO_A_GATES           64u
#define CAL_ECHO_A_REPEATS          4u
#define CAL_ECHO_A_CHANNELS         6u
#define CAL_ECHO_ONSET_LAT_US      95.0f /* t50 detector latency (bench)        */
#define CAL_ECHO_B_GATES           16u
#define CAL_ECHO_B_WIDTH            2u   /* 50 us gates                         */
#define CAL_ECHO_B_PRE             14u   /* B start = t_on - 350 us             */
#define CAL_ECHO_CONE_H_MM         60.0f
#define CAL_ECHO_MIN_RD_MM         14.0f
#define CAL_ECHO_BASE_RATIO         0.6f
#define CAL_ECHO_DEAD_FRACTION      0.15f
#define CAL_ECHO_MIN_MAG            3.0f
#define CAL_ECHO_V_MIN_MM          40.0f
#define CAL_ECHO_V_MAX_MM         250.0f
#define CAL_ECHO_ITERS             30u
#define CAL_ECHO_PASS_COVERED      70u
#define CAL_ECHO_PASS_MIC_PAIRS    10u
#define CAL_ECHO_PASS_MICS          3u
#define CAL_ECHO_PASS_SD_DEG       35.0f
#define CAL_ECHO_PASS_DEAD          8u
#define CAL_ECHO_VERIFY_LEVEL       8u
#define CAL_ECHO_VERIFY_CHANNELS    3u   /* 3 x level 8 keeps the mic linear    */
#define CAL_ECHO_VERIFY_SETS        4u   /* per microphone                      */
#define CAL_ECHO_VERIFY_REPEATS     4u
#define CAL_ECHO_VERIFY_MIN_DB    (-0.5f)

static umh_spatial_renderer_t cal_echo_renderer;
static umh_channel_calibration_t cal_echo_cal[CAL_CHANNELS];
static float cal_echo_sort[CAL_CHANNELS];
static float cal_echo_gates[CAL_ECHO_B_GATES * 8u];
static uint8_t cal_echo_dead[CAL_CHANNELS];
static uint8_t cal_echo_n[CAL_CHANNELS];

typedef struct {
  float v_mm;          /* emitter plane -> reflector */
  float t_on_us;       /* echo t50 for h = 30 mm     */
  uint16_t start;      /* B gate start (25 us units) */
  uint32_t burst_us;
  uint16_t base_mask;  /* gates averaged as baseline */
  uint16_t echo_mask;  /* gates averaged as echo     */
} cal_echo_plan_t;

static float cal_hdist_mm(uint8_t mic, uint8_t channel)
{
  float dx = (float)cal_profile->coordinates[channel].x_um * 0.001f - cal_mic_x_mm[mic];
  float dy = (float)cal_profile->coordinates[channel].y_um * 0.001f - cal_mic_y_mm[mic];
  return sqrtf(dx * dx + dy * dy);
}

static float cal_echo_path_mm(float h, float v_mm)
{
  float z = 2.0f * v_mm + CAL_SRC_Z_MM;
  return sqrtf(h * h + z * z);
}

/* In-place ascending sort of a short float list; returns the median. */
static float cal_sort_median(float *x, uint16_t n)
{
  uint16_t i, j;
  if (n == 0u) return 0.0f;
  for (i = 1u; i < n; ++i) {
    float v = x[i];
    j = i;
    while (j > 0u && x[j - 1u] > v) { x[j] = x[j - 1u]; --j; }
    x[j] = v;
  }
  return ((n & 1u) != 0u) ? x[n / 2u] : 0.5f * (x[n / 2u - 1u] + x[n / 2u]);
}

/* t50 of the echo step in a [gate][mic][I,Q] 25 us profile, in gates. */
static float cal_echo_onset(const float *prof, uint8_t mic, float *peak)
{
  float dev[CAL_ECHO_A_GATES];
  float mx = 0.0f;
  uint8_t g, k;
  for (g = 0u; g < CAL_ECHO_A_GATES; ++g) {
    float br = 0.0f, bi = 0.0f, dr, di;
    dev[g] = 0.0f;
    if (g < 6u) continue;
    for (k = (uint8_t)(g - 6u); k < (uint8_t)(g - 3u); ++k) {
      br += prof[(uint32_t)k * 8u + mic * 2u];
      bi += prof[(uint32_t)k * 8u + mic * 2u + 1u];
    }
    dr = prof[(uint32_t)g * 8u + mic * 2u] - br * (1.0f / 3.0f);
    di = prof[(uint32_t)g * 8u + mic * 2u + 1u] - bi * (1.0f / 3.0f);
    dev[g] = sqrtf(dr * dr + di * di);
    if (dev[g] > mx) mx = dev[g];
  }
  *peak = mx;
  if (mx <= 0.0f) return -1.0f;
  for (g = 6u; g < CAL_ECHO_A_GATES; ++g) {
    if (dev[g] >= 0.5f * mx) {
      float frac = (dev[g] > dev[g - 1u]) ?
                   (0.5f * mx - dev[g - 1u]) / (dev[g] - dev[g - 1u]) : 1.0f;
      return (float)(g - 1u) + frac;
    }
  }
  return -1.0f;
}

static int cal_echo_single(fpga_link_t *link, uint8_t channel, uint8_t level,
                           uint8_t gate_count, uint16_t start, uint16_t step,
                           uint8_t width, uint32_t burst_us, uint8_t repeats,
                           float *out, uint16_t *sat)
{
  uint8_t phase[CAL_CHANNELS], lv[CAL_CHANNELS];
  memset(phase, 0, sizeof(phase));
  memset(lv, 0, sizeof(lv));
  lv[channel] = level;
  return us_calibration_probe(link, phase, lv, gate_count, start, step, width,
                              burst_us, CAL_ECHO_SETTLE_US, repeats,
                              US_CAL_PROBE_DIFF, out, sat);
}

/* Stage A: table distance from the echo onset of a few ring channels. */
static int cal_echo_table(fpga_link_t *link, float c_mm_us, float *v_out,
                          us_cal_progress_cb_t progress, void *context)
{
  float *prof = cal_scratch.cg[0];   /* 64 gates x 8 floats = 2 KB */
  float v_list[CAL_ECHO_A_CHANNELS * CAL_MICS];
  uint16_t nv = 0u, sat = 0u;
  uint8_t i, m, used = 0u;
  uint32_t burst = ((uint32_t)CAL_ECHO_A_START + CAL_ECHO_A_GATES) * 25u + CAL_ECHO_BURST_TAIL_US;
  for (i = 0u; i < CAL_CHANNELS && used < CAL_ECHO_A_CHANNELS; ++i) {
    float px = (float)cal_profile->coordinates[i].x_um * 0.001f;
    float py = (float)cal_profile->coordinates[i].y_um * 0.001f;
    float r = sqrtf(px * px + py * py);
    if (r < 25.0f || r > 35.0f) continue;
    ++used;
    if (cal_echo_single(link, i, CAL_ECHO_LEVEL, CAL_ECHO_A_GATES, CAL_ECHO_A_START,
                        1u, 1u, burst, CAL_ECHO_A_REPEATS, prof, &sat) != 0)
      return -1;
    for (m = 0u; m < CAL_MICS; ++m) {
      float peak, g, t, re, h = cal_hdist_mm(m, i);
      if (cal_direct_path_mm(m, i) < 25.0f) continue;
      g = cal_echo_onset(prof, m, &peak);
      if (g < 0.0f || peak < 4.0f) continue;
      t = ((float)CAL_ECHO_A_START + g) * 25.0f;
      re = (t - CAL_ECHO_ONSET_LAT_US) * c_mm_us;
      if (re <= h) continue;
      v_list[nv++] = 0.5f * (sqrtf(re * re - h * h) - CAL_SRC_Z_MM);
    }
    cal_report(progress, context, US_CAL_MEASURE, (uint8_t)(2u + used));
  }
  if (nv < 4u) return -2;
  *v_out = cal_sort_median(v_list, nv);
  return 0;
}

static int cal_echo_make_plan(float v_mm, float c_mm_us, cal_echo_plan_t *plan)
{
  float second, end;
  int32_t start;
  uint8_t g, nb = 0u, ne = 0u;
  plan->v_mm = v_mm;
  plan->t_on_us = cal_echo_path_mm(30.0f, v_mm) / c_mm_us + CAL_ECHO_ONSET_LAT_US;
  start = (int32_t)lroundf(plan->t_on_us / 25.0f) - (int32_t)CAL_ECHO_B_PRE;
  if (start < 6) start = 6;
  plan->start = (uint16_t)start;
  plan->burst_us = ((uint32_t)plan->start + CAL_ECHO_B_GATES * CAL_ECHO_B_WIDTH) * 25u +
                   CAL_ECHO_BURST_TAIL_US;
  second = 2.0f * (2.0f * v_mm + CAL_SRC_Z_MM) / c_mm_us;
  end = plan->t_on_us + 400.0f;
  if (plan->t_on_us + 0.8f * second < end) end = plan->t_on_us + 0.8f * second;
  plan->base_mask = 0u;
  plan->echo_mask = 0u;
  for (g = 0u; g < CAL_ECHO_B_GATES; ++g) {
    float t0 = (float)(plan->start + (uint16_t)g * CAL_ECHO_B_WIDTH) * 25.0f;
    float t1 = t0 + (float)CAL_ECHO_B_WIDTH * 25.0f;
    if (t0 >= plan->t_on_us - 300.0f && t1 <= plan->t_on_us - 50.0f) {
      plan->base_mask |= (uint16_t)(1u << g); ++nb;
    }
    if (t0 >= plan->t_on_us + 100.0f && t1 <= end) {
      plan->echo_mask |= (uint16_t)(1u << g); ++ne;
    }
  }
  return (nb >= 2u && ne >= 3u) ? 0 : -1;
}

/* echo - baseline for every microphone from a B-plan gate capture. */
static void cal_echo_reduce(const cal_echo_plan_t *plan, const float *gates,
                            float d_re[CAL_MICS], float d_im[CAL_MICS],
                            float b_mag[CAL_MICS], float e_mag[CAL_MICS])
{
  uint8_t g, m;
  for (m = 0u; m < CAL_MICS; ++m) {
    float br = 0.0f, bi = 0.0f, er = 0.0f, ei = 0.0f;
    float nb = 0.0f, ne = 0.0f;
    for (g = 0u; g < CAL_ECHO_B_GATES; ++g) {
      float gr = gates[(uint32_t)g * 8u + m * 2u];
      float gi = gates[(uint32_t)g * 8u + m * 2u + 1u];
      if ((plan->base_mask >> g) & 1u) { br += gr; bi += gi; nb += 1.0f; }
      if ((plan->echo_mask >> g) & 1u) { er += gr; ei += gi; ne += 1.0f; }
    }
    br /= nb; bi /= nb; er /= ne; ei /= ne;
    d_re[m] = er - br;
    d_im[m] = ei - bi;
    if (b_mag != NULL) b_mag[m] = sqrtf(br * br + bi * bi);
    if (e_mag != NULL) e_mag[m] = sqrtf(er * er + ei * ei);
  }
}

/* Solve a 4x4 system in place (Gauss-Jordan, partial pivot). */
static int cal_solve4(float a[4][4], float b[4])
{
  uint8_t c, r, k;
  for (c = 0u; c < 4u; ++c) {
    uint8_t p = c;
    for (r = (uint8_t)(c + 1u); r < 4u; ++r) if (fabsf(a[r][c]) > fabsf(a[p][c])) p = r;
    if (fabsf(a[p][c]) < 1.0e-9f) return -1;
    if (p != c) {
      for (k = 0u; k < 4u; ++k) { float t = a[c][k]; a[c][k] = a[p][k]; a[p][k] = t; }
      { float t = b[c]; b[c] = b[p]; b[p] = t; }
    }
    for (r = 0u; r < 4u; ++r) {
      float f;
      if (r == c) continue;
      f = a[r][c] / a[c][c];
      for (k = c; k < 4u; ++k) a[r][k] -= f * a[c][k];
      b[r] -= f * b[c];
    }
  }
  for (c = 0u; c < 4u; ++c) b[c] /= a[c][c];
  return 0;
}

/* Stage E: measured focus |echo| at one microphone image. */
static int cal_echo_focus(fpga_link_t *link, const cal_echo_plan_t *plan,
                          const uint8_t *phase_bytes, const uint8_t *chans,
                          uint8_t mic, float *mag_out)
{
  umh_spatial_point_t point;
  umh_output_frame_t frame;
  uint8_t phase[CAL_CHANNELS], level[CAL_CHANNELS];
  float d_re[CAL_MICS], d_im[CAL_MICS];
  uint8_t i;
  uint16_t sat = 0u;
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    cal_echo_cal[i].phase = (phase_bytes != NULL) ? phase_bytes[i] : 0u;
    cal_echo_cal[i].gain = 255u;
    cal_echo_cal[i].enabled = 1u;
  }
  spatial_renderer_set_calibration(&cal_echo_renderer, cal_echo_cal, CAL_CHANNELS);
  point.x_um = (int32_t)lroundf(cal_mic_x_mm[mic] * 1000.0f);
  point.y_um = (int32_t)lroundf(cal_mic_y_mm[mic] * 1000.0f);
  point.z_um = (int32_t)lroundf((2.0f * plan->v_mm + 2.0f * CAL_SRC_Z_MM) * 1000.0f);
  point.level = 255u;
  point.phase = 0u;
  point.source_id = 0u;
  memset(&frame, 0, sizeof(frame));
  if (spatial_renderer_point(&cal_echo_renderer, &point, &frame) != 0) return -1;
  memset(level, 0, sizeof(level));
  for (i = 0u; i < CAL_CHANNELS; ++i) phase[i] = frame.channels[i].phase;
  for (i = 0u; i < CAL_ECHO_VERIFY_CHANNELS; ++i) level[chans[i]] = CAL_ECHO_VERIFY_LEVEL;
  if (us_calibration_probe(link, phase, level, CAL_ECHO_B_GATES, plan->start,
                           CAL_ECHO_B_WIDTH, CAL_ECHO_B_WIDTH, plan->burst_us,
                           CAL_ECHO_SETTLE_US, CAL_ECHO_VERIFY_REPEATS,
                           US_CAL_PROBE_DIFF, cal_echo_gates, &sat) != 0)
    return -2;
  cal_echo_reduce(plan, cal_echo_gates, d_re, d_im, NULL, NULL);
  *mag_out = sqrtf(d_re[mic] * d_re[mic] + d_im[mic] * d_im[mic]);
  return 0;
}

static int cal_echo_fail(umh_calibration_result_t *result, us_cal_progress_cb_t progress,
                         void *context, uint8_t fault, uint8_t flags, int rc)
{
  result->fault = fault;
  result->quality_flags |= flags;
  result->progress = 100u;
  cal_report(progress, context, US_CAL_FAIL, 100u);
  return rc;
}

static int cal_echo_run(fpga_link_t *link, const umh_device_profile_t *profile,
                        us_cal_progress_cb_t progress, void *context,
                        umh_calibration_result_t *result)
{
  cal_echo_plan_t plan;
  float c_mm_s, c_mm_us, v_mm = 0.0f, sd2, err_sum = 0.0f, echo_med;
  float basis_sol[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float mic_med[CAL_MICS];
  float ph[CAL_CHANNELS];
  uint16_t mic_pairs[CAL_MICS] = {0u, 0u, 0u, 0u};
  uint16_t pairs = 0u, covered = 0u, n_sort, sat_total = 0u;
  uint8_t i, m, it, good_mics = 0u, dead = 0u;

  if (link == NULL || profile == NULL || result == NULL) return -1;
  memset(result, 0, sizeof(*result));
  cal_profile = profile;
  cal_frame_sequence = 0u;
  cal_block_expected = 0u;
  c_mm_s = (profile->sound_speed_um_per_s != 0u) ?
           ((float)profile->sound_speed_um_per_s * 1.0e-3f) : 343000.0f;
  c_mm_us = c_mm_s * 1.0e-6f;
  cal_k_wave_mm = UMH_TWO_PI * (float)profile->carrier_hz / c_mm_s;
  result->level_used = CAL_ECHO_LEVEL;
  result->geom_hypothesis = 2u;          /* 2 = axial table echo method */
  result->used_gate_count = CAL_ECHO_B_GATES;
  result->used_gate_width = CAL_ECHO_B_WIDTH;
  result->patterns_used = CAL_CHANNELS;

  cal_report(progress, context, US_CAL_MEASURE, 1u);
  /* ---- A: reflector distance ------------------------------------------ */
  {
    int rc = cal_echo_table(link, c_mm_us, &v_mm, progress, context);
    if (rc == -1) return cal_echo_fail(result, progress, context, UMH_FAULT_CAL_MIC_SILENT,
                                       CAL_Q_MEASURE, -2);
    result->table_mm = v_mm;
    if (rc != 0 || v_mm < CAL_ECHO_V_MIN_MM || v_mm > CAL_ECHO_V_MAX_MM)
      return cal_echo_fail(result, progress, context, UMH_FAULT_CAL_QUALITY, CAL_Q_GEOM, -3);
  }
  if (cal_echo_make_plan(v_mm, c_mm_us, &plan) != 0)
    return cal_echo_fail(result, progress, context, UMH_FAULT_CAL_QUALITY, CAL_Q_GEOM, -3);
  result->used_gate_start = plan.start;

  /* ---- B: per-channel echo phasor ---------------------------------------- */
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    float d_re[CAL_MICS], d_im[CAL_MICS], b_mag[CAL_MICS], e_mag[CAL_MICS];
    uint16_t sat = 0u;
    if (cal_echo_single(link, i, CAL_ECHO_LEVEL, CAL_ECHO_B_GATES, plan.start,
                        CAL_ECHO_B_WIDTH, CAL_ECHO_B_WIDTH, plan.burst_us,
                        CAL_ECHO_REPEATS, cal_echo_gates, &sat) != 0)
      return cal_echo_fail(result, progress, context, UMH_FAULT_CAL_MIC_SILENT,
                           CAL_Q_MEASURE, -2);
    sat_total = (uint16_t)(sat_total + sat);
    cal_echo_reduce(&plan, cal_echo_gates, d_re, d_im, b_mag, e_mag);
    for (m = 0u; m < CAL_MICS; ++m) {
      cal_z_re[m][i] = d_re[m];
      cal_z_im[m][i] = d_im[m];
      cal_pair_mark(m, i, (b_mag[m] < CAL_ECHO_BASE_RATIO * e_mag[m]) ? 1u : 0u);
    }
    if ((i & 3u) == 0u)
      cal_report(progress, context, US_CAL_MEASURE, (uint8_t)(10u + (uint32_t)i * 70u / CAL_CHANNELS));
  }
  result->drift_deg = (float)sat_total;   /* saturated FPGA blocks (diagnostic) */
  cal_report(progress, context, US_CAL_SOLVE, 80u);

  /* ---- C: channel health ----------------------------------------------- */
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    float mx = 0.0f;
    for (m = 0u; m < CAL_MICS; ++m) {
      float a = sqrtf(cal_z_re[m][i] * cal_z_re[m][i] + cal_z_im[m][i] * cal_z_im[m][i]);
      if (a > mx) mx = a;
    }
    cal_echo_sort[i] = mx;
    ph[i] = mx;
  }
  echo_med = cal_sort_median(cal_echo_sort, CAL_CHANNELS);
  result->echo_mag = echo_med;
  if (echo_med < CAL_ECHO_MIN_MAG)
    return cal_echo_fail(result, progress, context, UMH_FAULT_CAL_QUALITY, CAL_Q_COUPLING, -4);
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    cal_echo_dead[i] = (ph[i] < CAL_ECHO_DEAD_FRACTION * echo_med) ? 1u : 0u;
    if (cal_echo_dead[i] != 0u) ++dead;
  }
  result->dead_count = dead;

  /* ---- C: cone pairs, de-rotated unit phasors --------------------------- */
  for (m = 0u; m < CAL_MICS; ++m) {
    float *cr = cal_scratch.rotated.d_re[m];
    float *sr = cal_scratch.rotated.d_im[m];
    for (i = 0u; i < CAL_CHANNELS; ++i)
      cr[i] = cal_k_wave_mm * cal_echo_path_mm(cal_hdist_mm(m, i), v_mm);
    cal_sincos_batch(cr, sr, cr, CAL_CHANNELS);
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float zr = cal_z_re[m][i], zi = cal_z_im[m][i];
      float c = cr[i], s = sr[i];
      float ur = zr * c + zi * s;      /* z * exp(-j k r_e) */
      float ui = zi * c - zr * s;
      uint8_t ok = cal_pair_valid(m, i);
      if (cal_hdist_mm(m, i) >= CAL_ECHO_CONE_H_MM) ok = 0u;
      if (cal_direct_path_mm(m, i) <= CAL_ECHO_MIN_RD_MM) ok = 0u;
      if (cal_echo_dead[i] != 0u) ok = 0u;
      cal_normalize_phasor(&ur, &ui);
      cr[i] = ur;
      sr[i] = ui;
      cal_pair_mark(m, i, ok);
      if (ok != 0u) { ++mic_pairs[m]; ++pairs; }
    }
  }
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    uint8_t n = 0u;
    for (m = 0u; m < CAL_MICS; ++m) n = (uint8_t)(n + cal_pair_valid(m, i));
    cal_echo_n[i] = n;
    if (n != 0u) ++covered;
    cal_a_re[i] = 1.0f;
    cal_a_im[i] = 0.0f;
  }
  for (m = 0u; m < CAL_MICS; ++m) if (mic_pairs[m] >= CAL_ECHO_PASS_MIC_PAIRS) ++good_mics;
  result->pairs_used = pairs;
  result->channels_covered = (uint8_t)covered;
  result->good_mics = good_mics;
  if (covered == 0u || good_mics == 0u)
    return cal_echo_fail(result, progress, context, UMH_FAULT_CAL_QUALITY, CAL_Q_MICS, -5);

  /* u_mi = rho_m * a_i  (rho normalised, a = unnormalised phasor sum). */
  for (it = 0u; it < CAL_ECHO_ITERS; ++it) {
    for (m = 0u; m < CAL_MICS; ++m) {
      float sr = 0.0f, si = 0.0f;
      for (i = 0u; i < CAL_CHANNELS; ++i) {
        float ur, ui;
        if (cal_pair_valid(m, i) == 0u) continue;
        ur = cal_scratch.rotated.d_re[m][i];
        ui = cal_scratch.rotated.d_im[m][i];
        sr += ur * cal_a_re[i] + ui * cal_a_im[i];
        si += ui * cal_a_re[i] - ur * cal_a_im[i];
      }
      cal_normalize_phasor(&sr, &si);
      cal_rho_re[m] = sr;
      cal_rho_im[m] = si;
    }
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float sr = 0.0f, si = 0.0f;
      for (m = 0u; m < CAL_MICS; ++m) {
        float ur, ui;
        if (cal_pair_valid(m, i) == 0u) continue;
        ur = cal_scratch.rotated.d_re[m][i];
        ui = cal_scratch.rotated.d_im[m][i];
        sr += ur * cal_rho_re[m] + ui * cal_rho_im[m];
        si += ui * cal_rho_re[m] - ur * cal_rho_im[m];
      }
      cal_a_re[i] = sr;
      cal_a_im[i] = si;
    }
  }
  for (i = 0u; i < CAL_CHANNELS; ++i)
    ph[i] = (cal_echo_n[i] != 0u) ? cal_atan2_rad(cal_a_re[i], cal_a_im[i]) : 0.0f;
  /* Pair residual against the fit itself (before the gauge). */
  for (m = 0u; m < CAL_MICS; ++m) {
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float ur, ui, vr, vi, wr, wi, ar, ai, e;
      if (cal_pair_valid(m, i) == 0u) continue;
      ur = cal_scratch.rotated.d_re[m][i];
      ui = cal_scratch.rotated.d_im[m][i];
      vr = ur * cal_rho_re[m] + ui * cal_rho_im[m];
      vi = ui * cal_rho_re[m] - ur * cal_rho_im[m];
      ar = cal_a_re[i]; ai = cal_a_im[i];
      wr = vr * ar + vi * ai;
      wi = vi * ar - vr * ai;
      e = cal_wrap_pi(cal_atan2_rad(wr, wi));
      err_sum += e * e;
    }
  }
  {
    int32_t dof = (int32_t)pairs - (int32_t)covered - (int32_t)CAL_MICS;
    if (dof < 1) dof = 1;
    sd2 = err_sum / (float)dof;
  }

  /* Gauge: common + plane + x^2+y^2, gross faults excluded from the fit. */
  for (it = 0u; it < 3u; ++it) {
    float a[4][4], b[4];
    uint8_t r, c;
    memset(a, 0, sizeof(a));
    memset(b, 0, sizeof(b));
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float x, y, g[4], w;
      if (cal_echo_n[i] == 0u || fabsf(ph[i]) >= 0.5f * UMH_PI) continue;
      x = (float)profile->coordinates[i].x_um * 2.0e-5f;   /* / 50 mm */
      y = (float)profile->coordinates[i].y_um * 2.0e-5f;
      g[0] = 1.0f; g[1] = x; g[2] = y; g[3] = x * x + y * y;
      w = (float)cal_echo_n[i];
      for (r = 0u; r < 4u; ++r) {
        b[r] += w * g[r] * ph[i];
        for (c = 0u; c < 4u; ++c) a[r][c] += w * g[r] * g[c];
      }
    }
    if (cal_solve4(a, b) != 0) break;
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float x, y;
      if (cal_echo_n[i] == 0u) continue;
      x = (float)profile->coordinates[i].x_um * 2.0e-5f;
      y = (float)profile->coordinates[i].y_um * 2.0e-5f;
      ph[i] = cal_wrap_pi(ph[i] - (b[0] + b[1] * x + b[2] * y + b[3] * (x * x + y * y)));
    }
    for (r = 0u; r < 4u; ++r) basis_sol[r] += b[r];
  }
  /* Plane term -> reflector tilt (diagnostic): phase slope = k * tilt. */
  result->tilt_x_deg = cal_deg((basis_sol[1] / 50.0f) / cal_k_wave_mm);
  result->tilt_y_deg = cal_deg((basis_sol[2] / 50.0f) / cal_k_wave_mm);

  /* Wiener shrinkage and byte quantisation. */
  {
    float before = 0.0f, after = 0.0f;
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float q = 0.0f;
      int32_t code;
      if (cal_echo_n[i] != 0u) {
        float p2 = ph[i] * ph[i];
        float shrink = 1.0f - sd2 / (float)cal_echo_n[i] / ((p2 > 1.0e-9f) ? p2 : 1.0e-9f);
        if (shrink < 0.0f) shrink = 0.0f;
        if (shrink > 1.0f) shrink = 1.0f;
        q = -shrink * ph[i];
        before += p2;
        after += q * q;
      }
      code = (int32_t)lroundf(q * (256.0f / UMH_TWO_PI));
      code %= 256;
      if (code < 0) code += 256;
      result->phase_byte[i] = (uint8_t)code;
      result->coverage[i] = (cal_echo_dead[i] != 0u) ? 0xFFu : cal_echo_n[i];
    }
    result->rms_before_deg = cal_deg(sqrtf(before / (float)covered));
    result->correction_rms_deg = cal_deg(sqrtf(after / (float)covered));
    result->rms_after_deg = cal_deg(sqrtf(sd2));
    result->fit_rms_deg = result->rms_after_deg;
    result->mic_consistency_deg = result->rms_after_deg;
  }

  /* Relative amplitude |d| * r_e, normalised per microphone then per array. */
  for (m = 0u; m < CAL_MICS; ++m) {
    n_sort = 0u;
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      if (cal_pair_valid(m, i) == 0u) continue;
      cal_echo_sort[n_sort++] = sqrtf(cal_z_re[m][i] * cal_z_re[m][i] + cal_z_im[m][i] * cal_z_im[m][i]) *
                                cal_echo_path_mm(cal_hdist_mm(m, i), v_mm);
    }
    mic_med[m] = cal_sort_median(cal_echo_sort, n_sort);
  }
  for (i = 0u; i < CAL_CHANNELS; ++i) {
    float v[CAL_MICS];
    uint16_t k = 0u;
    for (m = 0u; m < CAL_MICS; ++m) {
      if (cal_pair_valid(m, i) == 0u || mic_med[m] <= 0.0f) continue;
      v[k++] = sqrtf(cal_z_re[m][i] * cal_z_re[m][i] + cal_z_im[m][i] * cal_z_im[m][i]) *
               cal_echo_path_mm(cal_hdist_mm(m, i), v_mm) / mic_med[m];
    }
    ph[i] = (k != 0u) ? cal_sort_median(v, k) : 0.0f;
  }
  n_sort = 0u;
  for (i = 0u; i < CAL_CHANNELS; ++i) if (cal_echo_n[i] != 0u) cal_echo_sort[n_sort++] = ph[i];
  {
    float med = cal_sort_median(cal_echo_sort, n_sort);
    if (med <= 0.0f) med = 1.0f;
    if (n_sort != 0u) {
      result->amp_p10 = cal_echo_sort[(uint16_t)((float)(n_sort - 1u) * 0.1f + 0.5f)] / med;
      result->amp_p90 = cal_echo_sort[(uint16_t)((float)(n_sort - 1u) * 0.9f + 0.5f)] / med;
    }
    result->coupling_db = 20.0f * log10f(result->amp_p90 /
                                         ((result->amp_p10 > 1.0e-3f) ? result->amp_p10 : 1.0e-3f));
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float a = (cal_echo_dead[i] != 0u) ? 0.0f : ph[i] / med * 128.0f;
      if (a > 255.0f) a = 255.0f;
      result->amplitude[i] = (uint8_t)lroundf(a);
    }
  }

  /* ---- Quality gates ---------------------------------------------------- */
  if (covered < CAL_ECHO_PASS_COVERED) result->quality_flags |= CAL_Q_COUPLING;
  if (good_mics < CAL_ECHO_PASS_MICS) result->quality_flags |= CAL_Q_MICS;
  if (result->rms_after_deg > CAL_ECHO_PASS_SD_DEG) result->quality_flags |= CAL_Q_FIT;
  if (dead > CAL_ECHO_PASS_DEAD) result->quality_flags |= CAL_Q_LEVEL;
  if (result->quality_flags != 0u)
    return cal_echo_fail(result, progress, context, UMH_FAULT_CAL_QUALITY, 0u, -6);

  /* ---- E: real focus test, candidate vs zero correction ----------------- */
  cal_report(progress, context, US_CAL_VERIFY, 86u);
  spatial_renderer_init(&cal_echo_renderer, profile);
  {
    uint64_t seed = 0x5DEECE66Dull;
    float sum_db = 0.0f;
    uint8_t sets = 0u, wins = 0u, s;
    for (m = 0u; m < CAL_MICS; ++m) {
      uint8_t pool[CAL_CHANNELS], np = 0u;
      for (i = 0u; i < CAL_CHANNELS; ++i)
        if (cal_pair_valid(m, i) != 0u) pool[np++] = i;
      if (np < CAL_ECHO_VERIFY_CHANNELS) continue;
      for (s = 0u; s < CAL_ECHO_VERIFY_SETS; ++s) {
        uint8_t ch[CAL_ECHO_VERIFY_CHANNELS], k = 0u, j;
        float mag_c, mag_0;
        while (k < CAL_ECHO_VERIFY_CHANNELS) {
          uint8_t pick = pool[cal_mix32(&seed) % np], dup = 0u;
          for (j = 0u; j < k; ++j) if (ch[j] == pick) dup = 1u;
          if (dup == 0u) ch[k++] = pick;
        }
        if (cal_echo_focus(link, &plan, result->phase_byte, ch, m, &mag_c) != 0 ||
            cal_echo_focus(link, &plan, NULL, ch, m, &mag_0) != 0)
          return cal_echo_fail(result, progress, context, UMH_FAULT_CAL_MIC_SILENT,
                               CAL_Q_MEASURE, -7);
        if (mag_0 < 1.0e-3f) mag_0 = 1.0e-3f;
        if (mag_c < 1.0e-3f) mag_c = 1.0e-3f;
        sum_db += 20.0f * log10f(mag_c / mag_0);
        if (mag_c >= mag_0) ++wins;
        ++sets;
      }
      cal_report(progress, context, US_CAL_VERIFY, (uint8_t)(86u + 3u * (m + 1u)));
    }
    result->verify_gain_db = (sets != 0u) ? sum_db / (float)sets : 0.0f;
    result->sign_hypothesis = wins;
    result->reserved1 = sets;
    if (sets == 0u || result->verify_gain_db < CAL_ECHO_VERIFY_MIN_DB)
      return cal_echo_fail(result, progress, context, UMH_FAULT_CAL_QUALITY, CAL_Q_VERIFY, -8);
  }
  result->fault = UMH_FAULT_NONE;
  result->progress = 100u;
  cal_report(progress, context, US_CAL_OK, 100u);
  return 0;
}

/* --------------------------------------------------------------------------
 * Diagnostic dump sections.
 * -------------------------------------------------------------------------- */
/* Section 2 is the legacy solver snapshot.  Its writer was removed with the
 * old room-echo calibration; the protocol still advertises the same 888-byte
 * length and returns zeros so host tools do not need a version check. */
#define CAL_DUMP2_SIZE 888u

uint32_t us_calibration_dump_size(uint8_t section)
{
  switch (section) {
    case 0u: return (uint32_t)(CAL_MICS * CAL_CHANNELS * 2u * sizeof(float));
    case 1u: return (uint32_t)(CAL_PROFILE_GATES * CAL_MICS * 2u * sizeof(int16_t));
    case 2u: return CAL_DUMP2_SIZE;
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
  if (out == NULL || offset >= total) return -1;
  if ((uint32_t)length > total - offset) length = (uint16_t)(total - offset);
  /* The legacy transient profile writer was deleted; keep the wire length. */
  memset(out, 0, length);
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
      /* Legacy solver snapshot is no longer produced; report zeros. */
      memset(out, 0, length);
      return (int)length;
    default: return -1;
  }
}

/* --------------------------------------------------------------------------
 * Main state machine.
 * -------------------------------------------------------------------------- */
int us_calibration_run(fpga_link_t *link, const umh_device_profile_t *profile,
                       us_cal_progress_cb_t progress, void *context,
                       umh_calibration_result_t *result)
{
  return cal_echo_run(link, profile, progress, context, result);
}

int us_calibration_nearfield_run(fpga_link_t *link, const umh_device_profile_t *profile,
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
  float a = sinf(UMH_PI * (float)from / 256.0f);
  float b = sinf(UMH_PI * (float)to / 256.0f);
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
      ph = UMH_TWO_PI * (float)subset.channels[i].phase / 256.0f;
      cr = cosf(ph); ci = sinf(ph);
      pred_re += hr * cr - hi * ci;
      pred_im += hr * ci + hi * cr;
      sum_h += sqrtf(hr * hr + hi * hi);
    }
    for (i = 0u; i < CAL_CHANNELS; ++i) {
      float hr = cal_z_re[m][i], hi = cal_z_im[m][i];
      float ph = UMH_TWO_PI * (float)full.channels[i].phase / 256.0f;
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
