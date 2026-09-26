#ifndef MOTION_ENGINE_H
#define MOTION_ENGINE_H

#include <stdint.h>
#include "cmsis_os.h"
#include "device_profile.h"
#include "spatial_renderer.h"
#include "fpga_link.h"

/* Real-time focus motion generator.
 *
 * This is deliberately not a second block player.  The block path is a
 * compiled timeline rendered by the protocol task; motion_engine lives in the
 * render task and synthesises exactly one spatial field per output frame from
 * a compact path or a live target.  That keeps high-rate levitation/POV and
 * low-latency manual control off the USB hot path while still using the same
 * spatial_renderer calibration and the same FPGA frame transaction as every
 * other producer. */

#define UMH_MOTION_MAX_POINTS 255u
#define UMH_MOTION_MAX_PATTERN_SOURCES 8u
#define UMH_MOTION_PALETTE_SIZE 16u
#define UMH_MOTION_MIN_RATE_HZ 50u
#define UMH_MOTION_MAX_RATE_HZ 2000u
#define UMH_MOTION_STOP_FADE_MS 20u
#define UMH_MOTION_COORD_10UM_MAX 32767
/* palette bit7 marks a zero-transit jump to the next path point.  The low four
 * bits remain the RGB palette index. */
#define UMH_MOTION_POINT_JUMP 0x80u

typedef enum {
  UMH_MOTION_MODE_PATH = 0u,
  UMH_MOTION_MODE_LIVE = 1u
} umh_motion_mode_t;

typedef enum {
  UMH_MOTION_STATE_OFF = 0u,
  UMH_MOTION_STATE_READY = 1u,
  UMH_MOTION_STATE_RUNNING = 2u,
  UMH_MOTION_STATE_STOPPING = 3u,
  UMH_MOTION_STATE_FAULT = 4u
} umh_motion_state_t;

#define UMH_MOTION_FLAG_PAUSED       (1u << 0)
#define UMH_MOTION_FLAG_DIRECT       (1u << 1)
#define UMH_MOTION_FLAG_RGB          (1u << 2)
#define UMH_MOTION_FLAG_LOOP         (1u << 3)
#define UMH_MOTION_FLAG_LINEAR       (1u << 4)
#define UMH_MOTION_FLAG_TRAP_PATTERN (1u << 5)
#define UMH_MOTION_FLAG_STEP         (1u << 6)
#define UMH_MOTION_FLAG_ULM          (1u << 7)

/* trap_mode 6: single-sided dark-core vortex levitation. */
#define UMH_MOTION_TRAP_DARK_VORTEX 6u

/* Acoustic vortex (foam suppression) programs.
 *
 * A vortex beam carries orbital angular momentum.  Every channel radiates the
 * same amplitude, but the carrier phase is wound helically around the beam
 * axis by |l| full turns, so the wavefront leaving the array is a spiral and
 * the beam itself is a hollow tube with a pressure null on its axis.  Where
 * it crosses the foam layer the ring of maximum intensity drives the bubble
 * film tangentially and shears the lamellae apart instead of pinning them to
 * one spot the way a plain focus does, and the hollow core keeps the peak
 * pressure off the axis, where the liquid bulk is easiest to cavitate.
 *
 * Both programs use topological charge |l| = 1 and differ only in how the
 * sign of the charge evolves:
 *
 *   STEADY  one fixed spiral, l = +1, for as long as the program runs.  The
 *           phase field is constant, so the FPGA event table is written once,
 *           every carrier period adds up coherently, and the frames in
 *           between are suppressed by the hold optimisation.
 *   ALT     the charge flips between +1 and -1 every half of a 20 ms period,
 *           so the orbital motion reverses direction 100 times a second.
 *           The foam never settles into the steady tangential flow the
 *           single-direction beam builds up: each reversal launches a shear
 *           wave through the lamellae at the moment they had started to move
 *           with the field, which is the stronger defoaming action.  Nothing
 *           else is modulated -- the focus, the amplitude and the frame
 *           cadence are identical to STEADY, and only the winding sense of
 *           the phase changes, so inside a half period the frame is still
 *           constant and only the two reversal edges cost an SPI write.
 *
 * Both reuse the ordinary spatial renderer, calibration and FPGA frame
 * transaction; only the phase field differs from a host motion program. */
typedef enum {
  UMH_VORTEX_OFF = 0u,
  /* l = +1, held for the whole run. */
  UMH_VORTEX_STEADY = 1u,
  /* l = +1 / l = -1, one half of UMH_VORTEX_ALT_PERIOD_US each. */
  UMH_VORTEX_ALT = 2u
} umh_vortex_program_t;

/* Magnitude of the topological charge both programs wind: one full phase turn
 * per turn of azimuth around the beam axis.  ALT flips its sign, STEADY does
 * not. */
#define UMH_VORTEX_TURNS       1u
/* Focal plane of the vortex: the foam layer standing on the liquid surface
 * 100 mm above the array PCB, i.e. the (0, 0, 100 mm) working point the GUI
 * shows as FOCUS.  Focusing there puts the intensity ring in the foam, which
 * is what carries the angular momentum into the lamellae. */
#define UMH_VORTEX_FOCUS_Z_UM  100000
/* Full source amplitude.  spatial_renderer_finalize maps a full spatial
 * magnitude to level 128, i.e. 50 % duty, which is the real full power of
 * this hardware. */
#define UMH_VORTEX_LEVEL       255u
/* Frame cadence.  A static field only needs its link watchdog fed, and the
 * alternating one only has to place two edges per period, so both run at this
 * rate and the hold optimisation suppresses every frame that repeats one. */
#define UMH_VORTEX_RATE_HZ     200u
/* Reversal period of the alternating program: 20 ms, i.e. 10 ms of +1 and
 * 10 ms of -1, so the spin turns the other way 100 times a second. */
#define UMH_VORTEX_ALT_PERIOD_US 20000u
/* Frames per half period.  The alternation is counted in frames rather than
 * microseconds because a wall-clock flip read on the 5 ms frame grid can land
 * a few hundred microseconds either side of the boundary and split one period
 * 15 ms / 5 ms; counting frames makes the two halves equal by construction and
 * only moves the reversal by the frame period. */
#define UMH_VORTEX_ALT_HALF_FRAMES 2u
_Static_assert(UMH_VORTEX_ALT_HALF_FRAMES * 2000000u ==
               UMH_VORTEX_RATE_HZ * UMH_VORTEX_ALT_PERIOD_US,
               "alternating vortex half period must be whole frames");

/* A path point is deliberately compact: signed 10 um units cover +-327 mm
 * with 10 um resolution, which is far below the 40 kHz phase resolution. */
typedef struct __attribute__((packed)) {
  int16_t x_10um;
  int16_t y_10um;
  int16_t z_10um;
  uint8_t level;
  uint8_t palette;
} umh_motion_point_wire_t;

_Static_assert(sizeof(umh_motion_point_wire_t) == 8u, "motion point wire size");

typedef struct __attribute__((packed)) {
  uint8_t mode;                 /* 0=PATH, 1=LIVE */
  uint8_t flags;                /* UMH_MOTION_FLAG_* */
  uint16_t output_rate_hz;      /* 50..2000 */
  uint16_t loop_ms;             /* path traversal period */
  uint16_t max_speed_mm_s;      /* tracker limit, direct mode ignores it */
  uint16_t max_accel_mm_s2;     /* tracker limit, direct mode ignores it */
  int16_t z_offset_10um;        /* added to every path z coordinate */
  uint8_t level;                /* global 0..255 source scale */
  uint8_t trap_mode;            /* 0 single, 1 twin-z, 2 twin-y, 3 ring, 4 vortex, 5 dual-ring */
  uint16_t trap_radius_10um;    /* ring radius / twin separation / dual-ring radius */
  uint8_t trap_phase_span;      /* vortex phase span in carrier units, 0..255 */
  uint8_t palette_count;        /* 0 disables RGB palette output */
  uint8_t palette[UMH_MOTION_PALETTE_SIZE][3];
  /* reserved[0] advances the palette position in 0.1 palette steps per
   * second so a running trajectory can show a time-varying gradient without
   * host traffic.  reserved[1..2] is a signed turntable rate in 0.001 rad/s
   * that rotates the whole path around the array z axis in real time. */
  uint8_t palette_spin_x10;
  int16_t path_spin_mrad_s;
} umh_motion_config_wire_v1_t;

_Static_assert(sizeof(umh_motion_config_wire_v1_t) == 69u, "motion config v1 wire size");

/* Extension added for ULM tactile modulation.  The ULM field is a transverse
 * spatial oscillation of the focus added on top of the macro path: the path
 * centre moves as before while the emitted focus oscillates at the sensitive
 * skin frequency.  UMH_MOTION_FLAG_ULM bit7 selects it, so old 69-byte host
 * configs stay accepted and leave the feature disabled. */
typedef struct __attribute__((packed)) {
  umh_motion_config_wire_v1_t base;
  uint16_t ulm_frequency_hz;
  uint16_t ulm_amplitude_10um;
  uint16_t ulm_wave_speed_mm_s;
  uint8_t  ulm_axis;
} umh_motion_config_wire_t;

_Static_assert(sizeof(umh_motion_config_wire_t) == 76u, "motion config wire size");

typedef struct __attribute__((packed)) {
  uint8_t version;              /* must be 1 */
  uint8_t flags;                /* reserved, must be 0 */
  uint16_t point_count;         /* 0..UMH_MOTION_MAX_POINTS */
  uint8_t reserved[4];
  /* umh_motion_point_wire_t points[point_count] follows */
} umh_motion_upload_wire_t;

_Static_assert(sizeof(umh_motion_upload_wire_t) == 8u, "motion upload wire size");

typedef struct __attribute__((packed)) {
  uint8_t state;                /* umh_motion_state_t */
  uint8_t flags;                /* current UMH_MOTION_FLAG_* */
  uint16_t output_rate_hz;
  uint16_t path_points;
  uint16_t loop_ms;
  int32_t x_um;                 /* planner output, for host visualisation */
  int32_t y_um;
  int32_t z_um;
  uint8_t level;                /* instantaneous source level 0..255 */
  uint8_t trap_mode;
  uint16_t reserved;            /* keep the 32-bit counters word aligned */
  uint16_t service_max_us;      /* worst frame generation + SPI submit time */
  uint16_t service_avg_us;      /* averaged over the current run */
  uint16_t render_max_us;       /* worst spatial synthesis time */
  uint16_t render_avg_us;       /* averaged spatial synthesis time */
  uint16_t submit_max_us;       /* worst FPGA frame transaction time */
  uint16_t submit_avg_us;       /* averaged FPGA frame transaction time */
  uint32_t frames;              /* submitted frames */
  uint32_t missed_deadlines;
  uint32_t frame_errors;
  uint32_t fps_x100;
} umh_motion_status_wire_t;

_Static_assert(sizeof(umh_motion_status_wire_t) == 52u, "motion status wire size");

typedef struct {
  uint8_t state;
  uint8_t configured;
  uint8_t mode;
  uint8_t flags;
  uint16_t output_rate_hz;
  uint32_t period_us;
  uint32_t loop_us;
  uint32_t max_speed_um_s;
  uint32_t max_accel_um_s2;
  int32_t z_offset_um;
  uint8_t level;
  uint8_t trap_mode;
  int32_t trap_radius_um;
  uint8_t trap_phase_span;
  uint8_t palette_count;
  uint8_t palette[UMH_MOTION_PALETTE_SIZE][3];
  uint8_t palette_spin_x10;
  int16_t path_spin_mrad_s;
  uint16_t ulm_frequency_hz;
  uint16_t ulm_wave_speed_mm_s;
  int32_t ulm_amplitude_um;
  uint8_t ulm_axis;
  uint8_t ulm_reserved;
  float ulm_phase;
  float ulm_tangent_x;
  float ulm_tangent_y;
  uint8_t ulm_tangent_valid;
  /* Vortex program selection, UMH_VORTEX_OFF for ordinary motion. */
  uint8_t vortex_program;
  /* Frames emitted since the run started, which is what the alternating
   * program counts its half periods in. */
  uint32_t vortex_frames;
  float spin_angle;
  float palette_spin_phase;

  umh_motion_point_wire_t points[UMH_MOTION_MAX_POINTS];
  uint16_t point_count;

  float pattern_offsets[UMH_MOTION_MAX_PATTERN_SOURCES][3];
  uint8_t pattern_phases[UMH_MOTION_MAX_PATTERN_SOURCES];
  uint8_t pattern_count;

  float pos_x_um;
  float pos_y_um;
  float pos_z_um;
  float vel_x_um_s;
  float vel_y_um_s;
  float vel_z_um_s;
  float target_x_um;
  float target_y_um;
  float target_z_um;
  uint8_t target_level;
  uint8_t target_palette;
  uint8_t target_valid;
  uint8_t current_valid;

  float phase;                  /* path position 0..1 */
  float fade_scale;
  float fade_step;
  float last_level_scale;
  uint8_t last_level;
  uint8_t last_palette;
  uint8_t stop_requested;

  uint64_t last_service_us;
  uint64_t next_due_us;
  uint32_t frame_sequence;
  uint32_t frames_submitted;
  uint32_t missed_deadlines;
  uint32_t frame_errors;
  uint32_t fps_frames;
  uint32_t fps;
  uint64_t fps_window_start_us;
  uint32_t service_max_cycles;
  uint32_t service_sum_cycles;
  uint32_t service_count;
  uint32_t service_last_cycles;
  uint32_t render_max_cycles;
  uint32_t render_sum_cycles;
  uint32_t submit_max_cycles;
  uint32_t submit_sum_cycles;

  /* Static scratch keeps the 84-channel renderer out of the 4 KiB render
   * task stack. */
  float real_accum[UMH_DEVICE_CHANNEL_COUNT];
  float imag_accum[UMH_DEVICE_CHANNEL_COUNT];

  osMutexId_t lock;
  StaticSemaphore_t lock_memory;
} umh_motion_engine_t;

void motion_engine_init(umh_motion_engine_t *engine);
int motion_engine_upload(umh_motion_engine_t *engine, const uint8_t *payload,
                         uint16_t length);
int motion_engine_configure(umh_motion_engine_t *engine, const uint8_t *payload,
                            uint16_t length);
int motion_engine_target(umh_motion_engine_t *engine, const uint8_t *payload,
                         uint16_t length);
int motion_engine_configure_levitation(umh_motion_engine_t *engine,
                                       uint8_t level, int32_t trap_z_um);
int motion_engine_configure_vortex(umh_motion_engine_t *engine, uint8_t program);
int motion_engine_start(umh_motion_engine_t *engine);
void motion_engine_request_stop(umh_motion_engine_t *engine);
void motion_engine_abort(umh_motion_engine_t *engine, fpga_link_t *link);
uint8_t motion_engine_owns_output(const umh_motion_engine_t *engine);
uint8_t motion_engine_uses_rgb(const umh_motion_engine_t *engine);
uint8_t motion_engine_is_active(const umh_motion_engine_t *engine);
uint8_t motion_engine_trap_mode(const umh_motion_engine_t *engine);
uint8_t motion_engine_vortex_program(const umh_motion_engine_t *engine);
uint32_t motion_engine_service(umh_motion_engine_t *engine,
                               umh_spatial_renderer_t *renderer,
                               fpga_link_t *link, uint64_t now_us,
                               uint32_t *wait_us);
void motion_engine_get_status(const umh_motion_engine_t *engine,
                              umh_motion_status_wire_t *status);

#endif


