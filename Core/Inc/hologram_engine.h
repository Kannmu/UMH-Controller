#ifndef HOLOGRAM_ENGINE_H
#define HOLOGRAM_ENGINE_H

#include <stdint.h>
#include "cmsis_os.h"
#include "device_profile.h"
#include "fpga_link.h"

/* Hologram keyframe executor.
 *
 * The block path renders a compiled timeline, and motion_engine synthesises one
 * spatial point per frame.  This engine fills the third gap: it holds a small
 * set of complete 84-channel holograms (phase + level per channel) uploaded by
 * the host, and interpolates between consecutive keyframes on its own clock,
 * submitting the result through the ordinary fpga_link frame transaction.
 *
 * Why it exists: a shape morph has to keep the field continuous for seconds.
 * Streaming every interpolated frame from the host works, but any USB or host
 * hiccup shows up on the water as a jump, and the 25 ms FPGA link watchdog
 * turns a longer stall into a full blackout.  With the keyframes resident on
 * the device, only the upload has to be reliable.
 *
 * What it deliberately does NOT do: no rendering, no calibration maths, no new
 * FPGA command, no second SPI path.  Keyframe values are exactly the bytes a
 * CHANNEL_STATE record would carry, so anything the host can compute for the
 * block path it can also hand to this engine.
 *
 * Timing rules that matter on this board:
 *   - the level byte is slots of high time in a 256 slot carrier, so a full
 *     spatial amplitude is level 128 (50 % duty).  Interpolated levels are
 *     clamped to the configured ceiling, never to 255.
 *   - phase is interpolated along the shortest 8-bit wrap path, the same
 *     semantics the block parser uses for its DELTA encoding.
 *   - waiting longer than ~5 ms without touching SPI risks the 25 ms link
 *     watchdog clearing all 84 outputs, so the service routine polls link
 *     status while it idles instead of sleeping blindly.
 */

/* 10 keyframes cost 10 * 168 = 1680 bytes of BSS, which is what the 112 KiB
 * STM32G491 budget can still afford next to the frame ring, the block parser
 * and the USB rings.  A morph normally needs far fewer: the host does the heavy
 * interpolation itself and only hands over the shapes it wants blended, so a
 * cycle of several letters fits comfortably. */
#define UMH_HOLOGRAM_MAX_KEYFRAMES   10u
#define UMH_HOLOGRAM_CHANNELS        UMH_DEVICE_CHANNEL_COUNT
#define UMH_HOLOGRAM_KEYFRAME_BYTES  (2u * UMH_HOLOGRAM_CHANNELS)

#define UMH_HOLOGRAM_WIRE_VERSION    1u

#define UMH_HOLOGRAM_MIN_RATE_HZ     10u
#define UMH_HOLOGRAM_MAX_RATE_HZ     500u
#define UMH_HOLOGRAM_MAX_TRANSITION_MS 60000u
/* Frames that repeat the previous wire state are suppressed, but the FPGA link
 * watchdog is 25 ms, so a fully static hologram is refreshed well inside that. */
#define UMH_HOLOGRAM_REFRESH_MS      100u
#define UMH_HOLOGRAM_STOP_FADE_MS    20u

/* Message group.  The v7 groups 0x62..0x67 (motion) and 0x90..0x94 (focused
 * AM audio) are untouched; this is a new capability behind a new capability
 * bit, so an old host keeps working unchanged. */
#define UMH_MSG_HOLOGRAM_UPLOAD      0xA0u
#define UMH_MSG_HOLOGRAM_CONFIG      0xA1u
#define UMH_MSG_HOLOGRAM_START       0xA2u
#define UMH_MSG_HOLOGRAM_STOP        0xA3u
#define UMH_MSG_HOLOGRAM_STATUS      0xA5u

typedef enum {
  UMH_HOLOGRAM_STATE_OFF = 0u,
  UMH_HOLOGRAM_STATE_READY = 1u,
  UMH_HOLOGRAM_STATE_RUNNING = 2u,
  UMH_HOLOGRAM_STATE_STOPPING = 3u,
  UMH_HOLOGRAM_STATE_FAULT = 4u
} umh_hologram_state_t;

#define UMH_HOLOGRAM_FLAG_RUNNING    (1u << 0)
#define UMH_HOLOGRAM_FLAG_HOLD_LAST  (1u << 1)  /* do not wrap after keyframe N */
#define UMH_HOLOGRAM_FLAG_GAMMA      (1u << 2)  /* apply the level gamma LUT */
#define UMH_HOLOGRAM_FLAG_STATIC     (1u << 3)  /* single keyframe, never morph */

/* UPLOAD: 12 byte header followed by keyframe_count * 168 bytes.
 *   uint8  version          must be UMH_HOLOGRAM_WIRE_VERSION
 *   uint8  flags            reserved, must be 0
 *   uint16 first_index      destination slot of the first keyframe
 *   uint16 keyframe_count   1..11 (11 * 168 + 12 = 1860 <= 2048 payload)
 *   uint8  reserved[6]
 *   keyframe: uint8 phase[84]; uint8 level[84]
 * Chunks may arrive in any order; only the slots written are changed, so the
 * host can build a long sequence with several UPLOAD messages. */
typedef struct __attribute__((packed)) {
  uint8_t  version;
  uint8_t  flags;
  uint16_t first_index;
  uint16_t keyframe_count;
  uint8_t  reserved[6];
} umh_hologram_upload_wire_t;

_Static_assert(sizeof(umh_hologram_upload_wire_t) == 12u, "hologram upload header size");

#define UMH_HOLOGRAM_UPLOAD_MAX_PER_MSG 11u

/* CONFIG: 14 bytes.
 *   uint8  version          must be 1
 *   uint8  flags            UMH_HOLOGRAM_FLAG_*
 *   uint16 output_rate_hz   10..500 submit cadence
 *   uint16 loop_ms          0 = derive the cycle from the transition time
 *   uint16 transition_ms    interpolation time between consecutive keyframes
 *   uint8  level            global amplitude scale, 0..128
 *   uint8  keyframes        1..configured keyframe count
 *   uint8  reserved[2] */
typedef struct __attribute__((packed)) {
  uint8_t  version;
  uint8_t  flags;
  uint16_t output_rate_hz;
  uint16_t loop_ms;
  uint16_t transition_ms;
  uint8_t  level;
  uint8_t  keyframes;
  uint8_t  reserved[2];
} umh_hologram_config_wire_t;

/* Wire-layout guard.  The negative-array trick makes the compiler print the
 * real size when the layout ever changes, which _Static_assert cannot do. */
typedef char umh_hologram_config_size_guard[
    sizeof(umh_hologram_config_wire_t) == 12u ? 1 : -1];
/* STATUS: 42 bytes, laid out so the timing fields sit where MOTION_STATUS puts
 * them, which makes the two engines directly comparable on the bench. */
typedef struct __attribute__((packed)) {
  uint8_t  state;               /* umh_hologram_state_t */
  uint8_t  flags;
  uint16_t output_rate_hz;
  uint16_t keyframe_count;
  uint16_t current_keyframe;
  uint16_t alpha_x1000;         /* progress inside the current segment, 0..1000 */
  uint16_t service_max_us;
  uint16_t service_avg_us;
  uint16_t render_max_us;
  uint16_t render_avg_us;
  uint16_t submit_max_us;
  uint16_t submit_avg_us;
  uint32_t frames;              /* frames actually written to the FPGA */
  uint32_t skipped;             /* frames suppressed because nothing changed */
  uint32_t missed_deadlines;
  uint32_t frame_errors;
  uint32_t fps_x100;
} umh_hologram_status_wire_t;

typedef char umh_hologram_status_size_guard[
    sizeof(umh_hologram_status_wire_t) == 42u ? 1 : -1];

typedef struct {
  uint8_t state;
  uint8_t flags;
  uint8_t configured;           /* CONFIG accepted at least once */
  uint8_t stop_requested;       /* a stop arrived while a fade-out was running */
  uint16_t output_rate_hz;
  uint16_t loop_ms;             /* 0 when derived from the transition time */
  uint16_t transition_ms;
  uint8_t level;                /* global scale 0..128 */
  uint8_t keyframes;
  uint8_t keyframe_valid[UMH_HOLOGRAM_MAX_KEYFRAMES];
  uint8_t phase[UMH_HOLOGRAM_MAX_KEYFRAMES][UMH_HOLOGRAM_CHANNELS];
  uint8_t level_table[UMH_HOLOGRAM_MAX_KEYFRAMES][UMH_HOLOGRAM_CHANNELS];

  uint8_t  last_phase[UMH_HOLOGRAM_CHANNELS];
  uint8_t  last_level[UMH_HOLOGRAM_CHANNELS];
  uint8_t  have_last;

  uint64_t run_origin_us;       /* start of the current cycle */
  uint64_t period_us;
  uint64_t next_due_us;
  uint64_t last_service_us;
  uint64_t last_submit_us;

  uint32_t frame_sequence;
  uint32_t frames_submitted;
  uint32_t frames_skipped;
  uint32_t missed_deadlines;
  uint32_t frame_errors;
  uint32_t fps_frames;
  uint32_t fps;
  uint64_t fps_window_start_us;

  float fade_scale;
  float fade_step;

  uint32_t service_max_cycles;
  uint32_t service_sum_cycles;
  uint32_t service_count;
  uint32_t render_max_cycles;
  uint32_t render_sum_cycles;
  uint32_t submit_max_cycles;
  uint32_t submit_sum_cycles;

  osMutexId_t lock;
  StaticSemaphore_t lock_memory;
} umh_hologram_engine_t;

void hologram_engine_init(umh_hologram_engine_t *engine);

/* Wire handlers.  Return 0 on success, negative on a rejected payload; the
 * caller maps that to UMH_STATUS_BAD_LENGTH / UMH_STATUS_INVALID_STATE. */
int hologram_engine_upload(umh_hologram_engine_t *engine, const uint8_t *payload,
                           uint16_t length);
int hologram_engine_configure(umh_hologram_engine_t *engine, const uint8_t *payload,
                              uint16_t length);

void hologram_engine_start(umh_hologram_engine_t *engine);
void hologram_engine_request_stop(umh_hologram_engine_t *engine);
void hologram_engine_abort(umh_hologram_engine_t *engine, fpga_link_t *link);

uint8_t hologram_engine_owns_output(const umh_hologram_engine_t *engine);
uint8_t hologram_engine_is_active(const umh_hologram_engine_t *engine);

/* Called from the render task only.  `wait_us` receives the delay the caller
 * should sleep before servicing again. */
uint32_t hologram_engine_service(umh_hologram_engine_t *engine, fpga_link_t *link,
                                 uint64_t now_us, uint32_t *wait_us);

void hologram_engine_get_status(const umh_hologram_engine_t *engine,
                                umh_hologram_status_wire_t *status);

#endif
