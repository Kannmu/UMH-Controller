#ifndef AUDIO_ENGINE_H
#define AUDIO_ENGINE_H

#include <stdint.h>
#include "cmsis_os.h"
#include "device_profile.h"
#include "spatial_renderer.h"
#include "fpga_link.h"

/* Focused-AM audio extension.
 *
 * The host uploads a 20-byte configuration that selects a fixed spatial
 * focus.  The engine renders the 84 phase bytes once, loads them into the
 * FPGA through the ordinary FRAME path, and then keeps the FPGA level FIFO
 * topped up with common 8-bit envelope levels.  The FPGA pops one level per
 * two carrier periods from its own crystal and rebuilds its inactive event
 * table, so the aperture stays fixed, the carrier amplitude follows the
 * audio, and MCU scheduling jitter never reaches the output timing. */
#define UMH_AUDIO_RING_SIZE 2048u
#define UMH_AUDIO_RING_MASK (UMH_AUDIO_RING_SIZE - 1u)
#define UMH_AUDIO_MIN_RATE_HZ 8000u
#define UMH_AUDIO_MAX_RATE_HZ 20000u
#define UMH_AUDIO_MAX_PREBUFFER UMH_AUDIO_RING_SIZE
/* FIFO fill kept ahead of the FPGA (9.6 ms at 20 kHz): several missed 1 ms
 * service ticks cannot drain it, and fill + one block stays below 256. */
#define UMH_AUDIO_FIFO_TARGET 192u
/* Gain ramp (Q7) used for start, underrun, refill and stop, so every
 * transition is a smooth fade instead of a step in the envelope. */
#define UMH_AUDIO_GAIN_ONE 128u
#define UMH_AUDIO_GAIN_STEP 1u   /* 128 output samples = 6.4 ms per full fade */
#define UMH_AUDIO_MAX_LEVEL 128u
/* Fill-error clock recovery range.  The PC audio clock and the 64 MHz FPGA
 * clock are independent; allow a few thousand ppm so a slightly slow host
 * cannot gradually empty the ring.  The correction is only applied while the
 * fill deviates from the prebuffer target. */
#define UMH_AUDIO_MAX_CORRECTION_PPM 3000
/* The fill arrives in 256-sample packets, so the raw fill is a ~78 Hz
 * sawtooth.  It is low-passed with tau = 2^SHIFT ticks (102 ms at 20 kHz)
 * before it steers the clock; the unfiltered servo swung the playback rate
 * by several hundred ppm per packet, which is audible as a warble. */
#define UMH_AUDIO_FILL_FILTER_SHIFT 11u
#define UMH_AUDIO_FILL_GAIN_PPM 4
/* Host USB scheduling can leave a short gap between packets.  Hold the last
 * envelope for up to 25 ms (output samples at 20 kHz) before fading out as a
 * real underrun; ordinary Windows/USB jitter never inserts an audible zero. */
#define UMH_AUDIO_UNDERRUN_HOLD_TICKS 500u

_Static_assert((UMH_AUDIO_RING_SIZE & (UMH_AUDIO_RING_SIZE - 1u)) == 0u,
               "audio ring size must be a power of two");
_Static_assert(UMH_AUDIO_FIFO_TARGET + FPGA_AUDIO_BLOCK_MAX < FPGA_AUDIO_FIFO_DEPTH,
               "audio FIFO target leaves no room for one block");
_Static_assert(UMH_AUDIO_GAIN_ONE % UMH_AUDIO_GAIN_STEP == 0u,
               "gain ramp must land exactly on 0 and UMH_AUDIO_GAIN_ONE");

typedef enum {
  UMH_AUDIO_OFF = 0u,
  UMH_AUDIO_CONFIGURED = 1u,
  UMH_AUDIO_PRIMING = 2u,
  UMH_AUDIO_RUNNING = 3u,
  UMH_AUDIO_STOPPING = 4u,
  UMH_AUDIO_FAULT = 5u
} umh_audio_state_t;

#define UMH_AUDIO_STATUS_CONFIGURED (1u << 0)
#define UMH_AUDIO_STATUS_PRIMING    (1u << 1)
#define UMH_AUDIO_STATUS_RUNNING    (1u << 2)
#define UMH_AUDIO_STATUS_REFILLING  (1u << 3)
#define UMH_AUDIO_STATUS_UNDERRUN   (1u << 4)

/* AUDIO_CONFIGURE wire payload.  Coordinates are signed micrometres, the
 * envelope rate is a divisor of the host processing chain, and the source
 * level/phase are passed straight to the spatial renderer once. */
typedef struct __attribute__((packed)) {
  int32_t x_um;
  int32_t y_um;
  int32_t z_um;
  uint8_t phase;
  uint8_t level;
  uint16_t envelope_rate_hz;
  uint16_t prebuffer_samples;
  uint16_t flags;
} umh_audio_config_wire_t;

_Static_assert(sizeof(umh_audio_config_wire_t) == 20u, "audio config wire size");

/* Focused-AM cluster extension.  A base config may be followed by
 * extra_point_count and that many 14-byte points.  All points share the same
 * 20 kHz common envelope; the STM32 renders their combined phase image once,
 * so defoaming and beam shaping can cover several foci without any extra
 * real-time bandwidth.  Devices without FOCUSED_AM_MULTI ignore/reject the
 * extension and continue to accept the 20-byte base form. */
#define UMH_AUDIO_MAX_POINTS 8u
#define UMH_AUDIO_MAX_EXTRA_POINTS (UMH_AUDIO_MAX_POINTS - 1u)

typedef struct __attribute__((packed)) {
  int32_t x_um;
  int32_t y_um;
  int32_t z_um;
  uint8_t phase;
  uint8_t level;
} umh_audio_point_wire_t;

_Static_assert(sizeof(umh_audio_point_wire_t) == 14u, "audio point wire size");

typedef struct __attribute__((packed)) {
  uint8_t state;
  uint8_t flags;
  uint16_t ring_fill;
  uint16_t ring_capacity;
  uint16_t prebuffer;
  uint32_t underrun_count;
  uint32_t overrun_count;
  uint32_t packet_loss_count;
  uint32_t rendered_samples;
  int32_t clock_correction_ppm;
  uint32_t max_service_us;
} umh_audio_status_wire_t;

_Static_assert(sizeof(umh_audio_status_wire_t) == 32u, "audio status wire size");

typedef struct {
  volatile umh_audio_state_t state;
  umh_spatial_renderer_t *renderer;
  fpga_link_t *link;
  osMutexId_t lock;
  StaticSemaphore_t lock_memory;

  /* The envelope ring is only live from CONFIGURED on, and the aperture
   * scratch only inside configure (state OFF, lock held), so they share. */
  union {
    uint8_t envelope_ring[UMH_AUDIO_RING_SIZE];
    struct {
      float spatial_real[UMH_DEVICE_CHANNEL_COUNT];
      float spatial_imag[UMH_DEVICE_CHANNEL_COUNT];
      uint8_t aperture_phase[UMH_DEVICE_CHANNEL_COUNT];
      uint8_t aperture_enable[UMH_DEVICE_CHANNEL_COUNT];
    };
  };
  volatile uint16_t ring_head;
  volatile uint16_t ring_tail;
  uint16_t prebuffer_samples;
  uint8_t max_level;
  uint32_t base_step_q16;       /* host rate / FPGA output rate, Q16 */

  /* Per-session state from here to the end is cleared by configure. */
  uint32_t frac_q16;
  uint32_t step_q16;
  int32_t fill_error_q12;       /* low-passed (fill - prebuffer), Q12 */
  int32_t clock_correction_ppm;
  uint32_t underrun_count;
  uint32_t overrun_count;
  uint32_t packet_loss_count;
  uint32_t rendered_samples;
  uint32_t max_service_cycles;
  uint32_t expected_sequence;
  uint16_t underrun_grace;
  uint8_t current_sample;
  uint8_t next_sample;
  uint8_t gain;                 /* Q7 output fade, 0..UMH_AUDIO_GAIN_ONE */
  uint8_t waiting_refill;
  uint8_t sequence_valid;
  uint8_t poll_tick;           /* full status poll every 16 service calls */
} umh_audio_engine_t;

void audio_engine_init(umh_audio_engine_t *engine);
int audio_engine_configure(umh_audio_engine_t *engine,
                           umh_spatial_renderer_t *renderer,
                           fpga_link_t *link,
                           const uint8_t *payload,
                           uint16_t length);
int audio_engine_start(umh_audio_engine_t *engine);
int audio_engine_feed(umh_audio_engine_t *engine, const uint8_t *levels,
                      uint16_t length, uint32_t stream_sequence);
void audio_engine_request_stop(umh_audio_engine_t *engine);
void audio_engine_abort(umh_audio_engine_t *engine);
uint8_t audio_engine_owns_output(const umh_audio_engine_t *engine);
/* Call about once per millisecond: tops the FPGA level FIFO up to
 * UMH_AUDIO_FIFO_TARGET and finishes a requested stop once it has drained. */
void audio_engine_service(umh_audio_engine_t *engine);
void audio_engine_get_status(const umh_audio_engine_t *engine,
                             umh_audio_status_wire_t *status);

#endif
