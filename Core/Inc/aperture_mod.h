#ifndef APERTURE_MOD_H
#define APERTURE_MOD_H

#include <stdint.h>
#include "cmsis_os.h"
#include "device_profile.h"
#include "spatial_renderer.h"
#include "fpga_link.h"

/* Modulated-aperture stream.
 *
 * The device has three ways to produce a field, and they differ only in how
 * often the spatial image changes:
 *
 *   frame playback   the host or an engine hands over a complete 84-channel
 *                    image per output frame                 (frame_ring path)
 *   path motion      one or two foci are synthesised per frame (motion_engine)
 *   this engine      ONE fixed spatial image, whose common amplitude is
 *                    modulated by an uploaded scalar envelope at a much higher
 *                    rate than any frame transaction could carry
 *
 * That last case is what this module implements, and it is deliberately named
 * after the mechanism rather than after any one use of it.  The envelope is a
 * stream of 8-bit common-amplitude samples; what a host chooses to put in that
 * stream - an audible waveform, a tactile pattern, an amplitude ramp, a noise
 * probe - is not the device's business.
 *
 * How it works: the 84 phase bytes are rendered once through the ordinary
 * spatial renderer, loaded into the FPGA through the ordinary FRAME path, and
 * the FPGA is then switched to common-level mode.  From that point the MCU only
 * tops up a level FIFO; the FPGA pops one level per two carrier periods from
 * its own crystal and rebuilds its inactive event table, so the aperture stays
 * fixed, the common amplitude follows the envelope, and MCU scheduling jitter
 * never reaches the output timing.
 *
 * The envelope is resampled from the host's rate to the FPGA output rate with a
 * linear interpolator and a fill-error clock servo, so an independent host
 * clock can drift for minutes without an audible or visible discontinuity. */
#define UMH_MOD_RING_SIZE 2048u
#define UMH_MOD_RING_MASK (UMH_MOD_RING_SIZE - 1u)
#define UMH_MOD_MIN_RATE_HZ 8000u
#define UMH_MOD_MAX_RATE_HZ 20000u
#define UMH_MOD_MAX_PREBUFFER UMH_MOD_RING_SIZE
/* FIFO fill kept ahead of the FPGA (9.6 ms at 20 kHz): several missed 1 ms
 * service ticks cannot drain it, and fill + one block stays below 256. */
#define UMH_MOD_FIFO_TARGET 192u
/* Gain ramp (Q7) used for start, underrun, refill and stop, so every
 * transition is a smooth fade instead of a step in the common amplitude. */
#define UMH_MOD_GAIN_ONE 128u
#define UMH_MOD_GAIN_STEP 1u    /* 128 output samples = 6.4 ms per full fade */
#define UMH_MOD_MAX_LEVEL 128u
/* Fill-error clock recovery range.  The host envelope clock and the 64 MHz FPGA
 * clock are independent; allow a few thousand ppm so a slightly slow host
 * cannot gradually empty the ring.  The correction is only applied while the
 * fill deviates from the prebuffer target. */
#define UMH_MOD_MAX_CORRECTION_PPM 3000
/* The fill arrives in 256-sample packets, so the raw fill is a ~78 Hz
 * sawtooth.  It is low-passed with tau = 2^SHIFT ticks (102 ms at 20 kHz)
 * before it steers the clock; the unfiltered servo swung the playback rate
 * by several hundred ppm per packet, which is a slow periodic warble. */
#define UMH_MOD_FILL_FILTER_SHIFT 11u
#define UMH_MOD_FILL_GAIN_PPM 4
/* Host USB scheduling can leave a short gap between packets.  Hold the last
 * envelope sample for up to 25 ms (output samples at 20 kHz) before fading out
 * as a real underrun; ordinary Windows/USB jitter never inserts an audible
 * zero. */
#define UMH_MOD_UNDERRUN_HOLD_TICKS 500u

_Static_assert((UMH_MOD_RING_SIZE & (UMH_MOD_RING_SIZE - 1u)) == 0u,
               "envelope ring size must be a power of two");
_Static_assert(UMH_MOD_FIFO_TARGET + FPGA_MOD_BLOCK_MAX < FPGA_MOD_FIFO_DEPTH,
               "level FIFO target leaves no room for one block");
_Static_assert(UMH_MOD_GAIN_ONE % UMH_MOD_GAIN_STEP == 0u,
               "gain ramp must land exactly on 0 and UMH_MOD_GAIN_ONE");

typedef enum {
  UMH_MOD_OFF = 0u,
  UMH_MOD_CONFIGURED = 1u,
  UMH_MOD_PRIMING = 2u,
  UMH_MOD_RUNNING = 3u,
  UMH_MOD_STOPPING = 4u,
  UMH_MOD_FAULT = 5u
} umh_mod_state_t;

#define UMH_MOD_STATUS_CONFIGURED (1u << 0)
#define UMH_MOD_STATUS_PRIMING    (1u << 1)
#define UMH_MOD_STATUS_RUNNING    (1u << 2)
#define UMH_MOD_STATUS_REFILLING  (1u << 3)
#define UMH_MOD_STATUS_UNDERRUN   (1u << 4)

#define UMH_MOD_CONFIG_PLANE_WAVE (1u << 0)

/* MOD_CONFIGURE wire payload.  Coordinates are signed micrometres, the envelope
 * rate is the host's sample rate for the amplitude stream, and the source
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
} umh_mod_config_wire_t;

_Static_assert(sizeof(umh_mod_config_wire_t) == 20u, "modulation config wire size");

/* Multi-focus extension.  A base config may be followed by extra_point_count
 * and that many 14-byte points.  All points share the same common envelope; the
 * device renders their combined phase image once, so a multi-focus field, a
 * beam-shaping pattern or a defoaming pattern needs no extra real-time
 * bandwidth.  Devices without this capability ignore/reject the extension and
 * continue to accept the 20-byte base form. */
#define UMH_MOD_MAX_POINTS 8u
#define UMH_MOD_MAX_EXTRA_POINTS (UMH_MOD_MAX_POINTS - 1u)

typedef struct __attribute__((packed)) {
  int32_t x_um;
  int32_t y_um;
  int32_t z_um;
  uint8_t phase;
  uint8_t level;
} umh_mod_point_wire_t;

_Static_assert(sizeof(umh_mod_point_wire_t) == 14u, "modulation point wire size");

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
} umh_mod_status_wire_t;

_Static_assert(sizeof(umh_mod_status_wire_t) == 32u, "modulation status wire size");

typedef struct {
  volatile umh_mod_state_t state;
  umh_spatial_renderer_t *renderer;
  fpga_link_t *link;
  osMutexId_t lock;
  StaticSemaphore_t lock_memory;

  /* The envelope ring is only live from CONFIGURED on, and the aperture
   * scratch only inside configure (state OFF, lock held), so they share. */
  union {
    uint8_t envelope_ring[UMH_MOD_RING_SIZE];
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
  uint8_t envelope_ceiling;
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
  uint8_t gain;                 /* Q7 output fade, 0..UMH_MOD_GAIN_ONE */
  uint8_t waiting_refill;
  uint8_t sequence_valid;
  uint8_t poll_tick;            /* full status poll every 16 service calls */
} umh_aperture_mod_t;

void aperture_mod_init(umh_aperture_mod_t *engine);
int aperture_mod_configure(umh_aperture_mod_t *engine,
                           umh_spatial_renderer_t *renderer,
                           fpga_link_t *link,
                           const uint8_t *payload,
                           uint16_t length);
int aperture_mod_start(umh_aperture_mod_t *engine);
int aperture_mod_feed(umh_aperture_mod_t *engine, const uint8_t *samples,
                      uint16_t length, uint32_t stream_sequence);
void aperture_mod_request_stop(umh_aperture_mod_t *engine);
void aperture_mod_abort(umh_aperture_mod_t *engine);
uint8_t aperture_mod_owns_output(const umh_aperture_mod_t *engine);
/* Call about once per millisecond: tops the FPGA level FIFO up to
 * UMH_MOD_FIFO_TARGET and finishes a requested stop once it has drained. */
void aperture_mod_service(umh_aperture_mod_t *engine);
void aperture_mod_get_status(const umh_aperture_mod_t *engine,
                             umh_mod_status_wire_t *status);

#endif
