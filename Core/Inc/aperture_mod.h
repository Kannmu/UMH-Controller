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
/* The level FIFO in front of the FPGA is part of the same pipe, so the usable
 * ring depth is one FIFO behind the requested prebuffer.  Reserving it here
 * (a) makes the clock servo's zero-error point coincide with the fill that the
 * stream actually settles at, instead of biasing the playback rate by
 * FIFO_TARGET * UMH_MOD_FILL_GAIN_PPM forever, and (b) stops a prebuffer that
 * leaves no room for one FPGA block from overflowing the ring. */
#define UMH_MOD_MAX_PREBUFFER (UMH_MOD_RING_SIZE - 2u * FPGA_MOD_BLOCK_MAX)
/* FIFO fill kept ahead of the FPGA (11.2 ms at 20 kHz).  Refills happen only
 * when a whole block fits below the target, so the steady state is one
 * transaction every FIFO_TARGET / block_max output samples instead of one per
 * service tick: 156 transactions/s at 128 levels, and the FIFO never drops
 * below one block (6.4 ms) before the next refill is due. */
#define UMH_MOD_FIFO_TARGET 224u
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
 * sawtooth.  It is low-passed with tau = 2^SHIFT output samples (102 ms at
 * 20 kHz) before it steers the clock; the unfiltered servo swung the playback
 * rate by several hundred ppm per packet, which is a slow periodic warble.
 * The error is accumulated at the full sample rate but the filter and the
 * Q16 step are only recomputed every UMH_MOD_FILL_GROUP samples, which is the
 * same time constant (alpha * GROUP == 2^-8) at one eighth of the cost. */
#define UMH_MOD_FILL_FILTER_SHIFT 11u
#define UMH_MOD_FILL_GROUP 8u
#define UMH_MOD_FILL_FILTER_STEP (UMH_MOD_FILL_FILTER_SHIFT - 3u)
#define UMH_MOD_FILL_GAIN_PPM 8
/* Host USB scheduling can leave a short gap between packets.  Hold the last
 * envelope sample for up to 25 ms (output samples at 20 kHz) before fading out
 * as a real underrun; ordinary Windows/USB jitter never inserts an audible
 * zero. */
#define UMH_MOD_UNDERRUN_HOLD_TICKS 500u
/* Room a packet needs before the CDC OUT endpoint is re-armed.  Bulk NAK is
 * backpressure the host can see: a browser that burst-sends after a stall
 * waits in its own write() call instead of losing samples inside the ring. */
#define UMH_MOD_RX_HEADROOM 768u

_Static_assert((UMH_MOD_RING_SIZE & (UMH_MOD_RING_SIZE - 1u)) == 0u,
               "envelope ring size must be a power of two");
_Static_assert(UMH_MOD_RX_HEADROOM < UMH_MOD_RING_SIZE,
               "RX headroom must fit inside the envelope ring");
/* Refills are only issued when a whole block fits below the target, so the
 * FIFO fill itself never exceeds it; that is the only bound the RTL needs. */
_Static_assert(UMH_MOD_FIFO_TARGET <= FPGA_MOD_FIFO_DEPTH,
               "level FIFO target must fit in the FPGA level FIFO");
_Static_assert(FPGA_MOD_BLOCK_MAX <= UMH_MOD_FIFO_TARGET,
               "one block must fit below the level FIFO target");
_Static_assert(UMH_MOD_GAIN_ONE % UMH_MOD_GAIN_STEP == 0u,
               "gain ramp must land exactly on 0 and UMH_MOD_GAIN_ONE");
_Static_assert(UMH_MOD_FILL_GROUP == 8u,
               "servo decimation is baked into UMH_MOD_FILL_FILTER_STEP");

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
  uint16_t fill_target;         /* ring occupancy the clock servo regulates to */
  uint8_t envelope_ceiling;
  uint32_t base_step_q16;       /* host rate / FPGA output rate, Q16 */

  /* Per-session state from here to the end is cleared by configure. */
  uint32_t frac_q16;
  uint32_t step_q16;
  int32_t fill_error_q12;       /* low-passed (fill - fill_target), Q12 */
  int32_t clock_correction_ppm;
  uint32_t underrun_count;
  uint32_t overrun_count;
  uint32_t packet_loss_count;
  uint32_t rendered_samples;
  uint32_t max_service_cycles;
  uint32_t expected_sequence;
  uint32_t fill_sum;            /* sum of ring fill over one servo group */
  uint16_t underrun_grace;
  int16_t level_fill;           /* reconstructed FPGA level FIFO fill */
  uint8_t current_sample;
  uint8_t next_sample;
  uint8_t gain;                 /* Q7 output fade, 0..UMH_MOD_GAIN_ONE */
  uint8_t waiting_refill;
  uint8_t sequence_valid;
  uint8_t poll_tick;            /* full status poll every 16 service calls */
  uint8_t servo_tick;           /* decimates the fill servo to 2.5 kHz */
  uint8_t pending_count;        /* levels rendered but not yet accepted */
  uint8_t pending_block[FPGA_MOD_BLOCK_MAX];
  uint32_t last_service_us;     /* drain baseline for the FIFO estimate */
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
/* 1 while another received packet still fits in the envelope ring.  The CDC
 * OUT endpoint is only re-armed when this holds, so a host that runs ahead is
 * NAKed by the USB hardware instead of losing samples at the ring. */
uint8_t aperture_mod_rx_ready(const umh_aperture_mod_t *engine);
/* Call about once per millisecond: tops the FPGA level FIFO up to
 * UMH_MOD_FIFO_TARGET and finishes a requested stop once it has drained. */
void aperture_mod_service(umh_aperture_mod_t *engine);
void aperture_mod_get_status(const umh_aperture_mod_t *engine,
                             umh_mod_status_wire_t *status);

#endif
