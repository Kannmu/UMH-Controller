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
 * FPGA through the ordinary FRAME path, and then streams one common 8-bit
 * envelope level per audio sample.  The FPGA rebuilds its inactive event
 * table with the stored phases and the new common level, so the acoustic
 * aperture stays fixed while the carrier amplitude follows the audio. */
#define UMH_AUDIO_RING_SIZE 2048u
#define UMH_AUDIO_RING_MASK (UMH_AUDIO_RING_SIZE - 1u)
#define UMH_AUDIO_MIN_RATE_HZ 8000u
#define UMH_AUDIO_MAX_RATE_HZ 20000u
#define UMH_AUDIO_MAX_PREBUFFER UMH_AUDIO_RING_SIZE
#define UMH_AUDIO_LEAD_US 30u
#define UMH_AUDIO_SLEW_STEP 4u
#define UMH_AUDIO_LINK_POLL_US 5000u
#define UMH_AUDIO_STOP_FADE_MS 5u
#define UMH_AUDIO_MAX_LEVEL 128u
/* Fill-error clock recovery range.  The PC audio clock and the 64 MHz FPGA
 * clock are independent; allow a few thousand ppm so a slightly slow host
 * cannot gradually empty the ring.  The correction is only applied while the
 * fill deviates from the prebuffer target. */
#define UMH_AUDIO_MAX_CORRECTION_PPM 3000
/* Host USB scheduling can leave a short gap between packets.  Hold the last
 * envelope for up to 25 ms before declaring a real underrun; this covers
 * ordinary Windows/USB jitter without inserting an audible zero. */
#define UMH_AUDIO_UNDERRUN_HOLD_TICKS 500u

_Static_assert((UMH_AUDIO_RING_SIZE & (UMH_AUDIO_RING_SIZE - 1u)) == 0u,
               "audio ring size must be a power of two");

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

  uint8_t envelope_ring[UMH_AUDIO_RING_SIZE];
  volatile uint16_t ring_head;
  volatile uint16_t ring_tail;

  uint8_t aperture_phase[UMH_DEVICE_CHANNEL_COUNT];
  uint8_t aperture_enable[UMH_DEVICE_CHANNEL_COUNT];
  float spatial_real[UMH_DEVICE_CHANNEL_COUNT];
  float spatial_imag[UMH_DEVICE_CHANNEL_COUNT];
  uint32_t envelope_rate_hz;
  uint32_t period_q16_us;
  uint16_t prebuffer_samples;
  uint8_t max_level;
  uint8_t flags;

  uint64_t next_due_q16_us;
  uint64_t last_link_us;
  uint32_t frac_q16;
  uint32_t step_q16;
  uint8_t current_sample;
  uint8_t next_sample;
  uint8_t primed;
  uint8_t waiting_refill;
  uint8_t last_submitted_level;
  uint8_t fade_level;
  uint8_t fade_step;
  uint16_t underrun_grace;
  int32_t clock_correction_ppm;

  uint32_t underrun_count;
  uint32_t overrun_count;
  uint32_t packet_loss_count;
  uint32_t rendered_samples;
  uint32_t max_service_cycles;
  uint8_t sequence_valid;
  uint32_t expected_sequence;
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
uint8_t audio_engine_is_active(const umh_audio_engine_t *engine);
uint8_t audio_engine_owns_output(const umh_audio_engine_t *engine);
uint8_t audio_engine_next_deadline(const umh_audio_engine_t *engine,
                                   uint64_t *deadline_us);
void audio_engine_service(umh_audio_engine_t *engine, uint64_t now_us);
void audio_engine_get_status(const umh_audio_engine_t *engine,
                             umh_audio_status_wire_t *status);

#endif
