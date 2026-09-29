#include "audio_engine.h"
#include "FreeRTOS.h"
#include "task.h"
#include "system_status.h"
#include <stddef.h>
#include <string.h>


static int audio_lock(umh_audio_engine_t *engine)
{
  if (engine == NULL || engine->lock == NULL) return -1;
  return osMutexAcquire(engine->lock, osWaitForever) == osOK ? 0 : -1;
}

static void audio_unlock(umh_audio_engine_t *engine)
{
  if (engine != NULL && engine->lock != NULL) (void)osMutexRelease(engine->lock);
}

static uint16_t ring_count_locked(const umh_audio_engine_t *engine)
{
  return (uint16_t)(engine->ring_head - engine->ring_tail);
}

static uint16_t ring_free_locked(const umh_audio_engine_t *engine)
{
  /* The integer head/tail difference makes the full capacity usable, so the
   * maximum advertised prebuffer (2048) can actually be reached. */
  return (uint16_t)(UMH_AUDIO_RING_SIZE - ring_count_locked(engine));
}

static void ring_reset_locked(umh_audio_engine_t *engine)
{
  engine->ring_head = 0u;
  engine->ring_tail = 0u;
}

static uint8_t ring_pop_locked(umh_audio_engine_t *engine)
{
  return engine->envelope_ring[engine->ring_tail++ & UMH_AUDIO_RING_MASK];
}

static void prime_interpolator(umh_audio_engine_t *engine)
{
  /* Callers guarantee at least one queued sample (prebuffer >= 1).  next is
   * always already popped, so a refill after a hold never skips a sample. */
  engine->current_sample = ring_pop_locked(engine);
  engine->next_sample = ring_count_locked(engine) != 0u ? ring_pop_locked(engine)
                                                        : engine->current_sample;
  engine->frac_q16 = 0u;
}

static uint8_t fetch_interpolated(const umh_audio_engine_t *engine)
{
  /* frac < 2^16 keeps the result between the two samples, so no clamp. */
  int32_t delta = (int32_t)engine->next_sample - (int32_t)engine->current_sample;
  return (uint8_t)((int32_t)engine->current_sample +
                   ((delta * (int32_t)engine->frac_q16) >> 16));
}

static void commit_interpolated(umh_audio_engine_t *engine)
{
  engine->frac_q16 += engine->step_q16;
  while (engine->frac_q16 >= 65536u) {
    engine->frac_q16 -= 65536u;
    engine->current_sample = engine->next_sample;
    if (ring_count_locked(engine) == 0u) {
      /* Hold the last sample (next == current); render_level() turns a
       * lasting gap into a faded underrun. */
      engine->frac_q16 = 0u;
      break;
    }
    engine->next_sample = ring_pop_locked(engine);
  }
}

static void update_clock_correction(umh_audio_engine_t *engine)
{
  /* One-pole low-pass of the fill error (Q12, tau 2^11 output samples)
   * removes the 256-sample packet sawtooth, so the playback rate only
   * follows the real host/device clock difference. */
  int32_t error_q12 = ((int32_t)ring_count_locked(engine) -
                       (int32_t)engine->prebuffer_samples) * 4096;
  int32_t correction;
  engine->fill_error_q12 += (error_q12 - engine->fill_error_q12) >>
                            UMH_AUDIO_FILL_FILTER_SHIFT;
  correction = (engine->fill_error_q12 / 4096) * UMH_AUDIO_FILL_GAIN_PPM;
  if (correction > (int32_t)UMH_AUDIO_MAX_CORRECTION_PPM)
    correction = (int32_t)UMH_AUDIO_MAX_CORRECTION_PPM;
  if (correction < -(int32_t)UMH_AUDIO_MAX_CORRECTION_PPM)
    correction = -(int32_t)UMH_AUDIO_MAX_CORRECTION_PPM;
  engine->clock_correction_ppm = correction;
  /* base <= 2^16 and |ppm| <= 3000 keep the product in 32 bits. */
  engine->step_q16 = (uint32_t)((int32_t)engine->base_step_q16 +
                                (int32_t)engine->base_step_q16 * correction / 1000000);
}

/* One FPGA output sample (lock held).  Every transition - start, underrun,
 * refill and stop - is the same Q7 gain ramp over the held or live level. */
static uint8_t render_level(umh_audio_engine_t *engine)
{
  uint8_t live = 0u;
  uint8_t raw;
  if (engine->state == UMH_AUDIO_RUNNING) {
    if (engine->waiting_refill != 0u) {
      /* Restart at the new data only once fully faded out. */
      if (engine->gain == 0u && ring_count_locked(engine) >= engine->prebuffer_samples) {
        prime_interpolator(engine);
        engine->waiting_refill = 0u;
        live = 1u;
      }
    } else if (ring_count_locked(engine) != 0u) {
      engine->underrun_grace = 0u;
      update_clock_correction(engine);
      live = 1u;
    } else if (engine->underrun_grace == 0u) {
      engine->underrun_grace = UMH_AUDIO_UNDERRUN_HOLD_TICKS;
      live = 1u;
    } else if (--engine->underrun_grace != 0u) {
      live = 1u;
    } else {
      /* A real gap: fade out on the held level, then wait for the host to
       * refill the whole prebuffer before fading back in. */
      engine->underrun_count++;
      engine->waiting_refill = 1u;
    }
  }
  if (live != 0u) {
    if (engine->gain < UMH_AUDIO_GAIN_ONE) engine->gain += UMH_AUDIO_GAIN_STEP;
  } else if (engine->gain != 0u) {
    engine->gain -= UMH_AUDIO_GAIN_STEP;
  }
  raw = fetch_interpolated(engine);
  if (raw > engine->max_level) raw = engine->max_level;
  if (live != 0u) commit_interpolated(engine);
  engine->rendered_samples++;
  return (uint8_t)(((uint32_t)raw * engine->gain + UMH_AUDIO_GAIN_ONE / 2u) /
                   UMH_AUDIO_GAIN_ONE);
}

static void finish_stop_locked(umh_audio_engine_t *engine)
{
  /* Leaving audio mode also flushes the FPGA level FIFO. */
  if (engine->link != NULL) {
    (void)fpga_link_audio_mode(engine->link, 0u, engine->rendered_samples + 1u);
    (void)fpga_link_safe_stop(engine->link);
  }
  ring_reset_locked(engine);
  engine->waiting_refill = 0u;
  engine->gain = 0u;
  engine->state = UMH_AUDIO_OFF;
}

void audio_engine_init(umh_audio_engine_t *engine)
{
  osMutexAttr_t attributes;
  if (engine == NULL) return;
  memset(engine, 0, sizeof(*engine));
  engine->state = UMH_AUDIO_OFF;
  memset(&attributes, 0, sizeof(attributes));
  attributes.name = "umh-audio";
  attributes.cb_mem = &engine->lock_memory;
  attributes.cb_size = sizeof(engine->lock_memory);
  engine->lock = osMutexNew(&attributes);
}

int audio_engine_configure(umh_audio_engine_t *engine,
                           umh_spatial_renderer_t *renderer,
                           fpga_link_t *link,
                           const uint8_t *payload,
                           uint16_t length)
{
  const umh_audio_config_wire_t *config;
  umh_spatial_point_t point;
  umh_output_frame_t frame;
  uint16_t i;
  uint32_t rate;
  uint32_t prebuffer;
  uint8_t extra_count = 0u;

  if (engine == NULL || renderer == NULL || link == NULL || payload == NULL) return -1;
  if (length < sizeof(umh_audio_config_wire_t)) return -2;
  config = (const umh_audio_config_wire_t *)payload;
  if (length > sizeof(umh_audio_config_wire_t)) {
    if (length < sizeof(umh_audio_config_wire_t) + 1u) return -2;
    extra_count = payload[sizeof(umh_audio_config_wire_t)];
    if (extra_count > UMH_AUDIO_MAX_EXTRA_POINTS) return -2;
    if ((uint32_t)length != (uint32_t)sizeof(umh_audio_config_wire_t) + 1u +
        (uint32_t)extra_count * (uint32_t)sizeof(umh_audio_point_wire_t)) return -2;
  }
  rate = config->envelope_rate_hz;
  prebuffer = config->prebuffer_samples;
  if (rate < UMH_AUDIO_MIN_RATE_HZ || rate > UMH_AUDIO_MAX_RATE_HZ) return -2;
  if (prebuffer == 0u || prebuffer > UMH_AUDIO_MAX_PREBUFFER) return -3;
  if (audio_lock(engine) != 0) return -4;
  if (engine->state != UMH_AUDIO_OFF) {
    audio_unlock(engine);
    return -5;
  }

  /* Render the fixed aperture once.  The base point keeps the historical
   * single-focus behaviour (source level 255, envelope scale in config.level);
   * optional cluster points carry their own spatial weight.  All points are
   * complex-summed by the production renderer so calibration and geometry are
   * applied exactly once. */
  memset(&frame, 0, sizeof(frame));
  memset(engine->spatial_real, 0, sizeof(engine->spatial_real));
  memset(engine->spatial_imag, 0, sizeof(engine->spatial_imag));
  memset(&point, 0, sizeof(point));
  point.x_um = config->x_um;
  point.y_um = config->y_um;
  point.z_um = config->z_um;
  point.level = 255u;
  point.phase = config->phase;
  point.source_id = 0u;
  if (spatial_renderer_accumulate_point(renderer, &point, engine->spatial_real,
                                        engine->spatial_imag) != 0) {
    audio_unlock(engine);
    return -6;
  }
  for (i = 0u; i < (uint16_t)extra_count; ++i) {
    const umh_audio_point_wire_t *extra =
        (const umh_audio_point_wire_t *)(payload + sizeof(umh_audio_config_wire_t) + 1u +
                                         (uint32_t)i * sizeof(umh_audio_point_wire_t));
    memset(&point, 0, sizeof(point));
    point.x_um = extra->x_um;
    point.y_um = extra->y_um;
    point.z_um = extra->z_um;
    point.level = extra->level;
    point.phase = extra->phase;
    point.source_id = (uint8_t)(i + 1u);
    if (spatial_renderer_accumulate_point(renderer, &point, engine->spatial_real,
                                          engine->spatial_imag) != 0) {
      audio_unlock(engine);
      return -6;
    }
  }
  if (spatial_renderer_finalize(renderer, engine->spatial_real,
                                engine->spatial_imag, &frame) != 0) {
    audio_unlock(engine);
    return -6;
  }
  for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) {
    engine->aperture_phase[i] = frame.channels[i].phase;
    /* Derive the enable marker from the rendered per-channel amplitude
     * rather than the raw enabled bit: a calibration gain of zero also mutes
     * its channel, and that must survive the focused-AM common-level path. */
    engine->aperture_enable[i] = frame.channels[i].level != 0u ? 1u : 0u;
  }

  if (fpga_link_audio_begin(link, engine->aperture_phase,
                            engine->aperture_enable, 0u) != 0) {
    audio_unlock(engine);
    return -7;
  }

  ring_reset_locked(engine);
  engine->renderer = renderer;
  engine->link = link;
  engine->prebuffer_samples = (uint16_t)prebuffer;
  engine->max_level = (uint8_t)(((uint32_t)config->level * UMH_AUDIO_MAX_LEVEL + 127u) / 255u);
  engine->base_step_q16 = (rate << 16) / FPGA_AUDIO_OUTPUT_RATE_HZ;
  /* Clear the whole per-session tail (counters, interpolator, servo, gain). */
  memset((uint8_t *)engine + offsetof(umh_audio_engine_t, frac_q16), 0,
         sizeof(*engine) - offsetof(umh_audio_engine_t, frac_q16));
  engine->step_q16 = engine->base_step_q16;
  engine->state = UMH_AUDIO_CONFIGURED;
  audio_unlock(engine);
  return 0;
}

int audio_engine_start(umh_audio_engine_t *engine)
{
  int result = -2;
  if (engine == NULL || audio_lock(engine) != 0) return -1;
  /* configure() already cleared the per-session state. */
  if (engine->state == UMH_AUDIO_CONFIGURED) {
    engine->state = UMH_AUDIO_PRIMING;
    result = 0;
  }
  audio_unlock(engine);
  return result;
}

int audio_engine_feed(umh_audio_engine_t *engine, const uint8_t *levels,
                      uint16_t length, uint32_t stream_sequence)
{
  uint16_t accepted;
  uint16_t i;
  if (engine == NULL || levels == NULL || length == 0u) return -1;
  if (audio_lock(engine) != 0) return -2;
  if (engine->state == UMH_AUDIO_OFF || engine->state == UMH_AUDIO_STOPPING ||
      engine->state == UMH_AUDIO_FAULT) {
    audio_unlock(engine);
    return -3;
  }
  accepted = ring_free_locked(engine);
  if (accepted > length) accepted = length;
  for (i = 0u; i < accepted; ++i) {
    engine->envelope_ring[engine->ring_head & UMH_AUDIO_RING_MASK] = levels[i];
    engine->ring_head++;
  }
  if (accepted < length) engine->overrun_count += (uint32_t)(length - accepted);

  if (engine->sequence_valid == 0u) {
    engine->expected_sequence = stream_sequence + 1u;
    engine->sequence_valid = 1u;
  } else if (stream_sequence != engine->expected_sequence) {
    if (stream_sequence > engine->expected_sequence)
      engine->packet_loss_count += stream_sequence - engine->expected_sequence;
    else
      engine->packet_loss_count++;
    engine->expected_sequence = stream_sequence + 1u;
  } else {
    engine->expected_sequence++;
  }
  audio_unlock(engine);
  return accepted == length ? 0 : -4;
}

void audio_engine_request_stop(umh_audio_engine_t *engine)
{
  if (engine == NULL || audio_lock(engine) != 0) return;
  /* service() fades the gain out, lets the FIFO drain and then leaves audio
   * mode; states that never played are already at gain 0. */
  if (engine->state != UMH_AUDIO_OFF) engine->state = UMH_AUDIO_STOPPING;
  audio_unlock(engine);
}

void audio_engine_abort(umh_audio_engine_t *engine)
{
  if (engine == NULL || audio_lock(engine) != 0) return;
  if (engine->state != UMH_AUDIO_OFF) finish_stop_locked(engine);
  audio_unlock(engine);
}

uint8_t audio_engine_owns_output(const umh_audio_engine_t *engine)
{
  if (engine == NULL) return 0u;
  return engine->state != UMH_AUDIO_OFF ? 1u : 0u;
}

void audio_engine_service(umh_audio_engine_t *engine)
{
  uint8_t block[FPGA_AUDIO_BLOCK_MAX];
  uint32_t start_cycles;
  uint32_t elapsed;
  int fill;
  if (engine == NULL || audio_lock(engine) != 0) return;
  if (engine->state == UMH_AUDIO_OFF || engine->link == NULL) {
    audio_unlock(engine);
    return;
  }
  start_cycles = DWT->CYCCNT;
  if (engine->state == UMH_AUDIO_PRIMING &&
      ring_count_locked(engine) >= engine->prebuffer_samples) {
    prime_interpolator(engine);
    engine->state = UMH_AUDIO_RUNNING;
  }
  /* Every transaction's CS edge feeds the FPGA link watchdog; the full
   * status (PLL lock, running flag, credit) is refreshed every 16 calls. */
  /* Full status every 16 ms.  Should the FPGA link watchdog ever drop
   * AUDIO_MODE, its FIFO stops draining: end the stream and report it
   * instead of freezing silently behind a full ring. */
  if ((++engine->poll_tick & 15u) == 0u && fpga_link_poll_status(engine->link) == 0 &&
      (engine->link->status.status_flags & FPGA_STATUS_AUDIO_MODE) == 0u &&
      (engine->state == UMH_AUDIO_RUNNING || engine->state == UMH_AUDIO_STOPPING)) {
    finish_stop_locked(engine);
    system_status_fault(UMH_FAULT_FPGA_OUTPUT, 22u, UMH_FAULT_WARNING);
  }
  fill = fpga_link_audio_block(engine->link, NULL, 0u);
  /* Top the FIFO up while playing or while a stop fade is still audible.
   * Each reply reports the real fill before that block. */
  while (fill >= 0 && fill < (int)UMH_AUDIO_FIFO_TARGET &&
         (engine->state == UMH_AUDIO_RUNNING ||
          (engine->state == UMH_AUDIO_STOPPING && engine->gain != 0u))) {
    uint8_t count = (uint8_t)((int)UMH_AUDIO_FIFO_TARGET - fill);
    uint8_t i;
    if (count > FPGA_AUDIO_BLOCK_MAX) count = FPGA_AUDIO_BLOCK_MAX;
    for (i = 0u; i < count; ++i) block[i] = render_level(engine);
    fill = fpga_link_audio_block(engine->link, block, count);
    if (fill >= 0) fill += count;
  }
  /* A stop completes once faded out and drained; a dead link cannot drain,
   * so it ends the stop at once instead of hanging in STOPPING. */
  if (engine->state == UMH_AUDIO_STOPPING &&
      (fill < 0 || (engine->gain == 0u && fill == 0)))
    finish_stop_locked(engine);
  elapsed = DWT->CYCCNT - start_cycles;
  if (elapsed > engine->max_service_cycles) engine->max_service_cycles = elapsed;
  audio_unlock(engine);
}

void audio_engine_get_status(const umh_audio_engine_t *engine,
                             umh_audio_status_wire_t *status)
{
  if (status == NULL) return;
  memset(status, 0, sizeof(*status));
  if (engine == NULL) return;
  if (audio_lock((umh_audio_engine_t *)engine) != 0) return;
  status->state = (uint8_t)engine->state;
  if (engine->state == UMH_AUDIO_CONFIGURED) status->flags |= UMH_AUDIO_STATUS_CONFIGURED;
  if (engine->state == UMH_AUDIO_PRIMING) status->flags |= UMH_AUDIO_STATUS_PRIMING;
  if (engine->state == UMH_AUDIO_RUNNING) status->flags |= UMH_AUDIO_STATUS_RUNNING;
  if (engine->waiting_refill != 0u) status->flags |= UMH_AUDIO_STATUS_REFILLING;
  if (engine->underrun_count != 0u && engine->state != UMH_AUDIO_OFF)
    status->flags |= UMH_AUDIO_STATUS_UNDERRUN;
  status->ring_fill = ring_count_locked(engine);
  status->ring_capacity = UMH_AUDIO_RING_SIZE;
  status->prebuffer = engine->prebuffer_samples;
  status->underrun_count = engine->underrun_count;
  status->overrun_count = engine->overrun_count;
  status->packet_loss_count = engine->packet_loss_count;
  status->rendered_samples = engine->rendered_samples;
  status->clock_correction_ppm = engine->clock_correction_ppm;
  if (SystemCoreClock != 0u)
    status->max_service_us = (uint32_t)(((uint64_t)engine->max_service_cycles * 1000000u) /
                                        SystemCoreClock);
  else
    status->max_service_us = 0u;
  audio_unlock((umh_audio_engine_t *)engine);
}
