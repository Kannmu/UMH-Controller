#include "audio_engine.h"
#include "FreeRTOS.h"
#include "task.h"
#include "system_status.h"
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

static uint8_t ring_peek_locked(const umh_audio_engine_t *engine)
{
  return engine->envelope_ring[engine->ring_tail & UMH_AUDIO_RING_MASK];
}

static int prime_interpolator(umh_audio_engine_t *engine)
{
  if (ring_count_locked(engine) == 0u) return -1;
  engine->current_sample = ring_peek_locked(engine);
  engine->ring_tail++;
  engine->next_sample = ring_count_locked(engine) > 0u ? ring_peek_locked(engine)
                                                      : engine->current_sample;
  engine->frac_q16 = 0u;
  engine->primed = 1u;
  return 0;
}

static int fetch_interpolated(const umh_audio_engine_t *engine, uint8_t *level)
{
  int32_t delta;
  int32_t value;
  if (engine == NULL || level == NULL || engine->primed == 0u) return -1;
  delta = (int32_t)engine->next_sample - (int32_t)engine->current_sample;
  value = (int32_t)engine->current_sample +
          ((delta * (int32_t)engine->frac_q16) >> 16);
  if (value < 0) value = 0;
  if (value > 255) value = 255;
  *level = (uint8_t)value;
  return 0;
}

static void commit_interpolated(umh_audio_engine_t *engine)
{
  engine->frac_q16 += engine->step_q16;
  while (engine->frac_q16 >= 65536u) {
    engine->frac_q16 -= 65536u;
    if (ring_count_locked(engine) == 0u) {
      /* Hold the last differential pair; the next service call turns this
       * into an explicit underrun instead of randomly wrapping the level. */
      engine->current_sample = engine->next_sample;
      engine->frac_q16 = 0u;
      break;
    }
    engine->ring_tail++;
    engine->current_sample = engine->next_sample;
    engine->next_sample = ring_count_locked(engine) > 0u ? ring_peek_locked(engine)
                                                         : engine->current_sample;
  }
}

static void update_clock_correction(umh_audio_engine_t *engine)
{
  int32_t error = (int32_t)ring_count_locked(engine) - (int32_t)engine->prebuffer_samples;
  int32_t correction = error * 2;
  if (correction > (int32_t)UMH_AUDIO_MAX_CORRECTION_PPM)
    correction = (int32_t)UMH_AUDIO_MAX_CORRECTION_PPM;
  if (correction < -(int32_t)UMH_AUDIO_MAX_CORRECTION_PPM)
    correction = -(int32_t)UMH_AUDIO_MAX_CORRECTION_PPM;
  engine->clock_correction_ppm = correction;
  engine->step_q16 = (uint32_t)(65536 + ((int64_t)correction * 65536) / 1000000);
}

static int submit_level(umh_audio_engine_t *engine, uint8_t level, uint64_t now_us)
{
  int result;
  if (engine->link == NULL) return -1;
  if (engine->link->audio_short_supported != 0u)
    result = fpga_link_audio_level_fast(engine->link, level);
  else
    result = fpga_link_audio_level(engine->link, level, engine->rendered_samples);
  if (result != 0) return -1;
  engine->last_submitted_level = level;
  engine->last_link_us = now_us;
  return 0;
}

static int poll_link_if_due(umh_audio_engine_t *engine, uint64_t now_us)
{
  if (engine->link == NULL) return -1;
  if (now_us - engine->last_link_us < UMH_AUDIO_LINK_POLL_US) return 0;
  (void)fpga_link_poll_status(engine->link);
  engine->last_link_us = now_us;
  return 0;
}

static void enter_underrun(umh_audio_engine_t *engine, uint64_t now_us)
{
  engine->underrun_count++;
  engine->waiting_refill = 1u;
  engine->primed = 0u;
  engine->frac_q16 = 0u;
  if (engine->last_submitted_level != 0u) {
    (void)submit_level(engine, 0u, now_us);
  }
  engine->fade_level = 0u;
  engine->next_due_q16_us = ((now_us + engine->period_q16_us / 65536u + 1u) << 16);
}

static void service_running(umh_audio_engine_t *engine, uint64_t now_us)
{
  uint64_t now_q16 = now_us << 16;
  uint8_t raw_level;
  uint8_t target;
  uint32_t period_us;

  if (engine->waiting_refill != 0u) {
    /* A failed zero-level transaction must not leave the last carrier level
     * running while the host refills the ring. */
    if (engine->last_submitted_level != 0u) (void)submit_level(engine, 0u, now_us);
    if (ring_count_locked(engine) >= engine->prebuffer_samples) {
      if (prime_interpolator(engine) == 0) {
        engine->waiting_refill = 0u;
        engine->next_due_q16_us = ((now_us + UMH_AUDIO_LEAD_US) << 16);
      }
    }
    return;
  }

  if (now_q16 + ((uint64_t)1000u << 16) < engine->next_due_q16_us) return;

  period_us = engine->period_q16_us >> 16;
  if (period_us == 0u) period_us = 1u;
  /* Only discard a backlog after a genuinely catastrophic stall (debugger
   * halt, USB recovery, etc.).  Normal scheduling jitter is absorbed by
   * processing the overdue ticks back-to-back, which keeps the average
   * envelope rate locked to the 20 kHz crystal.  The old 4-period limit
   * dropped several percent of the ticks under load and made the ring fill
   * up even when the host stream was nominally correct. */
  if (now_us > (engine->next_due_q16_us >> 16) + 10000u) {
    engine->next_due_q16_us = ((now_us + UMH_AUDIO_LEAD_US) << 16);
  }

  if (ring_count_locked(engine) == 0u || fetch_interpolated(engine, &raw_level) != 0) {
    if (engine->underrun_grace == 0u) {
      engine->underrun_grace = UMH_AUDIO_UNDERRUN_HOLD_TICKS;
    } else if (engine->underrun_grace != 0u) {
      engine->underrun_grace--;
    }
    if (engine->underrun_grace != 0u) {
      /* Hold the last commanded envelope and keep the 20 kHz output clock
       * running.  A short host/USB gap therefore does not blank the carrier
       * or restart the prebuffer. */
      ++engine->rendered_samples;
      engine->next_due_q16_us += engine->period_q16_us;
      return;
    }
    enter_underrun(engine, now_us);
    return;
  }
  engine->underrun_grace = 0u;

  update_clock_correction(engine);
  target = raw_level > engine->max_level ? engine->max_level : raw_level;
  if (target > engine->fade_level) {
    uint8_t next = (uint8_t)(engine->fade_level + UMH_AUDIO_SLEW_STEP);
    engine->fade_level = next > target ? target : next;
  } else if (target < engine->fade_level) {
    engine->fade_level = engine->fade_level > UMH_AUDIO_SLEW_STEP
                       ? (uint8_t)(engine->fade_level - UMH_AUDIO_SLEW_STEP) : 0u;
    if (engine->fade_level < target) engine->fade_level = target;
  }

  if (engine->fade_level != engine->last_submitted_level) {
    if (submit_level(engine, engine->fade_level, now_us) != 0) {
      engine->next_due_q16_us = ((now_us + 200u) << 16);
      return;
    }
  } else {
    (void)poll_link_if_due(engine, now_us);
  }

  commit_interpolated(engine);
  ++engine->rendered_samples;
  engine->next_due_q16_us += engine->period_q16_us;
}

static void service_stopping(umh_audio_engine_t *engine, uint64_t now_us)
{
  if (engine->next_due_q16_us == 0u) {
    engine->next_due_q16_us = (now_us << 16);
  }
  if ((now_us << 16) < engine->next_due_q16_us) return;

  if (engine->fade_level != 0u) {
    if (engine->fade_step == 0u) engine->fade_step = 1u;
    engine->fade_level = engine->fade_level > engine->fade_step
                       ? (uint8_t)(engine->fade_level - engine->fade_step) : 0u;
    if (submit_level(engine, engine->fade_level, now_us) != 0) {
      engine->next_due_q16_us = ((now_us + 200u) << 16);
      return;
    }
    engine->next_due_q16_us += engine->period_q16_us;
    if (engine->fade_level != 0u) return;
  }

  (void)submit_level(engine, 0u, now_us);
  if (engine->link != NULL) {
    (void)fpga_link_audio_mode(engine->link, 0u, engine->rendered_samples + 1u);
    (void)fpga_link_safe_stop(engine->link);
  }
  ring_reset_locked(engine);
  engine->primed = 0u;
  engine->waiting_refill = 0u;
  engine->last_submitted_level = 0u;
  engine->fade_level = 0u;
  engine->next_due_q16_us = 0u;
  engine->state = UMH_AUDIO_OFF;
}

void audio_engine_init(umh_audio_engine_t *engine)
{
  osMutexAttr_t attributes;
  if (engine == NULL) return;
  memset(engine, 0, sizeof(*engine));
  engine->state = UMH_AUDIO_OFF;
  engine->step_q16 = 65536u;
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
  engine->envelope_rate_hz = rate;
  engine->period_q16_us = (uint32_t)(((uint64_t)1000000u << 16) / rate);
  engine->prebuffer_samples = (uint16_t)prebuffer;
  engine->max_level = (uint8_t)(((uint32_t)config->level * UMH_AUDIO_MAX_LEVEL + 127u) / 255u);
  if (engine->max_level > UMH_AUDIO_MAX_LEVEL) engine->max_level = (uint8_t)UMH_AUDIO_MAX_LEVEL;
  engine->flags = (uint8_t)config->flags;
  engine->next_due_q16_us = 0u;
  engine->last_link_us = 0u;
  engine->frac_q16 = 0u;
  engine->step_q16 = 65536u;
  engine->current_sample = 0u;
  engine->next_sample = 0u;
  engine->primed = 0u;
  engine->waiting_refill = 0u;
  engine->last_submitted_level = 0u;
  engine->fade_level = 0u;
  engine->fade_step = 0u;
  engine->underrun_grace = 0u;
  engine->clock_correction_ppm = 0;
  engine->underrun_count = 0u;
  engine->overrun_count = 0u;
  engine->packet_loss_count = 0u;
  engine->rendered_samples = 0u;
  engine->max_service_cycles = 0u;
  engine->sequence_valid = 0u;
  engine->expected_sequence = 0u;
  engine->state = UMH_AUDIO_CONFIGURED;
  audio_unlock(engine);
  return 0;
}

int audio_engine_start(umh_audio_engine_t *engine)
{
  if (engine == NULL || audio_lock(engine) != 0) return -1;
  if (engine->state != UMH_AUDIO_CONFIGURED) {
    audio_unlock(engine);
    return -2;
  }
  engine->state = UMH_AUDIO_PRIMING;
  engine->next_due_q16_us = 0u;
  engine->waiting_refill = 0u;
  engine->primed = 0u;
  engine->frac_q16 = 0u;
  engine->last_submitted_level = 0u;
  engine->fade_level = 0u;
  engine->fade_step = 0u;
  engine->underrun_grace = 0u;
  audio_unlock(engine);
  return 0;
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
  uint32_t fade_ticks;
  if (engine == NULL) return;
  if (audio_lock(engine) != 0) return;
  if (engine->state == UMH_AUDIO_OFF) {
    audio_unlock(engine);
    return;
  }
  if (engine->state == UMH_AUDIO_FAULT) {
    engine->state = UMH_AUDIO_STOPPING;
    engine->fade_level = 0u;
  } else if (engine->state != UMH_AUDIO_STOPPING) {
    fade_ticks = (engine->envelope_rate_hz * UMH_AUDIO_STOP_FADE_MS) / 1000u;
    if (fade_ticks == 0u) fade_ticks = 1u;
    engine->fade_step = (uint8_t)((engine->fade_level + fade_ticks - 1u) / fade_ticks);
    if (engine->fade_step == 0u) engine->fade_step = 1u;
    engine->state = UMH_AUDIO_STOPPING;
  }
  engine->next_due_q16_us = 0u;
  audio_unlock(engine);
}

void audio_engine_abort(umh_audio_engine_t *engine)
{
  if (engine == NULL) return;
  if (audio_lock(engine) != 0) return;
  if (engine->state != UMH_AUDIO_OFF) {
    if (engine->link != NULL) {
      (void)fpga_link_audio_level(engine->link, 0u, engine->rendered_samples);
      (void)fpga_link_audio_mode(engine->link, 0u, engine->rendered_samples + 1u);
      (void)fpga_link_safe_stop(engine->link);
    }
    ring_reset_locked(engine);
    engine->primed = 0u;
    engine->waiting_refill = 0u;
    engine->last_submitted_level = 0u;
    engine->fade_level = 0u;
    engine->next_due_q16_us = 0u;
    engine->state = UMH_AUDIO_OFF;
  }
  audio_unlock(engine);
}

uint8_t audio_engine_is_active(const umh_audio_engine_t *engine)
{
  umh_audio_state_t state;
  if (engine == NULL) return 0u;
  state = engine->state;
  return (state == UMH_AUDIO_PRIMING || state == UMH_AUDIO_RUNNING ||
          state == UMH_AUDIO_STOPPING) ? 1u : 0u;
}

uint8_t audio_engine_owns_output(const umh_audio_engine_t *engine)
{
  if (engine == NULL) return 0u;
  return engine->state != UMH_AUDIO_OFF ? 1u : 0u;
}

uint8_t audio_engine_next_deadline(const umh_audio_engine_t *engine,
                                   uint64_t *deadline_us)
{
  uint8_t result = 0u;
  if (engine == NULL || deadline_us == NULL) return 0u;
  if (audio_lock((umh_audio_engine_t *)engine) != 0) return 0u;
  if (engine->state == UMH_AUDIO_RUNNING && engine->waiting_refill == 0u) {
    *deadline_us = engine->next_due_q16_us >> 16;
    result = 1u;
  } else if (engine->state == UMH_AUDIO_STOPPING && engine->next_due_q16_us != 0u) {
    *deadline_us = engine->next_due_q16_us >> 16;
    result = 1u;
  }
  audio_unlock((umh_audio_engine_t *)engine);
  return result;
}

void audio_engine_service(umh_audio_engine_t *engine, uint64_t now_us)
{
  uint32_t start_cycles;
  if (engine == NULL) return;
  if (audio_lock(engine) != 0) return;
  start_cycles = DWT->CYCCNT;
  switch (engine->state) {
    case UMH_AUDIO_PRIMING:
      if (ring_count_locked(engine) >= engine->prebuffer_samples &&
          prime_interpolator(engine) == 0) {
        engine->state = UMH_AUDIO_RUNNING;
        engine->waiting_refill = 0u;
        engine->next_due_q16_us = ((now_us + UMH_AUDIO_LEAD_US) << 16);
      }
      break;
    case UMH_AUDIO_RUNNING:
      service_running(engine, now_us);
      break;
    case UMH_AUDIO_STOPPING:
      service_stopping(engine, now_us);
      break;
    default:
      break;
  }
  {
    uint32_t elapsed_cycles = DWT->CYCCNT - start_cycles;
    if (elapsed_cycles > engine->max_service_cycles)
      engine->max_service_cycles = elapsed_cycles;
  }
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
