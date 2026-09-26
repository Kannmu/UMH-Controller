#include "hologram_engine.h"
#include "main.h"
#include <math.h>
#include <string.h>

/* Level gamma table.
 *
 * `level` on this hardware is the number of high slots out of 256 in one 40 kHz
 * carrier period, so the launched fundamental is proportional to
 * sin(pi * level / 256): level 128 is a 50 % square wave and is the maximum
 * amplitude, while level 255 would be 99.6 % DC with almost no 40 kHz content.
 * The host uploads a linear amplitude per channel; this table turns it back
 * into the level byte that produces it.
 *
 * Entry 0 is special: it is the "channel off" value, not a zero-amplitude one. */
#define UMH_HOLOGRAM_GAMMA_MAX 128u
#define UMH_HOLOGRAM_LEVEL_FULL 128u

static uint8_t hologram_level_lut[UMH_HOLOGRAM_GAMMA_MAX + 1u];
static uint8_t hologram_level_lut_ready;

static void hologram_build_level_lut(void)
{
  uint32_t i;
  float peak = sinf(3.14159265358979f * (float)UMH_HOLOGRAM_GAMMA_MAX / 256.0f);
  if (peak <= 0.0f) peak = 1.0f;
  hologram_level_lut[0] = 0u;
  for (i = 1u; i <= UMH_HOLOGRAM_GAMMA_MAX; ++i) {
    float value = sinf(3.14159265358979f * (float)i / 256.0f) / peak;
    float scaled = value * (float)UMH_HOLOGRAM_LEVEL_FULL + 0.5f;
    if (scaled < 1.0f) scaled = 1.0f;
    if (scaled > (float)UMH_HOLOGRAM_LEVEL_FULL) scaled = (float)UMH_HOLOGRAM_LEVEL_FULL;
    hologram_level_lut[i] = (uint8_t)scaled;
  }
  hologram_level_lut_ready = 1u;
}

static uint8_t hologram_level_from_gain(uint32_t gain)
{
  uint32_t lo;
  uint32_t hi;
  if (hologram_level_lut_ready == 0u) hologram_build_level_lut();
  if (gain == 0u) return 0u;
  if (gain >= UMH_HOLOGRAM_GAMMA_MAX) return hologram_level_lut[UMH_HOLOGRAM_GAMMA_MAX];
  /* The table is monotonic, so a plain binary search is exact. */
  lo = 0u;
  hi = UMH_HOLOGRAM_GAMMA_MAX;
  while (hi - lo > 1u) {
    uint32_t mid = (lo + hi) / 2u;
    if ((uint32_t)hologram_level_lut[mid] <= gain) lo = mid;
    else hi = mid;
  }
  if (gain - (uint32_t)hologram_level_lut[lo] >
      (uint32_t)hologram_level_lut[hi] - gain) return hologram_level_lut[hi];
  return hologram_level_lut[lo];
}

static int hologram_lock(umh_hologram_engine_t *engine)
{
  if (engine == NULL || engine->lock == NULL) return -1;
  return osMutexAcquire(engine->lock, osWaitForever) == osOK ? 0 : -1;
}

static void hologram_unlock(umh_hologram_engine_t *engine)
{
  if (engine != NULL && engine->lock != NULL) (void)osMutexRelease(engine->lock);
}

static void hologram_record_service(umh_hologram_engine_t *engine, uint32_t start_cycles)
{
  uint32_t cycles;
  if (engine == NULL) return;
  cycles = DWT->CYCCNT - start_cycles;
  if (cycles > engine->service_max_cycles) engine->service_max_cycles = cycles;
  engine->service_sum_cycles += cycles;
  if (engine->service_count != 0xFFFFFFFFu) engine->service_count++;
}

static void hologram_record_render(umh_hologram_engine_t *engine, uint32_t cycles)
{
  if (cycles > engine->render_max_cycles) engine->render_max_cycles = cycles;
  engine->render_sum_cycles += cycles;
}

static void hologram_record_submit(umh_hologram_engine_t *engine, uint32_t cycles)
{
  if (cycles > engine->submit_max_cycles) engine->submit_max_cycles = cycles;
  engine->submit_sum_cycles += cycles;
}

static void hologram_clear_channel_state(umh_hologram_engine_t *engine)
{
  memset(engine->last_phase, 0, sizeof(engine->last_phase));
  memset(engine->last_level, 0, sizeof(engine->last_level));
  engine->have_last = 0u;
}

static uint64_t hologram_period_us(const umh_hologram_engine_t *engine)
{
  uint32_t rate = engine->output_rate_hz;
  if (rate < UMH_HOLOGRAM_MIN_RATE_HZ) rate = UMH_HOLOGRAM_MIN_RATE_HZ;
  if (rate > UMH_HOLOGRAM_MAX_RATE_HZ) rate = UMH_HOLOGRAM_MAX_RATE_HZ;
  return 1000000ull / (uint64_t)rate;
}

/* Length of one cycle: every segment costs transition_ms, and loop_ms only
 * exists so the host can stretch a cycle without changing the segment time. */
static uint64_t hologram_cycle_us(const umh_hologram_engine_t *engine)
{
  uint64_t segments;
  uint64_t cycle;
  if (engine->keyframes <= 1u) return 0u;
  if (engine->loop_ms != 0u) return (uint64_t)engine->loop_ms * 1000ull;
  segments = (engine->flags & UMH_HOLOGRAM_FLAG_HOLD_LAST) != 0u ?
             (uint64_t)(engine->keyframes - 1u) : (uint64_t)engine->keyframes;
  cycle = segments * (uint64_t)engine->transition_ms * 1000ull;
  return cycle != 0u ? cycle : 1000ull;
}

static uint16_t hologram_keyframe_index(const umh_hologram_engine_t *engine,
                                        uint64_t elapsed_us, uint32_t *alpha_x1000)
{
  uint64_t cycle;
  uint64_t offset;
  uint64_t segment_us;
  uint64_t index;
  uint64_t fraction;
  cycle = hologram_cycle_us(engine);
  if (alpha_x1000 != NULL) *alpha_x1000 = 0u;
  if (engine->keyframes <= 1u || cycle == 0u) return 0u;
  offset = elapsed_us % cycle;
  segment_us = (uint64_t)engine->transition_ms * 1000ull;
  if (segment_us == 0u) {
    index = (offset * (uint64_t)engine->keyframes) / cycle;
    if (index >= (uint64_t)engine->keyframes) index = (uint64_t)engine->keyframes - 1u;
    return (uint16_t)index;
  }
  index = offset / segment_us;
  if (index >= (uint64_t)engine->keyframes) index = (uint64_t)engine->keyframes - 1u;
  if (index + 1u >= (uint64_t)engine->keyframes) {
    /* Last keyframe: hold it, or wrap back to the first one. */
    if ((engine->flags & UMH_HOLOGRAM_FLAG_HOLD_LAST) != 0u) {
      if (alpha_x1000 != NULL) *alpha_x1000 = 0u;
      return (uint16_t)index;
    }
    if (engine->loop_ms != 0u) {
      /* An explicit loop time may be longer than the sum of the segments: the
       * remainder is a dwell on the last keyframe. */
      fraction = offset - index * segment_us;
      if (alpha_x1000 != NULL)
        *alpha_x1000 = (uint32_t)((fraction * 1000ull) / segment_us);
      return (uint16_t)index;
    }
  }
  fraction = offset - index * segment_us;
  if (alpha_x1000 != NULL) {
    uint32_t alpha = (uint32_t)((fraction * 1000ull) / segment_us);
    if (alpha > 1000u) alpha = 1000u;
    *alpha_x1000 = alpha;
  }
  return (uint16_t)index;
}

/* Fade envelope: 20 ms in, 20 ms out, applied to the level byte. */
static uint32_t hologram_fade_gain_x1000(const umh_hologram_engine_t *engine)
{
  float scale = engine->fade_scale;
  if (scale >= 1.0f) return 1000u;
  if (scale <= 0.0f) return 0u;
  return (uint32_t)(scale * 1000.0f + 0.5f);
}

static void hologram_compose(umh_hologram_engine_t *engine, uint16_t index,
                             uint32_t alpha_x1000, uint32_t fade_x1000,
                             uint8_t *phase_out, uint8_t *level_out)
{
  const uint8_t *pa = engine->phase[index];
  const uint8_t *la = engine->level_table[index];
  const uint8_t *pb;
  const uint8_t *lb;
  uint32_t scale = (uint32_t)engine->level;
  uint32_t i;
  if (index + 1u < engine->keyframes && alpha_x1000 > 0u) {
    pb = engine->phase[index + 1u];
    lb = engine->level_table[index + 1u];
  } else {
    pb = pa;
    lb = la;
    alpha_x1000 = 0u;
  }
  for (i = 0u; i < UMH_HOLOGRAM_CHANNELS; ++i) {
    /* Shortest 8-bit wrap path, exactly like the block parser's DELTA rule. */
    int32_t delta = (int32_t)((uint8_t)(pb[i] - pa[i]));
    if (delta > 127) delta -= 256;
    else if (delta < -128) delta += 256;
    phase_out[i] = (uint8_t)((int32_t)pa[i] +
                             (int32_t)(((int64_t)delta * (int64_t)alpha_x1000) / 1000));
    {
      uint32_t value = ((uint32_t)la[i] * (1000u - alpha_x1000) +
                        (uint32_t)lb[i] * alpha_x1000) / 1000u;
      if ((engine->flags & UMH_HOLOGRAM_FLAG_GAMMA) != 0u)
        value = hologram_level_from_gain(value);
      value = (value * scale) / 128u;
      value = (value * fade_x1000) / 1000u;
      if (value > 255u) value = 255u;
      level_out[i] = (uint8_t)value;
    }
  }
}

static uint8_t hologram_needs_submit(const umh_hologram_engine_t *engine,
                                     const uint8_t *phase, const uint8_t *level,
                                     uint64_t now_us)
{
  uint32_t i;
  uint32_t threshold = engine->keyframes > 1u ? 1u : 0u;
  if (engine->have_last == 0u) return 1u;
  if (now_us - engine->last_submit_us >=
      (uint64_t)UMH_HOLOGRAM_REFRESH_MS * 1000ull) return 1u;
  for (i = 0u; i < UMH_HOLOGRAM_CHANNELS; ++i) {
    int32_t dphase = (int32_t)(uint8_t)(phase[i] - engine->last_phase[i]);
    int32_t dlevel = (int32_t)level[i] - (int32_t)engine->last_level[i];
    if (dphase > 127) dphase -= 256;
    else if (dphase < -128) dphase += 256;
    if (dphase < 0) dphase = -dphase;
    if (dlevel < 0) dlevel = -dlevel;
    if (dphase > (int32_t)threshold || dlevel > (int32_t)threshold) return 1u;
  }
  return 0u;
}

static void hologram_store_last(umh_hologram_engine_t *engine, const uint8_t *phase,
                                const uint8_t *level, uint64_t now_us)
{
  memcpy(engine->last_phase, phase, UMH_HOLOGRAM_CHANNELS);
  memcpy(engine->last_level, level, UMH_HOLOGRAM_CHANNELS);
  engine->have_last = 1u;
  engine->last_submit_us = now_us;
}

static void hologram_finish_stop(umh_hologram_engine_t *engine)
{
  engine->state = engine->configured != 0u ? UMH_HOLOGRAM_STATE_READY : UMH_HOLOGRAM_STATE_OFF;
  engine->stop_requested = 0u;
  engine->fade_scale = 0.0f;
  engine->fade_step = 0.0f;
  engine->next_due_us = 0u;
  engine->last_service_us = 0u;
  hologram_clear_channel_state(engine);
}

void hologram_engine_init(umh_hologram_engine_t *engine)
{
  osMutexAttr_t attributes;
  if (engine == NULL) return;
  memset(engine, 0, sizeof(*engine));
  engine->output_rate_hz = 100u;
  engine->transition_ms = 3000u;
  engine->level = UMH_HOLOGRAM_LEVEL_FULL;
  engine->state = UMH_HOLOGRAM_STATE_OFF;
  attributes.name = "hologram";
  attributes.attr_bits = osMutexRecursive | osMutexPrioInherit;
  attributes.cb_mem = &engine->lock_memory;
  attributes.cb_size = sizeof(engine->lock_memory);
  engine->lock = osMutexNew(&attributes);
  if (hologram_level_lut_ready == 0u) hologram_build_level_lut();
  hologram_clear_channel_state(engine);
}

int hologram_engine_upload(umh_hologram_engine_t *engine, const uint8_t *payload,
                           uint16_t length)
{
  umh_hologram_upload_wire_t header;
  uint16_t count;
  uint16_t first;
  uint16_t i;
  uint32_t expected;
  if (engine == NULL || payload == NULL) return -1;
  if (length < sizeof(header)) return -1;
  memcpy(&header, payload, sizeof(header));
  if (header.version != UMH_HOLOGRAM_WIRE_VERSION || header.flags != 0u) return -2;
  first = header.first_index;
  count = header.keyframe_count;
  if (count == 0u || count > UMH_HOLOGRAM_UPLOAD_MAX_PER_MSG) return -3;
  if ((uint32_t)first + (uint32_t)count > UMH_HOLOGRAM_MAX_KEYFRAMES) return -4;
  expected = (uint32_t)sizeof(header) + (uint32_t)count * UMH_HOLOGRAM_KEYFRAME_BYTES;
  if ((uint32_t)length != expected) return -5;
  if (hologram_lock(engine) != 0) return -6;
  for (i = 0u; i < count; ++i) {
    const uint8_t *slot = payload + sizeof(header) +
                          (uint32_t)i * UMH_HOLOGRAM_KEYFRAME_BYTES;
    memcpy(engine->phase[first + i], slot, UMH_HOLOGRAM_CHANNELS);
    memcpy(engine->level_table[first + i], slot + UMH_HOLOGRAM_CHANNELS,
           UMH_HOLOGRAM_CHANNELS);
    engine->keyframe_valid[first + i] = 1u;
  }
  /* Lowest free slot wins the default count, so a host that uploads chunks out
   * of order still ends up with the sequence it meant. */
  if (engine->keyframes < first + count) {
    uint16_t slot = 0u;
    while (slot < UMH_HOLOGRAM_MAX_KEYFRAMES && engine->keyframe_valid[slot] != 0u) ++slot;
    if (slot == 0u) slot = first + count;
    engine->keyframes = first + count;
  }
  if (engine->state == UMH_HOLOGRAM_STATE_OFF) engine->state = UMH_HOLOGRAM_STATE_READY;
  hologram_unlock(engine);
  return 0;
}

int hologram_engine_configure(umh_hologram_engine_t *engine, const uint8_t *payload,
                              uint16_t length)
{
  umh_hologram_config_wire_t wire;
  uint16_t rate;
  uint8_t keyframes;
  if (engine == NULL || payload == NULL) return -1;
  if (length != sizeof(wire)) return -1;
  memcpy(&wire, payload, sizeof(wire));
  if (wire.version != UMH_HOLOGRAM_WIRE_VERSION) return -2;
  if ((wire.flags & (uint8_t)~0x0Fu) != 0u) return -3;
  if ((wire.flags & UMH_HOLOGRAM_FLAG_GAMMA) != 0u && hologram_level_lut_ready == 0u)
    hologram_build_level_lut();
  rate = wire.output_rate_hz;
  if (rate < UMH_HOLOGRAM_MIN_RATE_HZ) rate = UMH_HOLOGRAM_MIN_RATE_HZ;
  if (rate > UMH_HOLOGRAM_MAX_RATE_HZ) rate = UMH_HOLOGRAM_MAX_RATE_HZ;
  if (wire.transition_ms > UMH_HOLOGRAM_MAX_TRANSITION_MS) return -4;
  if (wire.level > UMH_HOLOGRAM_LEVEL_FULL) return -5;
  keyframes = wire.keyframes;
  if (keyframes == 0u) keyframes = 1u;
  if (keyframes > UMH_HOLOGRAM_MAX_KEYFRAMES) return -6;
  if (hologram_lock(engine) != 0) return -7;
  engine->output_rate_hz = rate;
  engine->loop_ms = wire.loop_ms;
  engine->transition_ms = wire.transition_ms;
  engine->level = wire.level;
  engine->keyframes = keyframes;
  engine->flags = (uint8_t)(wire.flags & 0x0Fu);
  engine->configured = 1u;
  if (engine->keyframes <= 1u) engine->flags |= UMH_HOLOGRAM_FLAG_STATIC;
  else engine->flags &= (uint8_t)~UMH_HOLOGRAM_FLAG_STATIC;
  engine->period_us = hologram_period_us(engine);
  if (engine->state == UMH_HOLOGRAM_STATE_OFF) engine->state = UMH_HOLOGRAM_STATE_READY;
  hologram_unlock(engine);
  return 0;
}

void hologram_engine_start(umh_hologram_engine_t *engine)
{
  if (engine == NULL) return;
  if (hologram_lock(engine) != 0) return;
  engine->state = UMH_HOLOGRAM_STATE_RUNNING;
  engine->stop_requested = 0u;
  engine->run_origin_us = 0u;
  engine->last_service_us = 0u;
  engine->last_submit_us = 0u;
  engine->next_due_us = 0u;
  engine->period_us = hologram_period_us(engine);
  /* 20 ms linear fade-in: the first frame after START is silence, which keeps a
   * power-up transient out of the transducer bank. */
  engine->fade_scale = 0.0f;
  engine->fade_step = (float)engine->period_us /
                      (float)(UMH_HOLOGRAM_STOP_FADE_MS * 1000u);
  if (engine->fade_step <= 0.0f) engine->fade_step = 1.0f;
  engine->frames_submitted = 0u;
  engine->frames_skipped = 0u;
  engine->missed_deadlines = 0u;
  engine->frame_errors = 0u;
  engine->fps = 0u;
  engine->fps_frames = 0u;
  engine->fps_window_start_us = 0u;
  engine->service_max_cycles = 0u;
  engine->service_sum_cycles = 0u;
  engine->service_count = 0u;
  engine->render_max_cycles = 0u;
  engine->render_sum_cycles = 0u;
  engine->submit_max_cycles = 0u;
  engine->submit_sum_cycles = 0u;
  hologram_clear_channel_state(engine);
  hologram_unlock(engine);
}

void hologram_engine_request_stop(umh_hologram_engine_t *engine)
{
  if (engine == NULL) return;
  if (hologram_lock(engine) != 0) return;
  if (engine->state == UMH_HOLOGRAM_STATE_RUNNING) {
    engine->state = UMH_HOLOGRAM_STATE_STOPPING;
    engine->fade_step = (float)engine->period_us /
                        (float)(UMH_HOLOGRAM_STOP_FADE_MS * 1000u);
    if (engine->fade_step <= 0.0f) engine->fade_step = 1.0f;
  }
  hologram_unlock(engine);
}

void hologram_engine_abort(umh_hologram_engine_t *engine, fpga_link_t *link)
{
  if (engine == NULL) return;
  if (hologram_lock(engine) != 0) return;
  hologram_finish_stop(engine);
  hologram_unlock(engine);
  if (link != NULL) (void)fpga_link_safe_stop(link);
}

uint8_t hologram_engine_owns_output(const umh_hologram_engine_t *engine)
{
  if (engine == NULL) return 0u;
  return (engine->state == UMH_HOLOGRAM_STATE_RUNNING ||
          engine->state == UMH_HOLOGRAM_STATE_STOPPING) ? 1u : 0u;
}

uint8_t hologram_engine_is_active(const umh_hologram_engine_t *engine)
{
  return hologram_engine_owns_output(engine);
}

uint32_t hologram_engine_service(umh_hologram_engine_t *engine, fpga_link_t *link,
                                 uint64_t now_us, uint32_t *wait_us)
{
  umh_output_frame_t frame;
  uint8_t phase[UMH_HOLOGRAM_CHANNELS];
  uint8_t level[UMH_HOLOGRAM_CHANNELS];
  uint16_t index = 0u;
  uint32_t alpha = 0u;
  uint32_t fade;
  uint64_t period;
  int result;
  if (wait_us == NULL) return 0u;
  *wait_us = 1000u;
  if (engine == NULL || link == NULL) return 0u;
  if (hologram_lock(engine) != 0) return 0u;
  if (engine->state != UMH_HOLOGRAM_STATE_RUNNING &&
      engine->state != UMH_HOLOGRAM_STATE_STOPPING) {
    hologram_unlock(engine);
    return 0u;
  }
  period = engine->period_us != 0u ? engine->period_us : hologram_period_us(engine);
  if (now_us < engine->next_due_us) {
    uint64_t delta = engine->next_due_us - now_us;
    *wait_us = delta > 100000u ? 100000u : (uint32_t)delta;
    hologram_unlock(engine);
    return 0u;
  }
  if (engine->run_origin_us == 0u) {
    engine->run_origin_us = now_us;
    engine->last_service_us = now_us;
    engine->next_due_us = now_us;
  }
  if (engine->last_submit_us == 0u) engine->last_submit_us = now_us;
  engine->last_service_us = now_us;

  if (engine->state == UMH_HOLOGRAM_STATE_RUNNING &&
      engine->fade_scale < 1.0f) {
    engine->fade_scale += engine->fade_step;
    if (engine->fade_scale > 1.0f) engine->fade_scale = 1.0f;
  }
  if (engine->state == UMH_HOLOGRAM_STATE_STOPPING) {
    engine->fade_scale -= engine->fade_step;
    if (engine->fade_scale < 0.0f) engine->fade_scale = 0.0f;
  }

  if (link->status.fifo_credit == 0u) {
    /* status.fifo_credit is one transaction stale, so a zero here is usually a
     * sampling artefact rather than a full FIFO.  Poll the live value and give
     * up the slot if the FPGA really is saturated. */
    (void)fpga_link_poll_status(link);
    if (link->status.fifo_credit == 0u) {
      *wait_us = 250u;
      hologram_unlock(engine);
      return 0u;
    }
  }

  fade = hologram_fade_gain_x1000(engine);
  {
    uint32_t service_start_cycles = DWT->CYCCNT;
    uint32_t render_start_cycles = DWT->CYCCNT;
    index = hologram_keyframe_index(engine, now_us - engine->run_origin_us, &alpha);
    hologram_compose(engine, index, alpha, fade, phase, level);
    hologram_record_render(engine, DWT->CYCCNT - render_start_cycles);

    if (hologram_needs_submit(engine, phase, level, now_us) == 0u) {
      engine->frames_skipped++;
      hologram_record_service(engine, service_start_cycles);
      engine->next_due_us = now_us + period;
      *wait_us = (uint32_t)period;
      hologram_unlock(engine);
      return 0u;
    }

    memset(&frame, 0, sizeof(frame));
    frame.sequence = engine->frame_sequence++;
    frame.deadline = now_us;
    frame.update_flags = UMH_FRAME_FLAG_ULTRASOUND;
    frame.digital_mask = 0u;
    frame.digital_state = 0u;
    frame.extension_length = 0u;
    {
      uint16_t i;
      for (i = 0u; i < UMH_HOLOGRAM_CHANNELS; ++i) {
        frame.channels[i].phase = phase[i];
        frame.channels[i].level = level[i];
      }
    }
    {
      uint32_t submit_start_cycles = DWT->CYCCNT;
      result = fpga_link_submit_allow_hold(link, &frame, 1u);
      hologram_record_submit(engine, DWT->CYCCNT - submit_start_cycles);
    }
    hologram_record_service(engine, service_start_cycles);
    if (result != 0) {
      engine->frame_errors++;
      engine->next_due_us = now_us + 1000u;
      *wait_us = 1000u;
      hologram_unlock(engine);
      return 0u;
    }
    hologram_store_last(engine, phase, level, now_us);
    engine->frames_submitted++;
    engine->fps_frames++;
    if (engine->fps_window_start_us == 0u) engine->fps_window_start_us = now_us;
    if (now_us - engine->fps_window_start_us >= 1000000u) {
      uint64_t elapsed = now_us - engine->fps_window_start_us;
      engine->fps = (uint32_t)(((uint64_t)engine->fps_frames * 100000000u) / elapsed);
      engine->fps_frames = 0u;
      engine->fps_window_start_us = now_us;
    }
  }

  if (engine->state == UMH_HOLOGRAM_STATE_STOPPING && engine->fade_scale <= 0.0f) {
    hologram_finish_stop(engine);
    hologram_unlock(engine);
    (void)fpga_link_safe_stop(link);
    *wait_us = 1000u;
    return 0u;
  }

  if (engine->next_due_us == 0u || engine->next_due_us == now_us) {
    engine->next_due_us = now_us + period;
  } else {
    engine->next_due_us += period;
    if (engine->next_due_us <= now_us) {
      uint64_t behind = now_us - engine->next_due_us;
      uint32_t missed = (uint32_t)(behind / period) + 1u;
      engine->missed_deadlines += missed;
      engine->next_due_us = now_us + period;
    }
  }
  {
    uint64_t wait = engine->next_due_us > now_us ? engine->next_due_us - now_us : 250u;
    *wait_us = wait > 100000u ? 100000u : (uint32_t)wait;
    if (*wait_us == 0u) *wait_us = 250u;
  }
  hologram_unlock(engine);
  return 0u;
}

void hologram_engine_get_status(const umh_hologram_engine_t *engine,
                                umh_hologram_status_wire_t *status)
{
  uint32_t cycles_per_us;
  if (status == NULL) return;
  memset(status, 0, sizeof(*status));
  if (engine == NULL) return;
  if (hologram_lock((umh_hologram_engine_t *)engine) != 0) return;
  status->state = engine->state;
  status->flags = engine->flags;
  status->output_rate_hz = engine->output_rate_hz;
  status->keyframe_count = engine->keyframes;
  status->current_keyframe = hologram_keyframe_index(
      engine, engine->run_origin_us != 0u && engine->last_service_us >= engine->run_origin_us ?
              engine->last_service_us - engine->run_origin_us : 0u, NULL);
  {
    uint32_t alpha = 0u;
    (void)hologram_keyframe_index(
        engine, engine->run_origin_us != 0u && engine->last_service_us >= engine->run_origin_us ?
                engine->last_service_us - engine->run_origin_us : 0u, &alpha);
    status->alpha_x1000 = (uint16_t)alpha;
  }
  cycles_per_us = SystemCoreClock / 1000000u;
  if (cycles_per_us == 0u) cycles_per_us = 1u;
  {
    uint32_t max_us = engine->service_max_cycles / cycles_per_us;
    uint32_t avg_us = 0u;
    status->service_max_us = max_us > 65535u ? 65535u : (uint16_t)max_us;
    if (engine->service_count != 0u)
      avg_us = (engine->service_sum_cycles / engine->service_count) / cycles_per_us;
    status->service_avg_us = avg_us > 65535u ? 65535u : (uint16_t)avg_us;
    max_us = engine->render_max_cycles / cycles_per_us;
    status->render_max_us = max_us > 65535u ? 65535u : (uint16_t)max_us;
    status->render_avg_us = engine->service_count != 0u ?
        (uint16_t)((engine->render_sum_cycles / engine->service_count) / cycles_per_us) : 0u;
    max_us = engine->submit_max_cycles / cycles_per_us;
    status->submit_max_us = max_us > 65535u ? 65535u : (uint16_t)max_us;
    status->submit_avg_us = engine->service_count != 0u ?
        (uint16_t)((engine->submit_sum_cycles / engine->service_count) / cycles_per_us) : 0u;
  }
  status->frames = engine->frames_submitted;
  status->skipped = engine->frames_skipped;
  status->missed_deadlines = engine->missed_deadlines;
  status->frame_errors = engine->frame_errors;
  status->fps_x100 = engine->fps;
  hologram_unlock((umh_hologram_engine_t *)engine);
}
