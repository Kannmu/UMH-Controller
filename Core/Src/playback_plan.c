#include "playback_plan.h"
#include <string.h>

static uint32_t rate_numerator_key;
static uint32_t rate_denominator_key;
static uint64_t rate_numerator_recip;
static uint64_t rate_denominator_recip;

static uint64_t rate_mulhi(uint64_t a, uint64_t b)
{
  uint64_t a0 = a & 0xFFFFFFFFu;
  uint64_t a1 = a >> 32;
  uint64_t b0 = b & 0xFFFFFFFFu;
  uint64_t b1 = b >> 32;
  uint64_t p0 = a0 * b0;
  uint64_t p1 = a0 * b1;
  uint64_t p2 = a1 * b0;
  uint64_t p3 = a1 * b1;
  uint64_t mid = (p0 >> 32) + (p1 & 0xFFFFFFFFu) + (p2 & 0xFFFFFFFFu);
  return p3 + (p1 >> 32) + (p2 >> 32) + (mid >> 32);
}

static uint64_t rate_mulhi_u32(uint64_t a, uint32_t b)
{
  uint64_t lo = (uint64_t)(uint32_t)a * b;
  uint64_t hi = (a >> 32) * b;
  return (hi + (lo >> 32)) >> 32;
}

static uint64_t rate_reciprocal(uint32_t divisor)
{
  return divisor > 1u ? UINT64_MAX / (uint64_t)divisor + 1u : 0u;
}

static uint64_t rate_divide(uint64_t numerator, uint32_t divisor, uint64_t reciprocal)
{
  uint64_t quotient;
  if (divisor <= 1u) return numerator;
  quotient = rate_mulhi(numerator, reciprocal);
  if (rate_mulhi_u32(quotient, divisor) != 0u || quotient * divisor > numerator)
    quotient -= 1u;
  return quotient;
}

static void rate_reciprocals_refresh(uint32_t numerator, uint32_t denominator)
{
  if (numerator != rate_numerator_key) {
    rate_numerator_recip = rate_reciprocal(numerator);
    rate_numerator_key = numerator;
  }
  if (denominator != rate_denominator_key) {
    rate_denominator_recip = rate_reciprocal(denominator);
    rate_denominator_key = denominator;
  }
}

int playback_plan_set(umh_playback_plan_t *plan, const umh_playback_plan_wire_t *wire)
{
  if (plan == NULL || wire == NULL || wire->rate_numerator == 0u || wire->rate_denominator == 0u ||
      wire->start_mode > UMH_PLAN_START_FRAME_BOUNDARY ||
      wire->repeat_mode > UMH_PLAN_STOP || wire->underrun_policy > UMH_UNDERRUN_REPORT ||
      wire->route != 0u) return -1;
  memcpy(&plan->wire, wire, sizeof(*wire));
  plan->configured = 1u;
  plan->running = 0u;
  plan->prebuffered = 0u;
  plan->trigger_seen = 0u;
  plan->source_time = 0u;
  plan->output_time = 0u;
  plan->origin_source_time = 0u;
  plan->origin_output_time = 0u;
  plan->origin_valid = 0u;
  plan->underrun_reported = 0u;
  plan->underrun_active = 0u;
  plan->loop_prepared = 0u;
  plan->current_frame = 0u;
  plan->loop_iteration = 0u;
  return 0;
}

void playback_plan_clear(umh_playback_plan_t *plan)
{
  if (plan != NULL) memset(plan, 0, sizeof(*plan));
}

int playback_plan_start(umh_playback_plan_t *plan, uint64_t device_time,
                        uint16_t buffered_frames, uint64_t source_origin_time)
{
  if (plan == NULL || plan->configured == 0u) return -1;
  if (buffered_frames < plan->wire.prebuffer_frames) return -2;
  plan->running = plan->wire.start_mode == UMH_PLAN_START_TRIGGER ? 0u : 1u;
  plan->prebuffered = 1u;
  plan->output_time = device_time;
  plan->source_time = source_origin_time;
  plan->origin_source_time = source_origin_time;
  plan->origin_valid = 0u;
  plan->underrun_reported = 0u;
  plan->underrun_active = 0u;
  plan->loop_iteration = 0u;
  plan->loop_prepared = 0u;
  return 0;
}

void playback_plan_stop(umh_playback_plan_t *plan)
{
  if (plan != NULL) { plan->running = 0u; plan->origin_valid = 0u; plan->prebuffered = 0u; }
}

void playback_plan_tick(umh_playback_plan_t *plan, uint64_t device_time,
                        uint32_t elapsed_us, uint16_t buffered_frames)
{
  uint32_t numerator;
  uint32_t denominator;
  if (plan == NULL || plan->running == 0u) return;
  numerator = plan->wire.rate_numerator;
  denominator = plan->wire.rate_denominator;
  plan->output_time = device_time;
  if (numerator == denominator) {
    /* Unity rate is the normal case and must not pay a 64-bit division on
     * every render-loop iteration (the render task calls this per frame). */
    plan->source_time += elapsed_us;
  } else {
    rate_reciprocals_refresh(numerator, denominator);
    plan->source_time += rate_divide((uint64_t)elapsed_us * numerator, denominator,
                                     rate_denominator_recip);
  }
  if (buffered_frames == 0u && plan->current_frame != 0u) {
    if (plan->underrun_active == 0u) {
      plan->underrun_active = 1u;
      plan->underrun_reported = 1u;
    }
    if (plan->wire.underrun_policy == UMH_UNDERRUN_DISABLE) plan->running = 0u;
  } else if (buffered_frames != 0u) {
    plan->underrun_active = 0u;
  }
}

uint8_t playback_plan_frame_due(umh_playback_plan_t *plan, uint64_t frame_source_time,
                                uint64_t device_time)
{
  uint64_t elapsed_source;
  uint64_t elapsed_output;
  uint32_t numerator;
  uint32_t denominator;
  if (plan == NULL || plan->running == 0u) return 0u;
  numerator = plan->wire.rate_numerator;
  denominator = plan->wire.rate_denominator;
  if (plan->origin_valid == 0u) {
    plan->origin_output_time = device_time;
    if (plan->wire.start_mode == UMH_PLAN_START_DEVICE_TIME &&
        plan->wire.start_time > plan->origin_output_time)
      plan->origin_output_time = plan->wire.start_time;
    plan->origin_valid = 1u;
  }
  if (frame_source_time < plan->origin_source_time || numerator == 0u) return 0u;
  elapsed_source = frame_source_time - plan->origin_source_time;
  if (numerator == denominator) {
    elapsed_output = elapsed_source;
  } else {
    rate_reciprocals_refresh(numerator, denominator);
    elapsed_output = rate_divide(elapsed_source * denominator, numerator, rate_numerator_recip);
  }
  if (device_time < plan->origin_output_time) return 0u;
  return device_time - plan->origin_output_time >= elapsed_output ? 1u : 0u;
}

uint8_t playback_plan_frame_output_time(const umh_playback_plan_t *plan,
                                    uint64_t frame_source_time,
                                    uint64_t *output_time)
{
  uint64_t elapsed_source;
  uint64_t elapsed_output;
  uint32_t numerator;
  uint32_t denominator;
  if (plan == NULL || output_time == NULL || plan->configured == 0u ||
      plan->origin_valid == 0u || plan->wire.rate_numerator == 0u ||
      frame_source_time < plan->origin_source_time) return 0u;
  numerator = plan->wire.rate_numerator;
  denominator = plan->wire.rate_denominator;
  elapsed_source = frame_source_time - plan->origin_source_time;
  if (numerator == denominator) {
    elapsed_output = elapsed_source;
  } else {
    rate_reciprocals_refresh(numerator, denominator);
    if (rate_denominator_recip != 0u && elapsed_source >= rate_denominator_recip) return 0u;
    elapsed_output = rate_divide(elapsed_source * denominator, numerator, rate_numerator_recip);
  }
  if (plan->origin_output_time > UINT64_MAX - elapsed_output) return 0u;
  *output_time = plan->origin_output_time + elapsed_output;
  return 1u;
}

void playback_plan_frame_submitted(umh_playback_plan_t *plan)
{
  if (plan != NULL) ++plan->current_frame;
}

int playback_plan_prepare_loop(umh_playback_plan_t *plan,
                               umh_frame_ring_t *frames,
                               uint64_t device_time)
{
  uint32_t next_iteration;
  if (plan == NULL || frames == NULL || plan->running == 0u) return 0;
  if (plan->wire.repeat_mode != UMH_PLAN_LOOP_RAM || frame_ring_loop_valid(frames) == 0u) return 0;
  if (plan->wire.loop_count != 0u && plan->loop_iteration >= plan->wire.loop_count) return 0;
  next_iteration = plan->loop_iteration + 1u;
  if (frame_ring_restore_loop(frames, next_iteration) != 0) return -1;
  plan->loop_iteration = next_iteration;
  plan->loop_prepared = 1u;
  plan->output_time = device_time;
  plan->underrun_active = 0u;
  return 1;
}
