#include "playback_plan.h"
#include <string.h>

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
  if (plan == NULL || plan->running == 0u) return;
  plan->output_time = device_time;
  plan->source_time += ((uint64_t)elapsed_us * plan->wire.rate_numerator) /
                      plan->wire.rate_denominator;
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

uint64_t playback_plan_map_time(const umh_playback_plan_t *plan, uint64_t output_time)
{
  uint64_t elapsed;
  uint64_t scaled;
  if (plan == NULL || plan->wire.rate_denominator == 0u) return 0u;
  if (output_time < plan->origin_output_time) return plan->origin_source_time;
  elapsed = output_time - plan->origin_output_time;
  if (elapsed > UINT64_MAX / plan->wire.rate_numerator) return UINT64_MAX;
  scaled = (elapsed * plan->wire.rate_numerator) / plan->wire.rate_denominator;
  if (plan->origin_source_time > UINT64_MAX - scaled) return UINT64_MAX;
  return plan->origin_source_time + scaled;
}

uint8_t playback_plan_frame_due(umh_playback_plan_t *plan, uint64_t frame_source_time,
                                uint64_t device_time)
{
  uint64_t elapsed_source;
  uint64_t elapsed_output;
  if (plan == NULL || plan->running == 0u) return 0u;
  if (plan->origin_valid == 0u) {
    plan->origin_output_time = device_time;
    if (plan->wire.start_mode == UMH_PLAN_START_DEVICE_TIME &&
        plan->wire.start_time > plan->origin_output_time)
      plan->origin_output_time = plan->wire.start_time;
    plan->origin_valid = 1u;
  }
  if (frame_source_time < plan->origin_source_time || plan->wire.rate_numerator == 0u) return 0u;
  elapsed_source = frame_source_time - plan->origin_source_time;
  elapsed_output = (elapsed_source * plan->wire.rate_denominator) /
                   plan->wire.rate_numerator;
  if (device_time < plan->origin_output_time) return 0u;
  return device_time - plan->origin_output_time >= elapsed_output ? 1u : 0u;
}

void playback_plan_frame_submitted(umh_playback_plan_t *plan)
{
  if (plan != NULL) ++plan->current_frame;
}

void playback_plan_notify_trigger(umh_playback_plan_t *plan)
{
  if (plan != NULL) {
    plan->trigger_seen = 1u;
    if (plan->configured != 0u && plan->prebuffered != 0u &&
        plan->wire.start_mode == UMH_PLAN_START_TRIGGER) {
      plan->running = 1u;
      plan->origin_valid = 0u;
    }
  }
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

uint8_t playback_plan_is_terminal(const umh_playback_plan_t *plan)
{
  if (plan == NULL) return 1u;
  if (plan->wire.repeat_mode == UMH_PLAN_LOOP_RAM &&
      (plan->wire.loop_count == 0u || plan->loop_iteration < plan->wire.loop_count)) return 0u;
  return plan->wire.repeat_mode == UMH_PLAN_LOOP_STREAM ? 0u : 1u;
}
