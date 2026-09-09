#include "demo_engine.h"
#include "spatiotemporal_block.h"
#include <math.h>
#include <string.h>

#define DEMO_FRAME_COUNT 32u
#define DEMO_PERIOD_US 156u
#define DEMO_STRENGTH 255u
#define DEMO_PHASE 0u
#define DEMO_Z_UM 100000

static const umh_demo_descriptor_t demos[UMH_DEMO_COUNT] = {
  { UMH_DEMO_ULM, "ULM", "15 mm linear sweep" },
  { UMH_DEMO_LML, "LML", "7.5 mm linear sweep" },
  { UMH_DEMO_LMC, "LMC", "4.77 mm circular sweep" }
};

const umh_demo_descriptor_t *demo_engine_descriptor(uint8_t id)
{
  return id < UMH_DEMO_COUNT ? &demos[id] : NULL;
}

uint8_t demo_engine_count(void) { return UMH_DEMO_COUNT; }

static void point_for(uint8_t id, uint16_t index, umh_spatial_point_t *point)
{
  float t = (float)index / (float)DEMO_FRAME_COUNT;
  float x = 0.0f;
  float y = 0.0f;
  if (id == UMH_DEMO_LMC) {
    const float angle = 6.28318530717958647692f * t;
    x = 4770.0f * cosf(angle);
    y = 4770.0f * sinf(angle);
  } else {
    const float span = id == UMH_DEMO_LML ? 7500.0f : 15000.0f;
    /* One complete period is a forward and backward pass. */
    const float phase = t < 0.5f ? t * 2.0f : (1.0f - t) * 2.0f;
    y = span * (1.0f - 2.0f * phase);
  }
  point->x_um = (int32_t)x;
  point->y_um = (int32_t)y;
  point->z_um = DEMO_Z_UM;
  point->level = DEMO_STRENGTH;
  point->phase = DEMO_PHASE;
  point->source_id = 0u;
}

int demo_engine_build(uint8_t id, umh_spatial_renderer_t *renderer,
                      umh_frame_ring_t *frames, uint64_t origin_time,
                      umh_playback_plan_t *plan)
{
  uint16_t i;
  if (demo_engine_descriptor(id) == NULL || renderer == NULL ||
      frames == NULL || plan == NULL) return -1;
  frame_ring_init(frames);
  for (i = 0u; i < DEMO_FRAME_COUNT; ++i) {
    umh_output_frame_t *slot = frame_ring_acquire_write(frames);
    umh_spatial_point_t point;
    if (slot == NULL) return -2;
    memset(slot, 0, sizeof(*slot));
    point_for(id, i, &point);
    slot->deadline = origin_time + (uint64_t)i * DEMO_PERIOD_US;
    slot->sequence = i;
    if (spatial_renderer_point(renderer, &point, slot) != 0 ||
        frame_ring_commit_write(frames) != 0) {
      frame_ring_init(frames);
      return -3;
    }
  }
  {
    umh_playback_plan_wire_t wire;
    memset(&wire, 0, sizeof(wire));
    wire.block_id = 0x44454D00u | id;
    wire.start_mode = UMH_PLAN_START_IMMEDIATE;
    wire.repeat_mode = UMH_PLAN_LOOP_RAM;
    wire.underrun_policy = UMH_UNDERRUN_HOLD;
    wire.route = UMH_PLAN_ROUTE_DEFAULT;
    wire.rate_numerator = 1u;
    wire.rate_denominator = 1u;
    wire.prebuffer_frames = DEMO_FRAME_COUNT;
    wire.loop_count = 0u;
    if (playback_plan_set(plan, &wire) != 0 ||
        playback_plan_start(plan, origin_time, DEMO_FRAME_COUNT, origin_time) != 0 ||
        frame_ring_snapshot(frames, (uint64_t)DEMO_FRAME_COUNT * DEMO_PERIOD_US) != 0)
      return -4;
  }
  return 0;
}
