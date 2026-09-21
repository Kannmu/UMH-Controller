#include "demo_engine.h"
#include "spatiotemporal_block.h"
#include "system_status.h"
#include <math.h>
#include <string.h>

#define DEMO_FRAME_COUNT UMH_DEVICE_FRAME_RING_SLOTS
#define DEMO_TRAJECTORY_PERIOD_US 5000u
#define DEMO_STRENGTH 255u
#define DEMO_PHASE 0u
#define DEMO_Z_UM 100000
#define DEMO_TRIGGER_MASK 0x01u

/* Every demo must cover the same total path length in one 5 ms trajectory
 * period.  The tactile requirement is 30 mm per period:
 *   ULM: one unidirectional 30 mm sweep.
 *   LML: 15 mm end-to-end stroke (one way), 15 mm back, 30 mm total.
 *   LMC: full circumference 30 mm -> radius 30000/(2*pi) = 4.77465 mm. */
#define DEMO_TOTAL_PATH_UM        30000.0f
#define DEMO_ULM_HALF_UM          15000.0f
#define DEMO_LML_HALF_UM           7500.0f
#define DEMO_LMC_RADIUS_UM        (DEMO_TOTAL_PATH_UM / 6.28318530717958647692f)

static const umh_demo_descriptor_t demos[UMH_DEMO_COUNT] = {
  { UMH_DEMO_ULM, "ULM", "30 mm unidirectional sweep" },
  { UMH_DEMO_LML, "LML", "15 mm end-to-end, 30 mm round trip" },
  { UMH_DEMO_LMC, "LMC", "30 mm circumference (r=4.775 mm)" }
};

const umh_demo_descriptor_t *demo_engine_descriptor(uint8_t id)
{
  return id < UMH_DEMO_COUNT ? &demos[id] : NULL;
}

uint8_t demo_engine_count(void) { return UMH_DEMO_COUNT; }

static void point_for(uint8_t id, uint16_t index, umh_spatial_point_t *point)
{
  float x = 0.0f;
  float y = 0.0f;
  const uint16_t last = (uint16_t)(DEMO_FRAME_COUNT - 1u);
  if (id == UMH_DEMO_LMC) {
    /* Full circle: sample t in [0,1] inclusive.  Frame 23 duplicates frame 0
     * at the loop boundary, so one period covers the complete circumference
     * continuously (r = 4.77465 mm -> 30.000 mm). */
    const float t = (float)index / (float)last;
    const float angle = 6.28318530717958647692f * t;
    x = DEMO_LMC_RADIUS_UM * cosf(angle);
    y = DEMO_LMC_RADIUS_UM * sinf(angle);
  } else if (id == UMH_DEMO_ULM) {
    /* One unidirectional +15 mm -> -15 mm sweep, total 30 mm.  The loop
     * returns to +15 mm as the next period starts (snap-back). */
    const float t = (float)index / (float)last;
    y = DEMO_ULM_HALF_UM * (1.0f - 2.0f * t);
  } else {
    /* Triangle wave with exactly +7.5 mm and -7.5 mm endpoints sampled.
     * Down leg: 12 intervals, up leg: 11 intervals; both legs are 15 mm, so
     * the total path is 30 mm per 5 ms period.  The tiny speed asymmetry
     * keeps both endpoints exact inside the fixed 24-frame ring. */
    if (index <= 12u) {
      y = DEMO_LML_HALF_UM - (2.0f * DEMO_LML_HALF_UM * (float)index / 12.0f);
    } else {
      y = -DEMO_LML_HALF_UM + (2.0f * DEMO_LML_HALF_UM * (float)(index - 12u) / 11.0f);
    }
  }
  point->x_um = (int32_t)lroundf(x);
  point->y_um = (int32_t)lroundf(y);
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
    slot->deadline = origin_time +
                     ((uint64_t)i * DEMO_TRAJECTORY_PERIOD_US) / DEMO_FRAME_COUNT;
    slot->sequence = i;
    if (spatial_renderer_point(renderer, &point, slot) != 0) {
      frame_ring_init(frames);
      return -3;
    }
    /* The renderer owns the ultrasound fields.  Add the PA9 power-stage gate
     * after rendering so it cannot be cleared while finalizing the frame. */
    slot->update_flags |= UMH_FRAME_FLAG_DIGITAL;
    slot->digital_mask = DEMO_TRIGGER_MASK;
    slot->digital_state = DEMO_TRIGGER_MASK;
    if (frame_ring_commit_write(frames) != 0) {
      frame_ring_init(frames);
      return -3;
    }
  }
  {
    umh_playback_plan_wire_t wire;
    uint64_t start_time = system_time_us();
    uint64_t shift;
    /* The frame generator needs a few milliseconds to render all channels
     * with sinf/cosf.  Deadlines that were anchored before that work are
     * already stale when playback begins, so the render task would spend
     * every loop catching up and never yield to the lower-priority UI
     * task.  Re-anchor the completed frame set to the real start time. */
    if (start_time < origin_time) start_time = origin_time;
    shift = start_time - origin_time;
    for (i = 0u; i < DEMO_FRAME_COUNT; ++i) frames->slots[i].deadline += shift;
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
        playback_plan_start(plan, start_time, DEMO_FRAME_COUNT, start_time) != 0 ||
        frame_ring_snapshot(frames, DEMO_TRAJECTORY_PERIOD_US) != 0)
      return -4;
  }
  return 0;
}
