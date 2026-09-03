#ifndef PLAYBACK_PLAN_H
#define PLAYBACK_PLAN_H

#include <stdint.h>
#include "frame_ring.h"

typedef enum {
  UMH_PLAN_START_IMMEDIATE = 0u,
  UMH_PLAN_START_DEVICE_TIME = 1u,
  UMH_PLAN_START_TRIGGER = 2u,
  UMH_PLAN_START_FRAME_BOUNDARY = 3u
} umh_plan_start_t;

typedef enum {
  UMH_PLAN_ONCE = 0u,
  UMH_PLAN_LOOP_RAM = 1u,
  UMH_PLAN_LOOP_STREAM = 2u,
  UMH_PLAN_HOLD_LAST = 3u,
  UMH_PLAN_STOP = 4u
} umh_plan_repeat_t;

typedef enum {
  UMH_UNDERRUN_HOLD = 0u,
  UMH_UNDERRUN_DISABLE = 1u,
  UMH_UNDERRUN_REPORT = 2u
} umh_underrun_policy_t;

/* The present board exposes one FPGA output route.  Other route identifiers
 * are intentionally rejected until a corresponding hardware endpoint exists. */
#define UMH_PLAN_ROUTE_DEFAULT 0u

typedef struct __attribute__((packed)) {
  uint32_t block_id;
  uint8_t start_mode;
  uint8_t repeat_mode;
  uint8_t underrun_policy;
  uint8_t route;
  uint32_t rate_numerator;
  uint32_t rate_denominator;
  uint64_t start_time;
  uint32_t prebuffer_frames;
  uint32_t loop_count;
} umh_playback_plan_wire_t;

_Static_assert(sizeof(umh_playback_plan_wire_t) == 32u,
               "playback plan wire size");

typedef struct {
  umh_playback_plan_wire_t wire;
  uint8_t configured;
  uint8_t running;
  uint8_t prebuffered;
  uint8_t trigger_seen;
  uint64_t source_time;
  uint64_t output_time;
  uint64_t origin_source_time;
  uint64_t origin_output_time;
  uint8_t origin_valid;
  uint8_t underrun_reported;
  uint8_t underrun_active;
  uint8_t loop_prepared;
  uint32_t current_frame;
  uint32_t loop_iteration;
} umh_playback_plan_t;

int playback_plan_set(umh_playback_plan_t *plan, const umh_playback_plan_wire_t *wire);
void playback_plan_clear(umh_playback_plan_t *plan);
int playback_plan_start(umh_playback_plan_t *plan, uint64_t device_time,
                        uint16_t buffered_frames, uint64_t source_origin_time);
void playback_plan_stop(umh_playback_plan_t *plan);
void playback_plan_tick(umh_playback_plan_t *plan, uint64_t device_time,
                        uint32_t elapsed_us, uint16_t buffered_frames);
uint64_t playback_plan_map_time(const umh_playback_plan_t *plan, uint64_t output_time);
uint8_t playback_plan_frame_due(umh_playback_plan_t *plan, uint64_t frame_source_time,
                                uint64_t device_time);
void playback_plan_frame_submitted(umh_playback_plan_t *plan);
void playback_plan_notify_trigger(umh_playback_plan_t *plan);
int playback_plan_prepare_loop(umh_playback_plan_t *plan,
                               umh_frame_ring_t *frames,
                               uint64_t device_time);
uint8_t playback_plan_is_terminal(const umh_playback_plan_t *plan);

#endif
