#ifndef DEMO_ENGINE_H
#define DEMO_ENGINE_H

#include <stdint.h>
#include "frame_ring.h"
#include "spatial_renderer.h"
#include "playback_plan.h"

/* Built-in demonstrations are compiled as ordinary output frames.  They do
 * not have a second stimulation/DMA path, so every demo benefits from the
 * same calibration, scheduling and FPGA safety rules as host supplied data. */
typedef enum {
  UMH_DEMO_ULM = 0u,
  UMH_DEMO_LML = 1u,
  UMH_DEMO_LMC = 2u,
  UMH_DEMO_COUNT
} umh_demo_id_t;

typedef struct {
  umh_demo_id_t id;
  const char *name;
  const char *description;
} umh_demo_descriptor_t;

const umh_demo_descriptor_t *demo_engine_descriptor(uint8_t id);
uint8_t demo_engine_count(void);
int demo_engine_build(uint8_t id, umh_spatial_renderer_t *renderer,
                      umh_frame_ring_t *frames, uint64_t origin_time,
                      umh_playback_plan_t *plan);

#endif
