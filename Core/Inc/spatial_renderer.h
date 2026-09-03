#ifndef SPATIAL_RENDERER_H
#define SPATIAL_RENDERER_H

#include <stdint.h>
#include "device_profile.h"
#include "frame_ring.h"
#include "spatiotemporal_block.h"

typedef struct {
  const umh_device_profile_t *profile;
  umh_channel_calibration_t calibration[UMH_DEVICE_CHANNEL_COUNT];
  uint8_t rgb_gain[UMH_DEVICE_RGB_COUNT][3];
  uint32_t carrier_hz;
  uint32_t sound_speed_um_per_s;
  uint32_t phase_resolution;
} umh_spatial_renderer_t;

void spatial_renderer_init(umh_spatial_renderer_t *renderer,
                           const umh_device_profile_t *profile);
void spatial_renderer_set_calibration(umh_spatial_renderer_t *renderer,
                                      const umh_channel_calibration_t *calibration,
                                      uint16_t count);
int spatial_renderer_point(umh_spatial_renderer_t *renderer,
                           const umh_spatial_point_t *point,
                           umh_output_frame_t *frame);
int spatial_renderer_accumulate_point(const umh_spatial_renderer_t *renderer,
                                      const umh_spatial_point_t *point,
                                      float *real_accum, float *imag_accum);
int spatial_renderer_finalize(const umh_spatial_renderer_t *renderer,
                              const float *real_accum, const float *imag_accum,
                              umh_output_frame_t *frame);
void spatial_renderer_set_rgb_calibration(umh_spatial_renderer_t *renderer,
                                          const uint8_t gain[UMH_DEVICE_RGB_COUNT][3]);
void spatial_renderer_merge_rgb(const umh_spatial_renderer_t *renderer,
                                umh_output_frame_t *frame,
                                const umh_rgb_value_t *color,
                                const uint8_t *levels,
                                uint8_t target_mask);

#endif
