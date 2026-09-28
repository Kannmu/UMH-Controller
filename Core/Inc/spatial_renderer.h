#ifndef SPATIAL_RENDERER_H
#define SPATIAL_RENDERER_H

#include <stdint.h>
#include "device_profile.h"
#include "frame_ring.h"
#include "spatiotemporal_block.h"
#include "umh_fast_math.h"

typedef struct {
  const umh_device_profile_t *profile;
  umh_channel_calibration_t calibration[UMH_DEVICE_CHANNEL_COUNT];
  /* Per-channel gain and calibration phase pre-scaled to the phase table
   * domain, so the per-frame inner loop is load/FMA only. */
  float gain_scale[UMH_DEVICE_CHANNEL_COUNT];
  int32_t phase_offset_q10[UMH_DEVICE_CHANNEL_COUNT];
  uint8_t rgb_gain[UMH_DEVICE_RGB_COUNT][3];
  uint32_t carrier_hz;
  uint32_t sound_speed_um_per_s;
  uint32_t phase_resolution;
} umh_spatial_renderer_t;

/* Shared per-channel complex accumulation used by every renderer producer.
 * phase_q10 is the absolute phase in 1/1024 carrier turn, i.e. it already
 * contains the calibration phase offset.  amplitude is the point/source level
 * before the per-channel calibration gain is applied, matching the historical
 * renderer arithmetic exactly.  static inline keeps every producer's inner
 * loop load/FMA only. */
static inline void spatial_renderer_accumulate_q10(
    const umh_spatial_renderer_t *renderer, uint16_t index,
    int32_t phase_q10, float amplitude,
    float *real_accum, float *imag_accum)
{
  float mag = amplitude * renderer->gain_scale[index];
  real_accum[index] += mag * umh_fast_cos_q10(phase_q10);
  imag_accum[index] += mag * umh_fast_sin_q10(phase_q10);
}

void spatial_renderer_init(umh_spatial_renderer_t *renderer,
                           const umh_device_profile_t *profile);
void spatial_renderer_set_calibration(umh_spatial_renderer_t *renderer,
                                      const umh_channel_calibration_t *calibration,
                                      uint16_t count);
void spatial_renderer_refresh_calibration(umh_spatial_renderer_t *renderer,
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

