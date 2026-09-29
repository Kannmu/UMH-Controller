#ifndef SPATIAL_RENDERER_H
#define SPATIAL_RENDERER_H

#include <stdint.h>
#include <math.h>
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
  /* Geometry cache.  The element coordinates are wire integers in micrometres,
   * but every producer needs them as floats in the per-point inner loop.  The
   * conversions are invariant for the life of the renderer, so they are done
   * once here instead of 3 x 84 times per point. */
  float coord_x_um[UMH_DEVICE_CHANNEL_COUNT];
  float coord_y_um[UMH_DEVICE_CHANNEL_COUNT];
  float coord_z_um[UMH_DEVICE_CHANNEL_COUNT];
  /* Channels with calibration enabled, in ascending order.  Every producer
   * walks this list instead of testing the enable flag 84 times per point, so
   * the inner loop is branch-free and its trip count follows the real aperture
   * (a partially populated array, e.g. after dead-channel detection, renders
   * proportionally faster). */
  uint8_t active_index[UMH_DEVICE_CHANNEL_COUNT];
  uint16_t active_count;
  /* Distance -> phase conversion constant: distance_um * phase_scale_q10 is
   * the phase in 1/1024 carrier turn. */
  float phase_scale_q10;
  uint8_t geometry_valid;
  uint8_t reserved[3];
  uint8_t rgb_gain[UMH_DEVICE_RGB_COUNT][3];
  uint32_t carrier_hz;
  uint32_t sound_speed_um_per_s;
  uint32_t phase_resolution;
} umh_spatial_renderer_t;

/* Shared per-channel complex accumulation used by the producers that drive
 * channels one at a time (spiral/vortex emitters) rather than per point.
 * phase_q10 is the absolute phase in 1/1024 carrier turn, i.e. it already
 * contains the calibration phase offset.  amplitude is the source level before
 * the per-channel calibration gain is applied.  The accumulate is a fused
 * multiply-add so one channel costs one load, two table loads, two FMAs and
 * two stores. */
static inline void spatial_renderer_accumulate_q10(
    const umh_spatial_renderer_t *renderer, uint16_t index,
    int32_t phase_q10, float amplitude,
    float *real_accum, float *imag_accum)
{
  float mag = amplitude * renderer->gain_scale[index];
  real_accum[index] = fmaf(mag, umh_fast_cos_q10(phase_q10), real_accum[index]);
  imag_accum[index] = fmaf(mag, umh_fast_sin_q10(phase_q10), imag_accum[index]);
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
