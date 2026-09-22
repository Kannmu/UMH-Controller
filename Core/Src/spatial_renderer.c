#include "spatial_renderer.h"
#include "cordic.h"
#include "umh_fast_math.h"
#include <math.h>
#include <string.h>
#include <float.h>

void spatial_renderer_init(umh_spatial_renderer_t *renderer,
                           const umh_device_profile_t *profile)
{
  uint16_t i;
  if (renderer == NULL) return;
  memset(renderer, 0, sizeof(*renderer));
  renderer->profile = profile;
  renderer->carrier_hz = profile != NULL ? profile->carrier_hz : 40000u;
  renderer->sound_speed_um_per_s = profile != NULL && profile->sound_speed_um_per_s != 0u ?
                                   profile->sound_speed_um_per_s : 343000000u;
  renderer->phase_resolution = 256u;
  for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) {
    renderer->calibration[i].gain = 255u;
    renderer->calibration[i].enabled = 1u;
    renderer->gain_scale[i] = 1.0f;
    renderer->phase_offset_q10[i] = 0;
  }
  for (i = 0u; i < UMH_DEVICE_RGB_COUNT; ++i) {
    renderer->rgb_gain[i][0] = 255u;
    renderer->rgb_gain[i][1] = 255u;
    renderer->rgb_gain[i][2] = 255u;
  }
}

void spatial_renderer_set_calibration(umh_spatial_renderer_t *renderer,
                                      const umh_channel_calibration_t *calibration,
                                      uint16_t count)
{
  if (renderer == NULL || calibration == NULL) return;
  if (count > UMH_DEVICE_CHANNEL_COUNT) count = UMH_DEVICE_CHANNEL_COUNT;
  memcpy(renderer->calibration, calibration, count * sizeof(calibration[0]));
  spatial_renderer_refresh_calibration(renderer, count);
}

void spatial_renderer_refresh_calibration(umh_spatial_renderer_t *renderer,
                                          uint16_t count)
{
  uint16_t i;
  if (renderer == NULL) return;
  if (count > UMH_DEVICE_CHANNEL_COUNT) count = UMH_DEVICE_CHANNEL_COUNT;
  for (i = 0u; i < count; ++i) {
    renderer->gain_scale[i] = (float)renderer->calibration[i].gain * (1.0f / 255.0f);
    /* source/calibration phases are 1/256 carrier turn, the LUT is 1/1024 */
    renderer->phase_offset_q10[i] = (int32_t)renderer->calibration[i].phase * 4;
  }
}

int spatial_renderer_point(umh_spatial_renderer_t *renderer,
                           const umh_spatial_point_t *point,
                           umh_output_frame_t *frame)
{
  float real_accum[UMH_DEVICE_CHANNEL_COUNT] = {0.0f};
  float imag_accum[UMH_DEVICE_CHANNEL_COUNT] = {0.0f};
  if (renderer == NULL || point == NULL || frame == NULL) return -1;
  if (spatial_renderer_accumulate_point(renderer, point, real_accum, imag_accum) != 0) return -1;
  if (spatial_renderer_finalize(renderer, real_accum, imag_accum, frame) != 0) return -1;
  frame->update_flags |= UMH_FRAME_FLAG_ULTRASOUND;
  return 0;
}

int spatial_renderer_accumulate_point(const umh_spatial_renderer_t *renderer,
                                      const umh_spatial_point_t *point,
                                      float *real_accum, float *imag_accum)
{
  uint16_t i;
  float wavelength;
  float phase_scale_q10;
  float source_level;
  if (renderer == NULL || renderer->profile == NULL || point == NULL ||
      real_accum == NULL || imag_accum == NULL || renderer->carrier_hz == 0u) return -1;
  if ((renderer->profile->capability_flags & UMH_PROFILE_CAP_GEOMETRY_VALID) == 0u) return -2;
  if (point->level == 0u) return 0;
  wavelength = (float)renderer->sound_speed_um_per_s / (float)renderer->carrier_hz;
  /* One carrier turn spans 1024 table entries, and source/calibration phase
   * bytes are 1/256 turn, so byte phases scale by four. */
  phase_scale_q10 = 1024.0f / wavelength;
  source_level = (float)point->level / 255.0f;
  {
    float source_phase_q10 = (float)point->phase * 4.0f;
    for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) {
      float dx;
      float dy;
      float dz;
      float distance;
      float amplitude;
      int32_t phase_q10;
      if (renderer->calibration[i].enabled == 0u) continue;
      dx = (float)point->x_um - (float)renderer->profile->coordinates[i].x_um;
      dy = (float)point->y_um - (float)renderer->profile->coordinates[i].y_um;
      dz = (float)point->z_um - (float)renderer->profile->coordinates[i].z_um;
      distance = sqrtf(dx * dx + dy * dy + dz * dz);
      phase_q10 = (int32_t)(source_phase_q10 +
                            (float)renderer->phase_offset_q10[i] -
                            distance * phase_scale_q10);
      amplitude = source_level * renderer->gain_scale[i];
      real_accum[i] += amplitude * umh_fast_cos_q10(phase_q10);
      imag_accum[i] += amplitude * umh_fast_sin_q10(phase_q10);
    }
  }
  return 0;
}

int spatial_renderer_finalize(const umh_spatial_renderer_t *renderer,
                              const float *real_accum, const float *imag_accum,
                              umh_output_frame_t *frame)
{
  uint16_t i;
  uint8_t phase_codes[UMH_DEVICE_CHANNEL_COUNT];
  uint8_t have_codes;
  const float two_pi = 6.28318530717958647692f;
  if (renderer == NULL || real_accum == NULL || imag_accum == NULL || frame == NULL) return -1;
  /* One CORDIC configuration serves all 84 channels instead of one HAL
   * configure/calculate round trip per channel.  The calibration path already
   * proves this unit on silicon; if it is unavailable the scalar fallback
   * below keeps the renderer correct, just slower. */
  have_codes = umh_cordic_phase8_batch(real_accum, imag_accum, phase_codes,
                                       UMH_DEVICE_CHANNEL_COUNT) == 0 ? 1u : 0u;
  for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) {
    float magnitude = sqrtf(real_accum[i] * real_accum[i] +
                            imag_accum[i] * imag_accum[i]);
    if (magnitude > 1.0f) magnitude = 1.0f;
    if (have_codes != 0u) {
      frame->channels[i].phase = phase_codes[i];
    } else {
      float angle = atan2f(imag_accum[i], real_accum[i]);
      int32_t phase = (int32_t)(angle * (float)renderer->phase_resolution / two_pi);
      phase %= (int32_t)renderer->phase_resolution;
      if (phase < 0) phase += (int32_t)renderer->phase_resolution;
      frame->channels[i].phase = (uint8_t)phase;
    }
    /* The FPGA interprets level as the high-time in a 256-slot 40 kHz
     * carrier.  Full spatial amplitude therefore maps to a 50% duty
     * (level=128).  Using 255 here would be 99.6% DC and would leave no
     * 40 kHz component for the transducer to launch. */
    frame->channels[i].level = (uint8_t)(magnitude * 128.0f + 0.5f);
    if (renderer->calibration[i].enabled == 0u) frame->channels[i].level = 0u;
  }
  frame->update_flags |= UMH_FRAME_FLAG_ULTRASOUND;
  return 0;
}

void spatial_renderer_set_rgb_calibration(umh_spatial_renderer_t *renderer,
                                          const uint8_t gain[UMH_DEVICE_RGB_COUNT][3])
{
  if (renderer == NULL || gain == NULL) return;
  memcpy(renderer->rgb_gain, gain, sizeof(renderer->rgb_gain));
}

void spatial_renderer_merge_rgb(const umh_spatial_renderer_t *renderer,
                                umh_output_frame_t *frame,
                                const umh_rgb_value_t *color,
                                const uint8_t *levels,
                                uint8_t target_mask)
{
  uint8_t i;
  if (frame == NULL || color == NULL) return;
  for (i = 0u; i < UMH_DEVICE_RGB_COUNT; ++i) {
    if ((target_mask & (uint8_t)(1u << i)) == 0u) continue;
    uint8_t level = levels != NULL ? levels[i] : 255u;
    uint8_t gain_r = renderer != NULL ? renderer->rgb_gain[i][0] : 255u;
    uint8_t gain_g = renderer != NULL ? renderer->rgb_gain[i][1] : 255u;
    uint8_t gain_b = renderer != NULL ? renderer->rgb_gain[i][2] : 255u;
    frame->rgb[i].red = (uint8_t)(((uint32_t)color[i].red * level * gain_r) / (255u * 255u));
    frame->rgb[i].green = (uint8_t)(((uint32_t)color[i].green * level * gain_g) / (255u * 255u));
    frame->rgb[i].blue = (uint8_t)(((uint32_t)color[i].blue * level * gain_b) / (255u * 255u));
  }
  frame->update_flags |= UMH_FRAME_FLAG_RGB;
}
