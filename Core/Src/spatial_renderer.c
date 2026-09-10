#include "spatial_renderer.h"
#include "cordic.h"
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
  const float two_pi = 6.28318530717958647692f;
  float wavelength;
  float phase_scale;
  float source_level;
  if (renderer == NULL || renderer->profile == NULL || point == NULL ||
      real_accum == NULL || imag_accum == NULL || renderer->carrier_hz == 0u) return -1;
  if ((renderer->profile->capability_flags & UMH_PROFILE_CAP_GEOMETRY_VALID) == 0u) return -2;
  wavelength = (float)renderer->sound_speed_um_per_s / (float)renderer->carrier_hz;
  phase_scale = (float)renderer->phase_resolution / wavelength;
  source_level = (float)point->level / 255.0f;
  for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) {
    float dx = (float)point->x_um - (float)renderer->profile->coordinates[i].x_um;
    float dy = (float)point->y_um - (float)renderer->profile->coordinates[i].y_um;
    float dz = (float)point->z_um - (float)renderer->profile->coordinates[i].z_um;
    float distance = sqrtf(dx * dx + dy * dy + dz * dz);
    float phase = ((float)point->phase - distance * phase_scale + (float)renderer->calibration[i].phase) * two_pi / (float)renderer->phase_resolution;
    float amplitude = source_level * ((float)renderer->calibration[i].gain / 255.0f);
    real_accum[i] += amplitude * cosf(phase);
    imag_accum[i] += amplitude * sinf(phase);
  }
  return 0;
}

int spatial_renderer_finalize(const umh_spatial_renderer_t *renderer,
                              const float *real_accum, const float *imag_accum,
                              umh_output_frame_t *frame)
{
  uint16_t i;
  const float two_pi = 6.28318530717958647692f;
  if (renderer == NULL || real_accum == NULL || imag_accum == NULL || frame == NULL) return -1;
  for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) {
    float magnitude = sqrtf(real_accum[i] * real_accum[i] + imag_accum[i] * imag_accum[i]);
    float angle;
    int32_t phase;
    if (magnitude > 1.0f) magnitude = 1.0f;
    if (umh_cordic_phase8(real_accum[i], imag_accum[i], &frame->channels[i].phase) != 0) {
      angle = atan2f(imag_accum[i], real_accum[i]);
      phase = (int32_t)(angle * (float)renderer->phase_resolution / two_pi);
      phase %= (int32_t)renderer->phase_resolution;
      if (phase < 0) phase += (int32_t)renderer->phase_resolution;
      frame->channels[i].phase = (uint8_t)phase;
    }
    frame->channels[i].level = (uint8_t)(magnitude * 255.0f + 0.5f);
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
