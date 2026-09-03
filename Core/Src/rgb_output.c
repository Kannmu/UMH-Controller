#include "rgb_output.h"
#include <string.h>

void rgb_output_init(rgb_output_t *output)
{
  uint8_t i;
  if (output == NULL) return;
  memset(output, 0, sizeof(*output));
  for (i = 0u; i < UMH_DEVICE_RGB_COUNT; ++i) output->level[i] = 255u;
}

void rgb_output_set_color(rgb_output_t *output, uint8_t index, umh_rgb_value_t color)
{
  if (output == NULL || index >= UMH_DEVICE_RGB_COUNT) return;
  output->color[index] = color;
  output->dirty = 1u;
}

void rgb_output_set_level(rgb_output_t *output, uint8_t index, uint8_t level)
{
  if (output == NULL || index >= UMH_DEVICE_RGB_COUNT) return;
  output->level[index] = level;
  output->dirty = 1u;
}

void rgb_output_apply(rgb_output_t *output, umh_output_frame_t *frame)
{
  uint8_t i;
  if (output == NULL || frame == NULL) return;
  for (i = 0u; i < UMH_DEVICE_RGB_COUNT; ++i) {
    frame->rgb[i].red = (uint8_t)(((uint16_t)output->color[i].red * output->level[i]) / 255u);
    frame->rgb[i].green = (uint8_t)(((uint16_t)output->color[i].green * output->level[i]) / 255u);
    frame->rgb[i].blue = (uint8_t)(((uint16_t)output->color[i].blue * output->level[i]) / 255u);
  }
  frame->update_flags |= UMH_FRAME_FLAG_RGB;
  output->dirty = 0u;
}
