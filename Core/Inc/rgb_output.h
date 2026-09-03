#ifndef RGB_OUTPUT_H
#define RGB_OUTPUT_H

#include <stdint.h>
#include "frame_ring.h"

typedef struct {
  umh_rgb_value_t color[UMH_DEVICE_RGB_COUNT];
  uint8_t level[UMH_DEVICE_RGB_COUNT];
  uint8_t dirty;
} rgb_output_t;

void rgb_output_init(rgb_output_t *output);
void rgb_output_set_color(rgb_output_t *output, uint8_t index, umh_rgb_value_t color);
void rgb_output_set_level(rgb_output_t *output, uint8_t index, uint8_t level);
void rgb_output_apply(rgb_output_t *output, umh_output_frame_t *frame);

#endif
