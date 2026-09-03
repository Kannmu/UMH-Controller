#ifndef BLOCK_PARSER_H
#define BLOCK_PARSER_H

#include <stdint.h>
#include "spatiotemporal_block.h"
#include "spatial_renderer.h"
#include "frame_ring.h"

#define UMH_BLOCK_PARSER_INPUT_SIZE 512u
#define UMH_BLOCK_PARSER_MAX_RECORD_BYTES 512u

typedef struct {
  umh_block_context_t block;
  umh_spatial_renderer_t *renderer;
  umh_frame_ring_t *frames;
  uint8_t input[UMH_BLOCK_PARSER_INPUT_SIZE];
  uint16_t input_length;
  uint8_t record[UMH_BLOCK_PARSER_MAX_RECORD_BYTES];
  uint16_t record_length;
  uint16_t record_expected;
  uint8_t record_active;
  uint64_t pending_time;
  umh_output_frame_t pending_frame;
  umh_output_frame_t current_state;
  float spatial_real[UMH_DEVICE_CHANNEL_COUNT];
  float spatial_imag[UMH_DEVICE_CHANNEL_COUNT];
  umh_rgb_value_t rgb_source[UMH_DEVICE_RGB_COUNT];
  uint8_t rgb_level[UMH_DEVICE_RGB_COUNT];
  uint8_t spatial_accum_valid;
  uint8_t state_initialized;
  uint8_t pending_valid;
  uint32_t next_frame_sequence;
  uint32_t parse_errors;
} umh_block_parser_t;

void block_parser_init(umh_block_parser_t *parser,
                       umh_spatial_renderer_t *renderer,
                       umh_frame_ring_t *frames);
int block_parser_begin(umh_block_parser_t *parser, const uint8_t *payload, uint16_t length);
int block_parser_data(umh_block_parser_t *parser, const uint8_t *payload, uint16_t length);
int block_parser_end(umh_block_parser_t *parser);
void block_parser_cancel(umh_block_parser_t *parser);
uint64_t block_parser_duration(const umh_block_parser_t *parser);

#endif
