#ifndef SPATIOTEMPORAL_BLOCK_H
#define SPATIOTEMPORAL_BLOCK_H

#include <stdint.h>
#include <stddef.h>
#include "device_profile.h"

#define UMH_BLOCK_MAX_TRACKS 16u
#define UMH_BLOCK_MAX_SOURCES 8u
#define UMH_BLOCK_MAX_RECORDS 512u
#define UMH_BLOCK_TARGET_BITMAP_BYTES UMH_DEVICE_CHANNEL_BITMAP_BYTES
#define UMH_BLOCK_RGB_BITMAP_BYTES 1u

typedef enum {
  UMH_PAYLOAD_CHANNEL_STATE = 1u,
  UMH_PAYLOAD_SPATIAL_POINT = 2u,
  UMH_PAYLOAD_COLOR_RGB8 = 3u,
  UMH_PAYLOAD_LIGHT_LEVEL8 = 4u,
  UMH_PAYLOAD_DIGITAL_STATE = 5u,
  UMH_PAYLOAD_SCALAR = 6u,
  UMH_PAYLOAD_VECTOR = 7u,
  UMH_PAYLOAD_RAW = 8u
} umh_payload_type_t;

typedef enum {
  UMH_VALUE_U8 = 1u,
  UMH_VALUE_S16 = 2u,
  UMH_VALUE_U16 = 3u,
  UMH_VALUE_S32 = 4u,
  UMH_VALUE_U32 = 5u,
  UMH_VALUE_FIXED16 = 6u,
  UMH_VALUE_FIXED32 = 7u,
  UMH_VALUE_BYTES = 8u
} umh_value_type_t;

typedef enum {
  UMH_ENCODING_CONSTANT = 1u,
  UMH_ENCODING_DENSE = 2u,
  UMH_ENCODING_SPARSE = 3u,
  UMH_ENCODING_DELTA = 4u,
  UMH_ENCODING_ZERO_SUPPRESS = 5u,
  UMH_ENCODING_VARIABLE = 6u
} umh_encoding_t;

typedef enum {
  UMH_INTERPOLATE_HOLD = 0u,
  UMH_INTERPOLATE_STEP = 1u,
  UMH_INTERPOLATE_LINEAR = 2u
} umh_interpolation_t;

typedef enum {
  UMH_TARGET_ALL = 0u,
  UMH_TARGET_RANGE = 1u,
  UMH_TARGET_BITMAP = 2u,
  UMH_TARGET_SPARSE = 3u
} umh_target_mode_t;

/* Record flags are per-record transport hints; timing is always delta encoded.
 * KEYFRAME asks the consumer to treat the payload as a complete state for its
 * selected targets. END_OF_FRAME is informational and may be used by a host
 * encoder to mark the last record at a timestamp. Other bits are reserved. */
typedef enum {
  UMH_RECORD_FLAG_KEYFRAME = 1u << 0,
  UMH_RECORD_FLAG_END_OF_FRAME = 1u << 1
} umh_record_flag_t;

typedef enum {
  UMH_TRACK_COMPONENT_PHASE = 1u << 0,
  UMH_TRACK_COMPONENT_LEVEL = 1u << 1,
  UMH_TRACK_COMPONENT_ENABLE = 1u << 2,
  UMH_TRACK_COMPONENT_XYZ = 1u << 3,
  UMH_TRACK_COMPONENT_SOURCE_LEVEL = 1u << 4,
  UMH_TRACK_COMPONENT_SOURCE_PHASE = 1u << 5,
  UMH_TRACK_COMPONENT_SOURCE_ID = 1u << 6,
  UMH_TRACK_COMPONENT_R = 1u << 7,
  UMH_TRACK_COMPONENT_G = 1u << 8,
  UMH_TRACK_COMPONENT_B = 1u << 9
} umh_track_component_t;

typedef struct __attribute__((packed)) {
  uint32_t block_id;
  uint32_t timebase_hz;
  uint64_t start_time;
  uint64_t duration;
  uint32_t record_count;
  uint32_t output_period;
  uint16_t track_count;
  uint16_t flags;
} umh_block_wire_header_t;

typedef struct __attribute__((packed)) {
  uint16_t track_id;
  uint8_t payload_type;
  uint8_t value_type;
  uint8_t encoding;
  uint8_t interpolation;
  uint16_t component_mask;
  uint8_t target_mode;
  uint8_t unit;
  uint16_t target_count;
  uint8_t quantization_bits;
  uint8_t attributes;
  uint16_t target_start;
  uint16_t payload_stride;
  uint32_t reserved;
} umh_track_wire_descriptor_t;

typedef struct __attribute__((packed)) {
  uint16_t track_id;
  uint8_t record_flags;
} umh_record_wire_prefix_t;

typedef struct __attribute__((packed)) {
  int32_t x_um;
  int32_t y_um;
  int32_t z_um;
  uint8_t level;
  uint16_t phase;
  uint8_t source_id;
} umh_spatial_point_t;

typedef struct __attribute__((packed)) {
  uint16_t phase;
  uint8_t level;
  uint8_t enabled;
} umh_channel_value_t;

typedef struct {
  umh_block_wire_header_t header;
  umh_track_wire_descriptor_t tracks[UMH_BLOCK_MAX_TRACKS];
  uint16_t track_count;
  uint32_t records_received;
  uint64_t current_time;
  uint8_t active;
} umh_block_context_t;

_Static_assert(sizeof(umh_block_wire_header_t) == 36u, "block header wire size");
_Static_assert(sizeof(umh_track_wire_descriptor_t) == 22u, "track descriptor wire size");
_Static_assert(sizeof(umh_record_wire_prefix_t) == 3u, "record prefix wire size");
_Static_assert(sizeof(umh_spatial_point_t) == 16u, "spatial point wire size");

int spatiotemporal_block_begin(umh_block_context_t *block,
                               const uint8_t *payload, uint16_t length);
int spatiotemporal_block_append(umh_block_context_t *block,
                                const uint8_t *payload, uint16_t length);
int spatiotemporal_block_end(umh_block_context_t *block);
const umh_track_wire_descriptor_t *spatiotemporal_block_track(
    const umh_block_context_t *block, uint16_t track_id);
/* Returns a non-zero fixed payload size. A zero result means the record must
 * carry a varuint payload length after record_flags. */
uint16_t spatiotemporal_track_fixed_payload_size(
    const umh_track_wire_descriptor_t *track);
uint32_t umh_varuint_decode(const uint8_t *data, uint16_t length, uint32_t *value);
uint16_t umh_varuint_encode(uint32_t value, uint8_t *data, uint16_t capacity);

#endif
