#include "spatiotemporal_block.h"
#include <string.h>

uint32_t umh_varuint_decode(const uint8_t *data, uint16_t length, uint32_t *value)
{
  uint32_t result = 0u;
  uint32_t shift = 0u;
  uint16_t i;
  if (data == NULL || value == NULL) return 0u;
  for (i = 0u; i < length && i < 5u; ++i) {
    /* The fifth byte has only four usable value bits in a uint32. */
    if (i == 4u && (data[i] & 0x7Fu) > 0x0Fu) return 0u;
    result |= ((uint32_t)(data[i] & 0x7Fu)) << shift;
    if ((data[i] & 0x80u) == 0u) {
      *value = result;
      return (uint32_t)i + 1u;
    }
    shift += 7u;
  }
  return 0u;
}

uint16_t umh_varuint_encode(uint32_t value, uint8_t *data, uint16_t capacity)
{
  uint16_t i = 0u;
  if (data == NULL) return 0u;
  do {
    if (i >= capacity) return 0u;
    data[i] = (uint8_t)(value & 0x7Fu);
    value >>= 7;
    if (value != 0u) data[i] |= 0x80u;
    ++i;
  } while (value != 0u);
  return i;
}

int spatiotemporal_block_begin(umh_block_context_t *block,
                               const uint8_t *payload, uint16_t length)
{
  uint16_t descriptor_bytes;
  uint16_t i;
  if (block == NULL || payload == NULL || length < sizeof(umh_block_wire_header_t)) return -1;
  memcpy(&block->header, payload, sizeof(block->header));
  if (block->header.timebase_hz != UMH_DEVICE_TIMEBASE_HZ ||
      block->header.track_count == 0u || block->header.track_count > UMH_BLOCK_MAX_TRACKS ||
      block->header.record_count > UMH_BLOCK_MAX_RECORDS ||
      block->header.start_time > UINT64_MAX - block->header.duration) return -2;
  if (block->header.output_period != 0u) return -12;
  descriptor_bytes = (uint16_t)(block->header.track_count * sizeof(umh_track_wire_descriptor_t));
  /* BLOCK_BEGIN is a complete descriptor transaction.  Accepting trailing
   * bytes would make a malformed host packet look valid and desynchronise the
   * block contract. */
  if (length != sizeof(umh_block_wire_header_t) + descriptor_bytes) return -3;
  memcpy(block->tracks, payload + sizeof(umh_block_wire_header_t), descriptor_bytes);
  for (i = 0u; i < block->header.track_count; ++i) {
    const umh_track_wire_descriptor_t *track = &block->tracks[i];
    uint16_t target_limit = UMH_DEVICE_CHANNEL_COUNT;
    uint16_t j;
    if (track->track_id >= UMH_BLOCK_MAX_TRACKS || track->target_mode > UMH_TARGET_SPARSE ||
        track->encoding < UMH_ENCODING_CONSTANT || track->encoding > UMH_ENCODING_VARIABLE ||
        track->interpolation > UMH_INTERPOLATE_LINEAR) return -4;
    /* The current compiler emits frames only at source record timestamps.  Do
     * not silently accept a request for interpolation or periodic synthesis. */
    if (track->interpolation != UMH_INTERPOLATE_HOLD) return -11;
    for (j = 0u; j < i; ++j)
      if (block->tracks[j].track_id == track->track_id) return -5;
    if (track->payload_type == UMH_PAYLOAD_COLOR_RGB8 ||
        track->payload_type == UMH_PAYLOAD_LIGHT_LEVEL8) target_limit = UMH_DEVICE_RGB_COUNT;
    if ((track->payload_type == UMH_PAYLOAD_SPATIAL_POINT ||
         track->payload_type == UMH_PAYLOAD_DIGITAL_STATE) &&
        track->target_mode != UMH_TARGET_ALL) return -10;
    if (track->target_mode == UMH_TARGET_RANGE) {
      if (track->target_count == 0u || track->target_start >= target_limit ||
          track->target_count > target_limit - track->target_start) return -6;
    } else if (track->target_mode == UMH_TARGET_BITMAP) {
      if (target_limit == UMH_DEVICE_CHANNEL_COUNT && track->target_count != 0u) return -7;
      if (target_limit == UMH_DEVICE_RGB_COUNT && track->target_count != 0u) return -7;
    } else if (track->target_mode == UMH_TARGET_SPARSE) {
      if (track->target_count == 0u || track->target_count > target_limit) return -8;
    } else if (track->target_count != 0u && track->target_count != target_limit) {
      return -9;
    }
  }
  block->track_count = block->header.track_count;
  block->records_received = 0u;
  block->current_time = block->header.start_time;
  block->active = 1u;
  return (int)(sizeof(umh_block_wire_header_t) + descriptor_bytes);
}

int spatiotemporal_block_append(umh_block_context_t *block,
                                const uint8_t *payload, uint16_t length)
{
  uint16_t pos = 0u;
  uint32_t delta;
  uint32_t consumed;
  uint16_t track_id;
  uint8_t flags;
  uint16_t payload_length;
  const umh_track_wire_descriptor_t *track;
  uint16_t fixed_length;
  uint32_t variable_length;
  uint32_t length_consumed;
  if (block == NULL || payload == NULL || block->active == 0u) return -1;
  while (pos < length) {
    consumed = umh_varuint_decode(&payload[pos], (uint16_t)(length - pos), &delta);
    if (consumed == 0u || (uint32_t)pos + consumed + sizeof(uint16_t) + 1u > length) return -2;
    pos = (uint16_t)(pos + consumed);
    track_id = (uint16_t)payload[pos] | ((uint16_t)payload[pos + 1u] << 8); pos += 2u;
    flags = payload[pos++];
    (void)flags;
    track = spatiotemporal_block_track(block, track_id);
    if (track == NULL) return -3;
    fixed_length = spatiotemporal_track_fixed_payload_size(track);
    if (fixed_length != 0u) payload_length = fixed_length;
    else {
      length_consumed = umh_varuint_decode(&payload[pos], (uint16_t)(length - pos),
                                           &variable_length);
      if (length_consumed == 0u || variable_length > UINT16_MAX) return -2;
      pos = (uint16_t)(pos + length_consumed);
      payload_length = (uint16_t)variable_length;
    }
    if ((uint32_t)pos + payload_length > length) return -3;
    if (delta > UINT64_MAX - block->current_time ||
        block->current_time + delta > block->header.start_time + block->header.duration) return -4;
    block->current_time += delta;
    pos = (uint16_t)(pos + payload_length);
    ++block->records_received;
    (void)flags;
    if (block->records_received > block->header.record_count) return -5;
  }
  return 0;
}

int spatiotemporal_block_end(umh_block_context_t *block)
{
  if (block == NULL || block->active == 0u) return -1;
  if (block->records_received != block->header.record_count) return -2;
  block->active = 0u;
  return 0;
}

const umh_track_wire_descriptor_t *spatiotemporal_block_track(
    const umh_block_context_t *block, uint16_t track_id)
{
  uint16_t i;
  if (block == NULL) return NULL;
  for (i = 0u; i < block->track_count; ++i) {
    if (block->tracks[i].track_id == track_id) return &block->tracks[i];
  }
  return NULL;
}

uint16_t spatiotemporal_track_fixed_payload_size(
    const umh_track_wire_descriptor_t *track)
{
  uint32_t count;
  if (track == NULL || track->encoding == UMH_ENCODING_VARIABLE ||
      track->target_mode == UMH_TARGET_SPARSE) return 0u;
  count = track->target_count;
  if (track->target_mode == UMH_TARGET_ALL)
    count = (track->payload_type == UMH_PAYLOAD_COLOR_RGB8 ||
             track->payload_type == UMH_PAYLOAD_LIGHT_LEVEL8) ?
            UMH_DEVICE_RGB_COUNT : UMH_DEVICE_CHANNEL_COUNT;
  if (track->payload_type == UMH_PAYLOAD_SPATIAL_POINT &&
      track->encoding == UMH_ENCODING_CONSTANT) return (uint16_t)sizeof(umh_spatial_point_t);
  if (track->payload_type == UMH_PAYLOAD_DIGITAL_STATE &&
      track->encoding == UMH_ENCODING_CONSTANT) return 2u;
  if (track->payload_type == UMH_PAYLOAD_CHANNEL_STATE) {
    if (track->encoding == UMH_ENCODING_CONSTANT)
      return (uint16_t)(2u + (track->target_mode == UMH_TARGET_BITMAP ?
                              UMH_DEVICE_CHANNEL_BITMAP_BYTES : 0u));
    if (track->encoding == UMH_ENCODING_DENSE &&
        track->target_mode != UMH_TARGET_BITMAP && count != 0u) {
      return (uint16_t)(count * 2u);
    }
  }
  if (track->payload_type == UMH_PAYLOAD_COLOR_RGB8) {
    if (track->encoding == UMH_ENCODING_CONSTANT)
      return (uint16_t)(3u + (track->target_mode == UMH_TARGET_BITMAP ?
                              UMH_BLOCK_RGB_BITMAP_BYTES : 0u));
    if (track->target_mode != UMH_TARGET_BITMAP && count != 0u) {
      if (track->encoding == UMH_ENCODING_DENSE) return (uint16_t)(count * 3u);
    }
  }
  if (track->payload_type == UMH_PAYLOAD_LIGHT_LEVEL8) {
    if (track->encoding == UMH_ENCODING_CONSTANT)
      return (uint16_t)(1u + (track->target_mode == UMH_TARGET_BITMAP ?
                              UMH_BLOCK_RGB_BITMAP_BYTES : 0u));
    if (track->target_mode != UMH_TARGET_BITMAP && count != 0u) {
      if (track->encoding == UMH_ENCODING_DENSE) return (uint16_t)count;
    }
  }
  return 0u;
}
