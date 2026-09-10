#include "block_parser.h"
#include <string.h>

static void frame_start(umh_block_parser_t *parser, uint64_t deadline)
{
  if (parser->state_initialized != 0u) parser->pending_frame = parser->current_state;
  else memset(&parser->pending_frame, 0, sizeof(parser->pending_frame));
  parser->pending_frame.sequence = parser->next_frame_sequence++;
  parser->pending_frame.deadline = deadline;
  parser->pending_frame.update_flags = 0u;
  parser->pending_frame.reserved = 0u;
  parser->pending_frame.extension_length = 0u;
  parser->pending_time = deadline;
  parser->pending_valid = 1u;
  parser->spatial_accum_valid = 0u;
  memset(parser->spatial_real, 0, sizeof(parser->spatial_real));
  memset(parser->spatial_imag, 0, sizeof(parser->spatial_imag));
}

static int finalize_spatial(umh_block_parser_t *parser)
{
  if (parser == NULL || parser->spatial_accum_valid == 0u) return 0;
  if (spatial_renderer_finalize(parser->renderer, parser->spatial_real,
                                parser->spatial_imag, &parser->pending_frame) != 0) return -1;
  parser->spatial_accum_valid = 0u;
  return 0;
}

static int commit_pending(umh_block_parser_t *parser)
{
  umh_output_frame_t *slot;
  if (parser->pending_valid == 0u) return 0;
  if (finalize_spatial(parser) != 0) return -1;
  slot = frame_ring_acquire_write(parser->frames);
  if (slot == NULL) return -1;
  memcpy(slot, &parser->pending_frame, sizeof(*slot));
  parser->current_state = parser->pending_frame;
  parser->current_state.update_flags = 0u;
  parser->state_initialized = 1u;
  parser->pending_valid = 0u;
  return frame_ring_commit_write(parser->frames);
}

static int apply_channel_state(const umh_track_wire_descriptor_t *track,
                               const uint8_t *payload, uint16_t length,
                               umh_output_frame_t *frame)
{
  uint16_t i;
  uint16_t target_start = 0u;
  uint16_t target_count = UMH_DEVICE_CHANNEL_COUNT;
  uint16_t payload_offset = 0u;
  uint16_t components;
  if (track == NULL || payload == NULL || frame == NULL) return -1;
  /* The wire value is always phase8, level8.  A zero level disables a channel;
   * the legacy enable component is accepted as a descriptor hint only. */
  components = track->component_mask != 0u ? track->component_mask :
               (UMH_TRACK_COMPONENT_PHASE | UMH_TRACK_COMPONENT_LEVEL);
  if (track->target_mode == UMH_TARGET_RANGE) {
    target_start = track->target_start;
    target_count = track->target_count;
    if (target_count == 0u || target_start >= UMH_DEVICE_CHANNEL_COUNT ||
        target_count > UMH_DEVICE_CHANNEL_COUNT - target_start) return -2;
  } else if (track->target_mode == UMH_TARGET_BITMAP) {
    if (length < UMH_DEVICE_CHANNEL_BITMAP_BYTES) return -2;
    payload_offset = UMH_DEVICE_CHANNEL_BITMAP_BYTES;
    target_count = 0u;
    for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i)
      if ((payload[i / 8u] & (uint8_t)(1u << (i % 8u))) != 0u) ++target_count;
  } else if (track->target_mode == UMH_TARGET_SPARSE) {
    target_count = track->target_count;
    if (target_count == 0u || target_count > UMH_DEVICE_CHANNEL_COUNT) return -2;
  } else if (track->target_mode != UMH_TARGET_ALL) return -2;
  if (track->encoding == UMH_ENCODING_DENSE) {
    if (track->target_mode == UMH_TARGET_SPARSE || length < payload_offset + target_count * 2u) return -3;
    for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) {
      uint8_t selected = track->target_mode == UMH_TARGET_BITMAP ?
                         (uint8_t)((payload[i / 8u] >> (i % 8u)) & 1u) :
                         (uint8_t)(i >= target_start && i < target_start + target_count);
      uint16_t source_index = 0u;
      if (selected == 0u) continue;
      if (track->target_mode == UMH_TARGET_BITMAP) {
        for (uint16_t j = 0u; j < i; ++j)
          if ((payload[j / 8u] & (uint8_t)(1u << (j % 8u))) != 0u) ++source_index;
      } else {
        source_index = (uint16_t)(i - target_start);
      }
      source_index = (uint16_t)(payload_offset + source_index * 2u);
      if ((components & UMH_TRACK_COMPONENT_PHASE) != 0u) frame->channels[i].phase = payload[source_index];
      if ((components & UMH_TRACK_COMPONENT_LEVEL) != 0u) frame->channels[i].level = payload[source_index + 1u];
    }
  } else if (track->encoding == UMH_ENCODING_SPARSE ||
             track->encoding == UMH_ENCODING_DELTA ||
             track->encoding == UMH_ENCODING_ZERO_SUPPRESS ||
             (track->encoding == UMH_ENCODING_CONSTANT && track->target_mode == UMH_TARGET_SPARSE)) {
    uint16_t pos = payload_offset;
    uint8_t seen[UMH_DEVICE_CHANNEL_COUNT] = {0u};
    uint16_t seen_count = 0u;
    while (pos + 4u <= length) {
      uint16_t channel = (uint16_t)payload[pos] | ((uint16_t)payload[pos + 1u] << 8);
      if (channel >= UMH_DEVICE_CHANNEL_COUNT || seen[channel] != 0u) return -3;
      seen[channel] = 1u;
      ++seen_count;
      if (track->target_mode == UMH_TARGET_RANGE && (channel < target_start || channel >= target_start + target_count)) return -3;
      if (track->target_mode == UMH_TARGET_BITMAP && (payload[channel / 8u] & (uint8_t)(1u << (channel % 8u))) == 0u) return -3;
      if (track->encoding == UMH_ENCODING_DELTA) {
        int16_t level = (int16_t)frame->channels[channel].level + (int8_t)payload[pos + 3u];
        if ((components & UMH_TRACK_COMPONENT_PHASE) != 0u) frame->channels[channel].phase = (uint8_t)(frame->channels[channel].phase + (int8_t)payload[pos + 2u]);
        if ((components & UMH_TRACK_COMPONENT_LEVEL) != 0u) frame->channels[channel].level = (uint8_t)(level < 0 ? 0 : level > 255 ? 255 : level);
      } else {
        if ((components & UMH_TRACK_COMPONENT_PHASE) != 0u) frame->channels[channel].phase = payload[pos + 2u];
        if ((components & UMH_TRACK_COMPONENT_LEVEL) != 0u) frame->channels[channel].level = payload[pos + 3u];
      }
      pos = (uint16_t)(pos + 4u);
    }
    if (pos != length || seen_count == 0u ||
        (track->target_mode == UMH_TARGET_SPARSE && seen_count != target_count) ||
        (track->target_mode == UMH_TARGET_BITMAP && seen_count > target_count)) return -3;
  } else if (track->encoding == UMH_ENCODING_CONSTANT) {
    if (length != payload_offset + 2u) return -4;
    for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) {
      if (track->target_mode == UMH_TARGET_BITMAP && (payload[i / 8u] & (uint8_t)(1u << (i % 8u))) == 0u) continue;
      if (track->target_mode == UMH_TARGET_RANGE && (i < target_start || i >= target_start + target_count)) continue;
      if ((components & UMH_TRACK_COMPONENT_PHASE) != 0u) frame->channels[i].phase = payload[payload_offset];
      if ((components & UMH_TRACK_COMPONENT_LEVEL) != 0u) frame->channels[i].level = payload[payload_offset + 1u];
    }
  } else return -5;
  frame->update_flags |= UMH_FRAME_FLAG_ULTRASOUND;
  return 0;
}

static uint16_t rgb_targets(const umh_track_wire_descriptor_t *track,
                            const uint8_t *payload, uint16_t length,
                            uint8_t indices[UMH_DEVICE_RGB_COUNT],
                            uint16_t *payload_offset)
{
  uint16_t i;
  uint16_t count = 0u;
  if (track == NULL || payload == NULL || indices == NULL || payload_offset == NULL) return 0u;
  *payload_offset = 0u;
  switch (track->target_mode) {
    case UMH_TARGET_ALL:
      count = track->target_count == 0u ? UMH_DEVICE_RGB_COUNT : track->target_count;
      if (count > UMH_DEVICE_RGB_COUNT) return 0u;
      for (i = 0u; i < count; ++i) indices[i] = (uint8_t)i;
      break;
    case UMH_TARGET_RANGE:
      if (track->target_start >= UMH_DEVICE_RGB_COUNT || track->target_count == 0u ||
          track->target_count > UMH_DEVICE_RGB_COUNT - track->target_start) return 0u;
      count = track->target_count;
      for (i = 0u; i < count; ++i) indices[i] = (uint8_t)(track->target_start + i);
      break;
    case UMH_TARGET_BITMAP:
      if (length < UMH_BLOCK_RGB_BITMAP_BYTES) return 0u;
      *payload_offset = UMH_BLOCK_RGB_BITMAP_BYTES;
      for (i = 0u; i < UMH_DEVICE_RGB_COUNT; ++i)
        if ((payload[0] & (uint8_t)(1u << i)) != 0u) indices[count++] = (uint8_t)i;
      break;
    case UMH_TARGET_SPARSE:
      count = track->target_count;
      if (count == 0u || count > UMH_DEVICE_RGB_COUNT) return 0u;
      break;
    default:
      return 0u;
  }
  return count;
}

static uint8_t rgb_seen_contains(const uint8_t *indices, uint16_t count, uint8_t index)
{
  uint16_t i;
  for (i = 0u; i < count; ++i) if (indices[i] == index) return 1u;
  return 0u;
}

static uint8_t rgb_add_delta(uint8_t value, int8_t delta)
{
  int16_t result = (int16_t)value + (int16_t)delta;
  if (result < 0) return 0u;
  if (result > 255) return 255u;
  return (uint8_t)result;
}

static int apply_record(umh_block_parser_t *parser, uint32_t delta,
                        uint16_t track_id, uint8_t flags,
                        const uint8_t *payload, uint16_t length)
{
  const umh_track_wire_descriptor_t *track = spatiotemporal_block_track(&parser->block, track_id);
  umh_spatial_point_t point;
  uint64_t timestamp;
  if (track == NULL) return -1;
  if ((flags & (uint8_t)~(UMH_RECORD_FLAG_KEYFRAME | UMH_RECORD_FLAG_END_OF_FRAME)) != 0u) return -2;
  if (delta > UINT64_MAX - parser->block.current_time ||
      parser->block.current_time + delta >
      parser->block.header.start_time + parser->block.header.duration) return -2;
  timestamp = parser->block.current_time + delta;
  parser->block.current_time = timestamp;
  if (parser->pending_valid == 0u) {
    frame_start(parser, timestamp);
  } else if (timestamp != parser->pending_time) {
    if (commit_pending(parser) != 0) return -2;
    frame_start(parser, timestamp);
  }
  switch (track->payload_type) {
    case UMH_PAYLOAD_CHANNEL_STATE:
      return apply_channel_state(track, payload, length, &parser->pending_frame);
    case UMH_PAYLOAD_SPATIAL_POINT:
      if (length < sizeof(point)) return -3;
      memcpy(&point, payload, sizeof(point));
      if (spatial_renderer_accumulate_point(parser->renderer, &point,
                                             parser->spatial_real,
                                             parser->spatial_imag) != 0) return -3;
      parser->spatial_accum_valid = 1u;
      parser->pending_frame.update_flags |= UMH_FRAME_FLAG_ULTRASOUND;
      return 0;
    case UMH_PAYLOAD_COLOR_RGB8:
      {
        uint8_t indices[UMH_DEVICE_RGB_COUNT];
        uint16_t offset;
        uint16_t count = rgb_targets(track, payload, length, indices, &offset);
        uint16_t k;
        if (count == 0u) return -4;
      if (track->encoding == UMH_ENCODING_CONSTANT && track->target_mode != UMH_TARGET_SPARSE) {
          if (length != offset + 3u) return -4;
          for (k = 0u; k < count; ++k) parser->rgb_source[indices[k]] = (umh_rgb_value_t){payload[offset], payload[offset + 1u], payload[offset + 2u]};
        } else if (track->encoding == UMH_ENCODING_DENSE) {
          if (length != offset + count * 3u) return -4;
          for (k = 0u; k < count; ++k) parser->rgb_source[indices[k]] = (umh_rgb_value_t){payload[offset + k * 3u], payload[offset + k * 3u + 1u], payload[offset + k * 3u + 2u]};
        } else if (track->encoding == UMH_ENCODING_SPARSE ||
                   track->encoding == UMH_ENCODING_DELTA ||
                   track->encoding == UMH_ENCODING_ZERO_SUPPRESS ||
                   (track->encoding == UMH_ENCODING_CONSTANT && track->target_mode == UMH_TARGET_SPARSE)) {
          uint16_t pos = offset;
          uint16_t seen = 0u;
          uint8_t sparse_indices[UMH_DEVICE_RGB_COUNT];
          while (pos + 4u <= length) {
            uint8_t index = payload[pos];
            if (index >= UMH_DEVICE_RGB_COUNT) return -4;
            if (seen >= UMH_DEVICE_RGB_COUNT) return -4;
            if (rgb_seen_contains(sparse_indices, seen, index)) return -4;
            if (track->target_mode == UMH_TARGET_RANGE &&
                (index < track->target_start || index >= track->target_start + track->target_count)) return -4;
            if (track->target_mode == UMH_TARGET_BITMAP &&
                (payload[0] & (uint8_t)(1u << index)) == 0u) return -4;
            sparse_indices[seen] = index;
            if (track->encoding == UMH_ENCODING_DELTA) {
              parser->rgb_source[index].red = rgb_add_delta(parser->rgb_source[index].red,
                                                             (int8_t)payload[pos + 1u]);
              parser->rgb_source[index].green = rgb_add_delta(parser->rgb_source[index].green,
                                                               (int8_t)payload[pos + 2u]);
              parser->rgb_source[index].blue = rgb_add_delta(parser->rgb_source[index].blue,
                                                              (int8_t)payload[pos + 3u]);
            } else {
              parser->rgb_source[index] = (umh_rgb_value_t){payload[pos + 1u], payload[pos + 2u], payload[pos + 3u]};
            }
            pos = (uint16_t)(pos + 4u); ++seen;
          }
          if (pos != length || seen == 0u ||
              (track->target_mode == UMH_TARGET_SPARSE && seen != count) ||
              (track->target_mode != UMH_TARGET_SPARSE && seen > count)) return -4;
          memcpy(indices, sparse_indices, seen);
          count = seen;
        } else return -4;
        for (k = 0u; k < count; ++k)
          spatial_renderer_merge_rgb(parser->renderer, &parser->pending_frame,
                                     parser->rgb_source, parser->rgb_level,
                                     (uint8_t)(1u << indices[k]));
        return 0;
      }
    case UMH_PAYLOAD_LIGHT_LEVEL8:
      {
        uint8_t indices[UMH_DEVICE_RGB_COUNT];
        uint16_t offset;
        uint16_t count = rgb_targets(track, payload, length, indices, &offset);
        uint16_t k;
        if (count == 0u) return -4;
        if (track->encoding == UMH_ENCODING_CONSTANT && track->target_mode != UMH_TARGET_SPARSE) {
          if (length != offset + 1u) return -4;
          for (k = 0u; k < count; ++k) parser->rgb_level[indices[k]] = payload[offset];
        } else if (track->encoding == UMH_ENCODING_DENSE) {
          if (length != offset + count) return -4;
          for (k = 0u; k < count; ++k) parser->rgb_level[indices[k]] = payload[offset + k];
        } else if (track->encoding == UMH_ENCODING_SPARSE ||
                   track->encoding == UMH_ENCODING_DELTA ||
                   track->encoding == UMH_ENCODING_ZERO_SUPPRESS ||
                   (track->encoding == UMH_ENCODING_CONSTANT && track->target_mode == UMH_TARGET_SPARSE)) {
          uint16_t pos = offset;
          uint16_t seen = 0u;
          uint8_t sparse_indices[UMH_DEVICE_RGB_COUNT];
          while (pos + 2u <= length) {
            uint8_t index = payload[pos];
            if (index >= UMH_DEVICE_RGB_COUNT) return -4;
            if (seen >= UMH_DEVICE_RGB_COUNT) return -4;
            if (rgb_seen_contains(sparse_indices, seen, index)) return -4;
            if (track->target_mode == UMH_TARGET_RANGE &&
                (index < track->target_start || index >= track->target_start + track->target_count)) return -4;
            if (track->target_mode == UMH_TARGET_BITMAP &&
                (payload[0] & (uint8_t)(1u << index)) == 0u) return -4;
            sparse_indices[seen] = index;
            if (track->encoding == UMH_ENCODING_DELTA) {
              int32_t level = (int32_t)parser->rgb_level[index] + (int8_t)payload[pos + 1u];
              parser->rgb_level[index] = (uint8_t)(level < 0 ? 0 : level > 255 ? 255 : level);
            } else {
              parser->rgb_level[index] = payload[pos + 1u];
            }
            pos = (uint16_t)(pos + 2u); ++seen;
          }
          if (pos != length || seen == 0u ||
              (track->target_mode == UMH_TARGET_SPARSE && seen != count) ||
              (track->target_mode != UMH_TARGET_SPARSE && seen > count)) return -4;
          memcpy(indices, sparse_indices, seen);
          count = seen;
        } else return -4;
        for (k = 0u; k < count; ++k)
          spatial_renderer_merge_rgb(parser->renderer, &parser->pending_frame,
                                     parser->rgb_source, parser->rgb_level,
                                     (uint8_t)(1u << indices[k]));
        return 0;
      }
    case UMH_PAYLOAD_DIGITAL_STATE:
      if (length != 2u) return -6;
      parser->pending_frame.digital_state =
          (uint8_t)((parser->pending_frame.digital_state & (uint8_t)~payload[0]) |
                    (payload[1] & payload[0]));
      parser->pending_frame.digital_mask |= payload[0];
      parser->pending_frame.update_flags |= UMH_FRAME_FLAG_DIGITAL;
      return 0;
    case UMH_PAYLOAD_RAW:
    case UMH_PAYLOAD_VECTOR:
    case UMH_PAYLOAD_SCALAR:
      if ((uint32_t)parser->pending_frame.extension_length + length >
          sizeof(parser->pending_frame.extension)) return -7;
      memcpy(&parser->pending_frame.extension[parser->pending_frame.extension_length],
             payload, length);
      parser->pending_frame.extension_length =
          (uint16_t)(parser->pending_frame.extension_length + length);
      parser->pending_frame.update_flags |= UMH_FRAME_FLAG_EXTENDED;
      return 0;
    default:
      (void)flags;
      return -8;
  }
}

static int parse_record_bytes(umh_block_parser_t *parser)
{
  uint32_t delta;
  uint32_t consumed;
  uint16_t track_id;
  uint8_t flags;
  uint16_t length;
  uint32_t variable_length;
  uint32_t length_consumed;
  const umh_track_wire_descriptor_t *track;
  uint16_t fixed_length;
  uint16_t pos = 0u;
  consumed = umh_varuint_decode(parser->record, parser->record_length, &delta);
  if (consumed == 0u || consumed + 3u > parser->record_length) return -1;
  pos = (uint16_t)consumed;
  track_id = (uint16_t)parser->record[pos] | ((uint16_t)parser->record[pos + 1u] << 8); pos += 2u;
  flags = parser->record[pos++];
  track = spatiotemporal_block_track(&parser->block, track_id);
  if (track == NULL) return -1;
  fixed_length = spatiotemporal_track_fixed_payload_size(track);
  if (fixed_length != 0u) length = fixed_length;
  else {
    length_consumed = umh_varuint_decode(&parser->record[pos],
                                         (uint16_t)(parser->record_length - pos),
                                         &variable_length);
    if (length_consumed == 0u || variable_length > UINT16_MAX) return -1;
    pos = (uint16_t)(pos + length_consumed);
    length = (uint16_t)variable_length;
  }
  if ((uint16_t)(pos + length) != parser->record_length) return -2;
  return apply_record(parser, delta, track_id, flags, &parser->record[pos], length);
}

void block_parser_init(umh_block_parser_t *parser,
                       umh_spatial_renderer_t *renderer,
                       umh_frame_ring_t *frames)
{
  if (parser == NULL) return;
  memset(parser, 0, sizeof(*parser));
  parser->renderer = renderer;
  parser->frames = frames;
  for (uint16_t i = 0u; i < UMH_DEVICE_RGB_COUNT; ++i) parser->rgb_level[i] = 255u;
}

int block_parser_begin(umh_block_parser_t *parser, const uint8_t *payload, uint16_t length)
{
  uint16_t i;
  int block_result;
  if (parser == NULL) return -1;
  block_parser_cancel(parser);
  /* rgb_source/rgb_level hold the uncalibrated logical state.  Keep them
   * across block boundaries: current_state.rgb already contains calibration
   * and brightness composition and feeding it back would apply both twice. */
  block_result = spatiotemporal_block_begin(&parser->block, payload, length);
  if (block_result != 0) return block_result;
  /* Reject spatial data before accepting stream bytes when geometry is not
   * available in the active device profile. */
  if (parser->renderer == NULL || parser->renderer->profile == NULL) {
    block_parser_cancel(parser);
    return -1;
  }
  for (i = 0u; i < parser->block.track_count; ++i) {
    if (parser->block.tracks[i].payload_type == UMH_PAYLOAD_SPATIAL_POINT &&
        (parser->renderer->profile->capability_flags & UMH_PROFILE_CAP_GEOMETRY_VALID) == 0u) {
      block_parser_cancel(parser);
      return -2;
    }
  }
  return 0;
}

int block_parser_data(umh_block_parser_t *parser, const uint8_t *payload, uint16_t length)
{
  uint16_t i;
  if (parser == NULL || payload == NULL || parser->block.active == 0u) return -1;
  for (i = 0u; i < length; ++i) {
    if (parser->record_length >= sizeof(parser->record)) {
      ++parser->parse_errors;
      return -2;
    }
    parser->record[parser->record_length++] = payload[i];
    if (parser->record_length >= 3u) {
      uint32_t delta;
      uint32_t consumed = umh_varuint_decode(parser->record, parser->record_length, &delta);
      if (consumed != 0u && parser->record_length >= consumed + 3u) {
        uint16_t track_id = (uint16_t)parser->record[consumed] |
                            ((uint16_t)parser->record[consumed + 1u] << 8);
        const umh_track_wire_descriptor_t *track =
            spatiotemporal_block_track(&parser->block, track_id);
        uint16_t fixed = spatiotemporal_track_fixed_payload_size(track);
        uint16_t header = (uint16_t)(consumed + 3u);
        if (track == NULL) return -3;
        if (fixed != 0u) {
          if ((uint32_t)header + fixed > sizeof(parser->record)) return -3;
          parser->record_expected = (uint16_t)(header + fixed);
          parser->record_active = 1u;
        } else if (parser->record_length > header) {
          uint32_t variable_length;
          uint32_t length_bytes = umh_varuint_decode(
              &parser->record[header], (uint16_t)(parser->record_length - header),
              &variable_length);
          if (length_bytes != 0u) {
            if (variable_length > UINT16_MAX ||
                (uint32_t)header + length_bytes + variable_length > sizeof(parser->record))
              return -3;
            parser->record_expected = (uint16_t)(header + length_bytes + variable_length);
            parser->record_active = 1u;
          }
        }
      }
    }
    if (parser->record_active != 0u && parser->record_length > parser->record_expected) {
      ++parser->parse_errors;
      block_parser_cancel(parser);
      return -3;
    }
    if (parser->record_active != 0u && parser->record_length == parser->record_expected) {
      if (parse_record_bytes(parser) != 0) {
        ++parser->parse_errors;
        block_parser_cancel(parser);
        return -3;
      }
      parser->record_length = 0u;
      parser->record_active = 0u;
      parser->record_expected = 0u;
      ++parser->block.records_received;
      if (parser->block.records_received > parser->block.header.record_count) return -4;
    }
  }
  return 0;
}

int block_parser_end(umh_block_parser_t *parser)
{
  if (parser == NULL || parser->block.active == 0u || parser->record_length != 0u) return -1;
  if (parser->block.records_received != parser->block.header.record_count) return -2;
  if (parser->block.current_time < parser->block.header.start_time ||
      parser->block.current_time > parser->block.header.start_time + parser->block.header.duration) return -2;
  if (commit_pending(parser) != 0) return -3;
  if (spatiotemporal_block_end(&parser->block) != 0) return -4;
  return 0;
}

void block_parser_cancel(umh_block_parser_t *parser)
{
  if (parser == NULL) return;
  parser->block.active = 0u;
  parser->record_length = 0u;
  parser->record_active = 0u;
  parser->pending_valid = 0u;
}

uint64_t block_parser_duration(const umh_block_parser_t *parser)
{
  return parser != NULL ? parser->block.header.duration : 0u;
}
