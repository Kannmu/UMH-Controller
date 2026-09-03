#include "umh_protocol.h"
#include <string.h>

static uint16_t read_u16_le(const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void write_u16_le(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

static uint32_t read_u32_le(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void write_u32_le(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

void umh_rx_ring_init(umh_rx_ring_t *ring)
{
  if (ring == NULL) return;
  ring->write_index = 0;
  ring->read_index = 0;
  ring->dropped_bytes = 0;
}

uint32_t umh_rx_ring_available(const umh_rx_ring_t *ring)
{
  uint32_t w = ring->write_index;
  uint32_t r = ring->read_index;
  return w - r;
}

uint32_t umh_rx_ring_write(umh_rx_ring_t *ring, const uint8_t *data, uint32_t length)
{
  uint32_t available;
  uint32_t accepted;
  uint32_t i;
  if (ring == NULL || data == NULL || length == 0u) return 0u;
  available = UMH_PROTOCOL_RX_RING_SIZE - umh_rx_ring_available(ring);
  accepted = (length < available) ? length : available;
  for (i = 0u; i < accepted; ++i) {
    ring->bytes[(ring->write_index + i) % UMH_PROTOCOL_RX_RING_SIZE] = data[i];
  }
  ring->write_index += accepted;
  ring->dropped_bytes += length - accepted;
  return accepted;
}

uint32_t umh_rx_ring_read(umh_rx_ring_t *ring, uint8_t *data, uint32_t length)
{
  uint32_t available;
  uint32_t count;
  uint32_t i;
  if (ring == NULL || data == NULL || length == 0u) return 0u;
  available = umh_rx_ring_available(ring);
  count = (length < available) ? length : available;
  for (i = 0u; i < count; ++i) {
    data[i] = ring->bytes[(ring->read_index + i) % UMH_PROTOCOL_RX_RING_SIZE];
  }
  ring->read_index += count;
  return count;
}

static void parser_reset(umh_protocol_parser_t *parser)
{
  parser->header_pos = 0u;
  parser->payload_pos = 0u;
  parser->expected_payload = 0u;
}

static void parser_try_header(umh_protocol_parser_t *parser)
{
  const uint8_t *h = parser->header_bytes;
  if (h[0] != UMH_PROTOCOL_SYNC0 || h[1] != UMH_PROTOCOL_SYNC1 ||
      h[2] != UMH_PROTOCOL_VERSION || h[5] != UMH_PROTOCOL_HEADER_SIZE ||
      (h[4] & (uint8_t)~(UMH_FLAG_ACK_REQUIRED | UMH_FLAG_FIRST |
                         UMH_FLAG_LAST | UMH_FLAG_RESPONSE | UMH_FLAG_ERROR)) != 0u) {
    ++parser->parser_errors;
    parser->header_pos = (h[0] == UMH_PROTOCOL_SYNC0) ? 1u : 0u;
    return;
  }
  parser->frame.header.sync0 = h[0];
  parser->frame.header.sync1 = h[1];
  parser->frame.header.protocol_version = h[2];
  parser->frame.header.message_type = h[3];
  parser->frame.header.flags = h[4];
  parser->frame.header.header_length = h[5];
  parser->frame.header.payload_length = read_u16_le(&h[6]);
  parser->frame.header.transaction_id = read_u32_le(&h[8]);
  parser->frame.header.stream_sequence = read_u32_le(&h[12]);
  parser->expected_payload = parser->frame.header.payload_length;
  if (parser->expected_payload > UMH_PROTOCOL_MAX_PAYLOAD) {
    ++parser->parser_errors;
    parser_reset(parser);
    return;
  }
  parser->payload_pos = 0u;
}

uint32_t umh_protocol_parser_error_count(const umh_protocol_parser_t *parser)
{
  return parser != NULL ? parser->parser_errors : 0u;
}

void umh_protocol_parser_init(umh_protocol_parser_t *parser,
                              umh_protocol_frame_cb_t callback,
                              void *context)
{
  if (parser == NULL) return;
  memset(parser, 0, sizeof(*parser));
  parser->callback = callback;
  parser->callback_context = context;
}

void umh_protocol_parser_feed(umh_protocol_parser_t *parser,
                              const uint8_t *data, uint32_t length)
{
  uint32_t i;
  if (parser == NULL || data == NULL) return;
  for (i = 0u; i < length; ++i) {
    if (parser->header_pos < UMH_PROTOCOL_HEADER_SIZE) {
      parser->header_bytes[parser->header_pos++] = data[i];
      if (parser->header_pos == UMH_PROTOCOL_HEADER_SIZE) {
        parser_try_header(parser);
        if (parser->header_pos != UMH_PROTOCOL_HEADER_SIZE) continue;
        if (parser->expected_payload == 0u) {
          parser->frame.payload_size = 0u;
          if (parser->callback != NULL) parser->callback(&parser->frame, parser->callback_context);
          parser_reset(parser);
        }
      }
      continue;
    }
    parser->frame.payload[parser->payload_pos++] = data[i];
    if (parser->payload_pos == parser->expected_payload) {
      parser->frame.payload_size = parser->expected_payload;
      if (parser->callback != NULL) parser->callback(&parser->frame, parser->callback_context);
      parser_reset(parser);
    }
  }
}

uint16_t umh_protocol_encode(uint8_t message_type, uint8_t flags,
                             uint32_t transaction_id, uint32_t stream_sequence,
                             const uint8_t *payload, uint16_t payload_length,
                             uint8_t *output, uint16_t output_capacity)
{
  if (output == NULL || (payload_length != 0u && payload == NULL) ||
      payload_length > UMH_PROTOCOL_MAX_PAYLOAD ||
      output_capacity < (uint16_t)(UMH_PROTOCOL_HEADER_SIZE + payload_length)) return 0u;
  output[0] = UMH_PROTOCOL_SYNC0;
  output[1] = UMH_PROTOCOL_SYNC1;
  output[2] = UMH_PROTOCOL_VERSION;
  output[3] = message_type;
  output[4] = flags;
  output[5] = UMH_PROTOCOL_HEADER_SIZE;
  write_u16_le(&output[6], payload_length);
  write_u32_le(&output[8], transaction_id);
  write_u32_le(&output[12], stream_sequence);
  if (payload_length != 0u && payload != NULL) memcpy(&output[UMH_PROTOCOL_HEADER_SIZE], payload, payload_length);
  return (uint16_t)(UMH_PROTOCOL_HEADER_SIZE + payload_length);
}
