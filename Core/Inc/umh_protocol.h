#ifndef UMH_PROTOCOL_H
#define UMH_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#define UMH_PROTOCOL_VERSION             7u
#define UMH_PROTOCOL_SYNC0               0x55u
#define UMH_PROTOCOL_SYNC1               0xAAu
#define UMH_PROTOCOL_HEADER_SIZE         16u
#define UMH_PROTOCOL_MAX_PAYLOAD         2048u
#define UMH_PROTOCOL_RX_RING_SIZE        (8u * 1024u)
#define UMH_PROTOCOL_TX_POOL_SIZE        (4u * 1024u)

typedef enum {
  UMH_MSG_GET_PROFILE       = 0x01,
  UMH_MSG_PROFILE            = 0x02,
  UMH_MSG_GET_STATUS         = 0x03,
  UMH_MSG_STATUS              = 0x04,
  UMH_MSG_BLOCK_BEGIN        = 0x10,
  UMH_MSG_BLOCK_DATA         = 0x11,
  UMH_MSG_BLOCK_END          = 0x12,
  UMH_MSG_BLOCK_CANCEL       = 0x13,
  UMH_MSG_SET_PLAN           = 0x20,
  UMH_MSG_START_PLAN         = 0x21,
  UMH_MSG_STOP_PLAN          = 0x22,
  UMH_MSG_CLEAR_PLAN         = 0x23,
  UMH_MSG_FPGA_STATUS        = 0x30,
  UMH_MSG_EEPROM_READ        = 0x40,
  UMH_MSG_EEPROM_WRITE       = 0x41,
  UMH_MSG_EEPROM_COMMIT      = 0x42,
  UMH_MSG_FLASH_LIST         = 0x50,
  UMH_MSG_FLASH_READ         = 0x51,
  UMH_MSG_FLASH_WRITE        = 0x52,
  UMH_MSG_FLASH_DELETE       = 0x53,
  UMH_MSG_ERROR_COUNTERS     = 0x60,
  UMH_MSG_SET_DEMO           = 0x61,
  UMH_MSG_ACK                 = 0x70,
  UMH_MSG_NACK                = 0x71
} umh_message_type_t;

typedef enum {
  UMH_FLAG_ACK_REQUIRED = 1u << 0,
  UMH_FLAG_FIRST         = 1u << 1,
  UMH_FLAG_LAST          = 1u << 2,
  UMH_FLAG_RESPONSE      = 1u << 3,
  UMH_FLAG_ERROR         = 1u << 4
} umh_frame_flags_t;

typedef enum {
  UMH_STATUS_OK              = 0,
  UMH_STATUS_BAD_HEADER      = 1,
  UMH_STATUS_BAD_LENGTH      = 2,
  UMH_STATUS_BAD_SEQUENCE    = 3,
  UMH_STATUS_NO_MEMORY       = 4,
  UMH_STATUS_UNSUPPORTED     = 5,
  UMH_STATUS_INVALID_STATE   = 6,
  UMH_STATUS_BUSY            = 7,
  UMH_STATUS_IO              = 8,
  UMH_STATUS_CRC             = 9
} umh_status_t;

typedef struct __attribute__((packed)) {
  uint8_t sync0;
  uint8_t sync1;
  uint8_t protocol_version;
  uint8_t message_type;
  uint8_t flags;
  uint8_t header_length;
  uint16_t payload_length;
  uint32_t transaction_id;
  uint32_t stream_sequence;
} umh_wire_header_t;

_Static_assert(sizeof(umh_wire_header_t) == UMH_PROTOCOL_HEADER_SIZE,
               "v7 USB header is part of the wire contract");

typedef struct {
  uint8_t bytes[UMH_PROTOCOL_RX_RING_SIZE];
  volatile uint32_t write_index;
  volatile uint32_t read_index;
  volatile uint32_t dropped_bytes;
} umh_rx_ring_t;

typedef struct {
  umh_wire_header_t header;
  uint16_t payload_size;
  uint8_t payload[UMH_PROTOCOL_MAX_PAYLOAD];
} umh_protocol_frame_t;

typedef void (*umh_protocol_frame_cb_t)(const umh_protocol_frame_t *frame,
                                        void *context);

typedef struct {
  uint8_t header_bytes[UMH_PROTOCOL_HEADER_SIZE];
  uint8_t header_pos;
  uint16_t payload_pos;
  uint16_t expected_payload;
  umh_protocol_frame_t frame;
  umh_protocol_frame_cb_t callback;
  void *callback_context;
  uint32_t parser_errors;
} umh_protocol_parser_t;

void umh_rx_ring_init(umh_rx_ring_t *ring);
uint32_t umh_rx_ring_write(umh_rx_ring_t *ring, const uint8_t *data, uint32_t length);
uint32_t umh_rx_ring_read(umh_rx_ring_t *ring, uint8_t *data, uint32_t length);
uint32_t umh_rx_ring_available(const umh_rx_ring_t *ring);

void umh_protocol_parser_init(umh_protocol_parser_t *parser,
                              umh_protocol_frame_cb_t callback,
                              void *context);
void umh_protocol_parser_feed(umh_protocol_parser_t *parser,
                              const uint8_t *data, uint32_t length);
uint32_t umh_protocol_parser_error_count(const umh_protocol_parser_t *parser);
uint16_t umh_protocol_encode(uint8_t message_type, uint8_t flags,
                             uint32_t transaction_id, uint32_t stream_sequence,
                             const uint8_t *payload, uint16_t payload_length,
                             uint8_t *output, uint16_t output_capacity);

#endif
