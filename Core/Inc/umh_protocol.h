#ifndef UMH_PROTOCOL_H
#define UMH_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#define UMH_PROTOCOL_VERSION             7u
#define UMH_PROTOCOL_SYNC0               0x55u
#define UMH_PROTOCOL_SYNC1               0xAAu
#define UMH_PROTOCOL_HEADER_SIZE         16u
#define UMH_PROTOCOL_MAX_PAYLOAD         2048u
#define UMH_PROTOCOL_RX_RING_SIZE        (4u * 1024u) /* 4K: one max frame plus burst; halved to keep SRAM1 at 96K (CCM alias reserved for .ccmram) */
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
  UMH_MSG_LIB_BEGIN          = 0x54,
  UMH_MSG_LIB_DATA           = 0x55,
  UMH_MSG_LIB_END            = 0x56,
  UMH_MSG_LIB_STATUS         = 0x57,
  UMH_MSG_LIB_START          = 0x58,
  UMH_MSG_LIB_STOP           = 0x59,
  UMH_MSG_ERROR_COUNTERS     = 0x60,
  UMH_MSG_SET_DEMO           = 0x61,
  /* Real-time focus motion generator.  UPLOAD stores a compact path in RAM,
   * CONFIG chooses live/path mode and physical limits, TARGET streams a live
   * point, START/STOP own the FPGA output exactly like the modulated-aperture
   * stream does. */
  UMH_MSG_MOTION_UPLOAD      = 0x62,
  UMH_MSG_MOTION_CONFIG      = 0x63,
  UMH_MSG_MOTION_START       = 0x64,
  UMH_MSG_MOTION_TARGET      = 0x65,
  UMH_MSG_MOTION_STOP        = 0x66,
  UMH_MSG_MOTION_STATUS      = 0x67,
  /* Path-rate ease for a running PATH program (4 bytes, see motion_engine.h). */
  UMH_MSG_MOTION_RATE        = 0x68,
  /* Modulated-aperture stream (see Core/Inc/aperture_mod.h).  FEED frames
   * deliberately do not require an ACK: the STM32 consumes envelope bytes from
   * the USB header sequence and keeps the streaming path independent of the
   * command/response loop. */
  UMH_MSG_MOD_CONFIGURE      = 0x90,
  UMH_MSG_MOD_START          = 0x91,
  UMH_MSG_MOD_FEED           = 0x92,
  UMH_MSG_MOD_STOP           = 0x93,
  UMH_MSG_MOD_STATUS         = 0x94,
  /* Single-channel burst start from silence, used to characterise the
   * turn-on transient of one element.  Payload is four bytes:
   *   [0] channel 0..83, [1] final level 1..128, [2] ramp_ticks 0..31,
   *   [3] reserved.
   * The device silences the array, enables that one element through the FPGA
   * common-level path, then plays a linear level ramp 0 -> level in
   * ramp_ticks steps of two carrier periods each (50 us at 40 kHz); zero
   * ramp_ticks is a hard step.  The FPGA paces the ramp from its own clock,
   * and the final level is held until STOP_PLAN / CLEAR_PLAN clears the
   * common-level mode. */
  UMH_MSG_RAMP_BURST         = 0x95,
  UMH_MSG_ACK                 = 0x70,
  UMH_MSG_NACK                = 0x71,
  UMH_MSG_CAL_DUMP            = 0x80,
  UMH_MSG_CAL_RESULT          = 0x81,
  UMH_MSG_CAL_START           = 0x82, /* bench/debug: start calibration */
  UMH_MSG_CAL_RAW             = 0x83, /* bench/debug: stream raw per-pattern I/Q */
  UMH_MSG_CAL_SELFTEST        = 0x84, /* run independent built-in phase self-test */
  UMH_MSG_CAL_SELFTEST_RESULT = 0x85, /* query last self-test result */
  UMH_MSG_CAL_PROBE           = 0x86, /* bench: multi-gate I/Q time profile */
  /* Device-side hologram keyframe executor (Core/Inc/hologram_engine.h).
   * UPLOAD stores whole 84-channel holograms in RAM, CONFIG sets the cadence and
   * interpolation time, START/STOP own the FPGA output exactly like motion and
   * aperture stream do.  The host keeps computing the holograms; the device
   * only interpolates between them so a long morph cannot be broken by USB. */
  UMH_MSG_HOLOGRAM_UPLOAD     = 0xA0,
  UMH_MSG_HOLOGRAM_CONFIG     = 0xA1,
  UMH_MSG_HOLOGRAM_START      = 0xA2,
  UMH_MSG_HOLOGRAM_STOP       = 0xA3,
  UMH_MSG_HOLOGRAM_STATUS     = 0xA5
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

typedef struct __attribute__((packed)) {
  uint32_t object_id;
  uint32_t record_count;
  uint32_t frame_period_us;
  uint8_t level_mode;
  uint8_t repeat;
  uint16_t reserved;
} umh_lib_begin_wire_t;

typedef struct __attribute__((packed)) {
  uint32_t object_id;
  uint8_t repeat;
  uint8_t reserved[3];
} umh_lib_start_wire_t;

typedef struct __attribute__((packed)) {
  uint8_t state;
  uint8_t object_count;
  uint8_t active_bank;
  uint8_t mounted;
  uint32_t free_bytes;
  uint32_t next_generation;
  uint32_t upload_object_id;
  uint32_t upload_received;
  uint32_t play_object_id;
  uint32_t play_record_count;
  uint32_t play_frame_period_us;
  uint32_t play_cursor;
  uint32_t replay_frames;
  uint32_t page_programs;
  uint32_t block_erases;
  uint32_t rotations;
  uint32_t bytes_programmed;
  uint16_t last_result;
  uint16_t reserved;
} umh_lib_status_wire_t;

_Static_assert(sizeof(umh_lib_begin_wire_t) == 16u, "lib begin wire size");
_Static_assert(sizeof(umh_lib_start_wire_t) == 8u, "lib start wire size");
_Static_assert(sizeof(umh_lib_status_wire_t) == 60u, "lib status wire size");

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
