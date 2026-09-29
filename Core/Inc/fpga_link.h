#ifndef FPGA_LINK_H
#define FPGA_LINK_H

#include <stdint.h>
#include "spi.h"
#include "frame_ring.h"
#include "cmsis_os.h"

#define FPGA_PROTOCOL_VERSION 1u
#define FPGA_FIFO_DEFAULT_CREDIT 64u
#define FPGA_TX_BUFFER_SIZE 256u  /* longest transaction: 36+168+12+32 = 248 */
#define FPGA_ULTRASOUND_BITMAP_LAST_MASK 0x0Fu
#define FPGA_DIGITAL_TRIGGER_BIT 0x01u
#define FPGA_STATUS_UNDERRUN      (1u << 0)
#define FPGA_STATUS_OVERFLOW      (1u << 1)
#define FPGA_STATUS_INVALID_FRAME (1u << 2)
#define FPGA_STATUS_OUTPUT_FAULT  (1u << 3)
#define FPGA_STATUS_RUNNING       (1u << 4)
/* Diagnostic bit 15: focused-AM common-level mode is active.  STM32 uses it
 * to prove that a new FPGA bitstream understands AUDIO_MODE, so old logic
 * can never be mistaken for a successful audio configuration. */
#define FPGA_STATUS_AUDIO_MODE    (1u << 15)
/* Bit 12: this FPGA image has the AUDIO_BLOCK (0x1A) level FIFO. */
#define FPGA_STATUS_AUDIO_BLOCK   (1u << 12)
/* AUDIO_BLOCK: [0]=0x1A [1]=version [2..]=levels.  The reply's byte 1 is the
 * FIFO fill before the transaction.  More than 32 levels would reach header
 * byte 35 and flag the frame invalid; the FIFO holds 256 levels, consumed at
 * one per two carrier periods (20 kHz). */
#define FPGA_AUDIO_BLOCK_MAX      32u
#define FPGA_AUDIO_FIFO_DEPTH     256u
#define FPGA_AUDIO_OUTPUT_RATE_HZ 20000u

/* Microphone calibration commands.  MIC_CONFIG carries six extension bytes:
 *   [0] gate_count (1..64), [1..2] gate0 start, [3..4] start-to-start step,
 *   [5] gate width.  The three time fields are in 40 kHz samples (25 us),
 *   matching the fixed one-carrier-period microphone I/Q integrator.
 * MIC_READ carries the gate index in the header frame-sequence field and
 * replies with 24 payload bytes after the usual 16 status bytes:
 *   status u16, block_count u16, gate_count u16, reserved u16,
 *   I0..I3 u16, Q0..Q3 u16   (all big-endian). */
#define FPGA_MIC_READ_BYTES 40u
#define FPGA_MIC_MAX_GATES 64u
#define FPGA_MIC_STATUS_DONE          (1u << 0)
#define FPGA_MIC_STATUS_WAIT_PATTERN  (1u << 1)
#define FPGA_MIC_STATUS_SATURATED     (1u << 2)
#define FPGA_MIC_STATUS_RUNNING       (1u << 3)
#define FPGA_MIC_STATUS_CONFIGURED    (1u << 4)

typedef enum {
  FPGA_CMD_STATUS = 0x01u,
  FPGA_CMD_FRAME = 0x10u,
  FPGA_CMD_STOP = 0x11u,
  FPGA_CMD_RESET = 0x12u,
  FPGA_CMD_WS2812 = 0x13u,
  FPGA_CMD_MIC_CONFIG = 0x14u,
  FPGA_CMD_MIC_READ = 0x15u,
  /* Compact 16-byte AUDIO_MODE enables/disables the common-envelope
   * rebuild path, which keeps the 84 phases loaded by a normal FRAME.
   * AUDIO_BLOCK then streams common levels into the FPGA FIFO. */
  FPGA_CMD_AUDIO_MODE = 0x17u,
  FPGA_CMD_AUDIO_BLOCK = 0x1Au
} fpga_command_t;

typedef struct {
  uint16_t status;
  uint16_t block_count;
  uint16_t gate_count;
  uint16_t reserved;
  int16_t i[UMH_DEVICE_MIC_COUNT];
  int16_t q[UMH_DEVICE_MIC_COUNT];
} fpga_mic_gate_wire_t;

typedef struct __attribute__((packed)) {
  uint8_t protocol_version;
  uint8_t audio_fill;           /* AUDIO_BLOCK FIFO fill before the transaction */
  uint16_t fifo_credit;
  uint16_t fifo_depth;
  uint16_t status_flags;
  uint32_t fpga_time;
  uint32_t accepted_sequence;
} fpga_status_wire_t;

_Static_assert(sizeof(fpga_status_wire_t) == 16u, "FPGA status wire size");

typedef struct {
  SPI_HandleTypeDef *spi;
  uint8_t tx[FPGA_TX_BUFFER_SIZE];
  uint8_t rx[FPGA_TX_BUFFER_SIZE];
  volatile uint8_t dma_done;
  volatile uint8_t dma_error;
  uint16_t tx_length;
  uint32_t transaction_sequence;
  fpga_status_wire_t status;
  uint8_t running;
  umh_output_frame_t hold_frame;
  uint8_t hold_valid;
  uint32_t hold_last_tx_tick;
  osMutexId_t mutex;
  StaticSemaphore_t mutex_memory;
} fpga_link_t;

/* Runtime logical-channel -> us_tx bit permutation.  Initialized to identity
 * by fpga_link_init(); bench code may overwrite it with the board mapping. */
extern uint8_t fpga_logical_to_physical[UMH_DEVICE_CHANNEL_COUNT];

void fpga_link_init(fpga_link_t *link, SPI_HandleTypeDef *spi);
uint32_t fpga_link_calibration_link_begin(fpga_link_t *link);
void fpga_link_calibration_link_end(fpga_link_t *link, uint32_t saved_cr1);
int fpga_link_submit(fpga_link_t *link, const umh_output_frame_t *frame);
int fpga_link_poll_status(fpga_link_t *link);
int fpga_link_submit_allow_hold(fpga_link_t *link, const umh_output_frame_t *frame, uint8_t allow_hold);
int fpga_link_safe_stop(fpga_link_t *link);
int fpga_link_set_ws2812(fpga_link_t *link, uint8_t r, uint8_t g, uint8_t b);
int fpga_link_mic_config(fpga_link_t *link, uint8_t gate_count, uint16_t start,
                         uint16_t step, uint8_t width);
/* Focused-AM setup.  The 84 phase bytes are loaded through one ordinary
 * FRAME, then AUDIO_MODE switches the FPGA event builder to the common-level
 * substitution path.  Audio data itself uses fpga_link_audio_block(). */
int fpga_link_audio_begin(fpga_link_t *link, const uint8_t *phases,
                          const uint8_t *enables, uint32_t sequence);
/* Appends count (<= FPGA_AUDIO_BLOCK_MAX) levels to the FPGA FIFO; count 0
 * is a 2-byte fill poll.  Returns the fill before the transfer, or < 0. */
int fpga_link_audio_block(fpga_link_t *link, const uint8_t *levels, uint8_t count);
int fpga_link_audio_mode(fpga_link_t *link, uint8_t enable, uint32_t sequence);
int fpga_link_mic_read(fpga_link_t *link, uint8_t gate, fpga_mic_gate_wire_t *result);
const fpga_status_wire_t *fpga_link_status(const fpga_link_t *link);
void fpga_link_spi_txrx_complete(fpga_link_t *link);
void fpga_link_spi_error(fpga_link_t *link);

#endif
