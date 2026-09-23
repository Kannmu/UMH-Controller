#ifndef FPGA_LINK_H
#define FPGA_LINK_H

#include <stdint.h>
#include "spi.h"
#include "frame_ring.h"
#include "cmsis_os.h"

#define FPGA_PROTOCOL_VERSION 1u
#define FPGA_FIFO_DEFAULT_CREDIT 64u
#define FPGA_TX_BUFFER_SIZE 512u
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
/* Diagnostic bit 14: this FPGA image accepts the 3-byte AUDIO_LEVEL_SHORT
 * (0x19) hot path.  Old images can still stream through 0x16. */
#define FPGA_STATUS_AUDIO_SHORT   (1u << 14)

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
  /* Compact 16-byte focused-AM transactions.  AUDIO_MODE enables/disables
   * the common-envelope rebuild path; AUDIO_LEVEL substitutes one common
   * level byte while keeping the 84 phases already loaded by a normal FRAME. */
  FPGA_CMD_AUDIO_LEVEL = 0x16u,
  FPGA_CMD_AUDIO_MODE = 0x17u,
  /* Three-byte hot path for the 20 kHz envelope stream. */
  FPGA_CMD_AUDIO_LEVEL_SHORT = 0x19u
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
  uint8_t reserved;
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
  uint8_t audio_short_supported;
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
int fpga_link_submit(fpga_link_t *link, const umh_output_frame_t *frame);
int fpga_link_poll_status(fpga_link_t *link);
int fpga_link_submit_allow_hold(fpga_link_t *link, const umh_output_frame_t *frame, uint8_t allow_hold);
int fpga_link_safe_stop(fpga_link_t *link);
int fpga_link_set_ws2812(fpga_link_t *link, uint8_t r, uint8_t g, uint8_t b);
int fpga_link_mic_config(fpga_link_t *link, uint8_t gate_count, uint16_t start,
                         uint16_t step, uint8_t width);
/* Focused-AM setup.  The 84 phase bytes are loaded through one ordinary
 * FRAME, then AUDIO_MODE switches the FPGA event builder to the common-level
 * substitution path.  Audio data itself uses fpga_link_audio_level(). */
int fpga_link_audio_begin(fpga_link_t *link, const uint8_t *phases,
                          const uint8_t *enables, uint32_t sequence);
int fpga_link_audio_level(fpga_link_t *link, uint8_t level, uint32_t sequence);
/* Hot-path variant: 3-byte transaction, no mutex/status read.  It must only
 * be called from the highest-priority render task while focused-AM mode is
 * active and no other task can be in fpga_link SPI code. */
int fpga_link_audio_level_fast(fpga_link_t *link, uint8_t level);
int fpga_link_audio_mode(fpga_link_t *link, uint8_t enable, uint32_t sequence);
int fpga_link_mic_read(fpga_link_t *link, uint8_t gate, fpga_mic_gate_wire_t *result);
const fpga_status_wire_t *fpga_link_status(const fpga_link_t *link);
void fpga_link_spi_txrx_complete(fpga_link_t *link);
void fpga_link_spi_error(fpga_link_t *link);

#endif
