#ifndef SYSTEM_STATUS_H
#define SYSTEM_STATUS_H

#include <stdint.h>

typedef enum {
  UMH_SYSTEM_CONNECTED = 1u << 0,
  UMH_SYSTEM_PLAYING = 1u << 1,
  UMH_SYSTEM_FPGA_READY = 1u << 2,
  UMH_SYSTEM_FLASH_READY = 1u << 3,
  UMH_SYSTEM_CALIBRATION_VALID = 1u << 4,
  UMH_SYSTEM_UNDERRUN = 1u << 5,
  UMH_SYSTEM_ERROR = 1u << 6
} umh_system_flag_t;

/* Stable, short identifiers used by the OLED and the USB error counters.
 * Keep these codes semantic: the UI can explain the cause without exposing
 * a source-file-dependent numeric error. */
typedef enum {
  UMH_FAULT_NONE = 0u,
  UMH_FAULT_FPGA_SPI_START = 1u,
  UMH_FAULT_FPGA_SPI_TIMEOUT,
  UMH_FAULT_FPGA_SPI_DMA,
  UMH_FAULT_FPGA_PROTOCOL,
  UMH_FAULT_FPGA_UNDERRUN,
  UMH_FAULT_FPGA_OVERFLOW,
  UMH_FAULT_FPGA_INVALID_FRAME,
  UMH_FAULT_FPGA_OUTPUT,
  UMH_FAULT_PROTOCOL_NACK,
  UMH_FAULT_PROTOCOL_PARSE,
  UMH_FAULT_BLOCK_PARSE,
  UMH_FAULT_PLAN_START,
  UMH_FAULT_FRAME_RING,
  UMH_FAULT_FLASH_IO,
  UMH_FAULT_EEPROM_IO,
  UMH_FAULT_USB_RX_DROP,
  UMH_FAULT_USB_TX_DROP,
  UMH_FAULT_HAL_INIT,
  UMH_FAULT_COUNT
} umh_fault_code_t;

typedef enum {
  UMH_FAULT_INFO = 0u,
  UMH_FAULT_WARNING = 1u,
  UMH_FAULT_CRITICAL = 2u
} umh_fault_severity_t;

typedef struct {
  volatile uint32_t flags;
  volatile uint32_t protocol_errors;
  volatile uint32_t parser_errors;
  volatile uint32_t fpga_errors;
  volatile uint32_t underruns;
  volatile uint32_t usb_dropped;
  volatile uint32_t rx_dropped;
  volatile uint32_t frame_dropped;
  volatile uint16_t frame_count;
  volatile uint16_t frame_free;
  volatile uint16_t fpga_credit;
  volatile uint16_t fpga_depth;
  volatile uint32_t device_time;
  volatile uint32_t uptime_ms;
  volatile uint32_t last_fault_time_ms;
  volatile uint32_t fault_count;
  volatile uint32_t last_fault_arg;
  volatile uint16_t last_fault_code;
  volatile uint8_t last_fault_severity;
  volatile uint8_t reserved;
  volatile uint32_t critical_fault_time_ms;
  volatile uint32_t critical_fault_arg;
  volatile uint16_t critical_fault_code;
  volatile uint8_t critical_fault_valid;
  volatile uint8_t reserved2;
  volatile uint8_t heartbeat;
} umh_system_status_t;

void system_status_init(void);
void system_time_init(void);
uint64_t system_time_us(void);
umh_system_status_t *system_status_get(void);
void system_status_set(uint32_t flags);
void system_status_clear(uint32_t flags);
void system_status_error(uint32_t count);
void system_status_fault(umh_fault_code_t code, uint32_t argument,
                         umh_fault_severity_t severity);

#endif
