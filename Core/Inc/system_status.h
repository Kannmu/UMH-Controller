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

typedef struct {
  volatile uint32_t flags;
  volatile uint32_t protocol_errors;
  volatile uint32_t parser_errors;
  volatile uint32_t fpga_errors;
  volatile uint32_t underruns;
  volatile uint32_t usb_dropped;
  volatile uint16_t frame_count;
  volatile uint16_t frame_free;
  volatile uint16_t fpga_credit;
  volatile uint16_t fpga_depth;
  volatile uint32_t device_time;
  volatile uint8_t heartbeat;
} umh_system_status_t;

void system_status_init(void);
void system_time_init(void);
uint64_t system_time_us(void);
umh_system_status_t *system_status_get(void);
void system_status_set(uint32_t flags);
void system_status_clear(uint32_t flags);
void system_status_error(uint32_t count);

#endif
