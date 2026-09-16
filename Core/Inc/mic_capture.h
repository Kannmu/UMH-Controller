#ifndef MIC_CAPTURE_H
#define MIC_CAPTURE_H

#include <stdint.h>
#include "fpga_link.h"
#include "device_profile.h"
#include "cmsis_os.h"

typedef struct {
  int16_t i[UMH_DEVICE_MIC_COUNT];
  int16_t q[UMH_DEVICE_MIC_COUNT];
} mic_capture_gate_t;

/* Thin wrapper over the FPGA microphone commands.  It keeps the calibration
 * state machine free of raw SPI framing details. */
int mic_capture_configure(fpga_link_t *link, uint8_t gate_count, uint16_t start,
                          uint16_t step, uint8_t width);
int mic_capture_wait_block(fpga_link_t *link, uint16_t expected_block,
                           uint32_t timeout_ms, fpga_mic_gate_wire_t *status_out);
int mic_capture_read_gate(fpga_link_t *link, uint8_t gate,
                          mic_capture_gate_t *gate_out,
                          fpga_mic_gate_wire_t *status_out);

#endif
