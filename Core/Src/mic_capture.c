#include "mic_capture.h"
#include "system_status.h"

int mic_capture_configure(fpga_link_t *link, uint8_t gate_count, uint16_t start,
                          uint16_t step, uint8_t width)
{
  if (link == NULL || gate_count == 0u || width == 0u) return -1;
  return fpga_link_mic_config(link, gate_count, start, step, width);
}

int mic_capture_wait_block(fpga_link_t *link, uint16_t expected_block,
                           uint32_t timeout_ms, fpga_mic_gate_wire_t *status_out)
{
  uint32_t start_ms = HAL_GetTick();
  fpga_mic_gate_wire_t gate;
  if (link == NULL) return -1;
  for (;;) {
    if (fpga_link_mic_read(link, 0u, &gate) == 0) {
      if ((gate.status & FPGA_MIC_STATUS_DONE) != 0u &&
          gate.block_count >= expected_block) {
        if (status_out != NULL) *status_out = gate;
        return 0;
      }
    }
    if ((HAL_GetTick() - start_ms) >= timeout_ms) return -2;
    osDelay(1u);
  }
}

int mic_capture_read_gate(fpga_link_t *link, uint8_t gate,
                          mic_capture_gate_t *gate_out,
                          fpga_mic_gate_wire_t *status_out)
{
  fpga_mic_gate_wire_t wire;
  uint8_t k;
  if (link == NULL || gate_out == NULL) return -1;
  if (fpga_link_mic_read(link, gate, &wire) != 0) return -2;
  if (gate >= wire.gate_count) return -3;
  for (k = 0u; k < UMH_DEVICE_MIC_COUNT; ++k) {
    gate_out->i[k] = wire.i[k];
    gate_out->q[k] = wire.q[k];
  }
  if (status_out != NULL) *status_out = wire;
  return 0;
}
