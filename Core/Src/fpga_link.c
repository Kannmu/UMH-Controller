#include "fpga_link.h"
#include "main.h"
#include "cmsis_os.h"
#include "system_status.h"
#include <string.h>

static void cs_low(void) { HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_RESET); }
static void cs_high(void) { HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_SET); }

static void mark_link_fault(fpga_link_t *link)
{
  if (link == NULL) return;
  link->running = 0u;
  link->status.status_flags |= FPGA_STATUS_OUTPUT_FAULT;
  HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);
}

static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put_u32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }
static void put_u64(uint8_t *p, uint64_t v) { put_u32(p, (uint32_t)v); put_u32(p + 4u, (uint32_t)(v >> 32)); }
static uint16_t get_u16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static uint32_t get_u32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

void fpga_link_init(fpga_link_t *link, SPI_HandleTypeDef *spi)
{
  osMutexAttr_t mutex_attributes;
  if (link == NULL) return;
  memset(link, 0, sizeof(*link));
  link->spi = spi;
  memset(&mutex_attributes, 0, sizeof(mutex_attributes));
  mutex_attributes.name = "fpga-link";
  mutex_attributes.cb_mem = &link->mutex_memory;
  mutex_attributes.cb_size = sizeof(link->mutex_memory);
  link->mutex = osMutexNew(&mutex_attributes);
  link->status.protocol_version = FPGA_PROTOCOL_VERSION;
  link->status.fifo_credit = FPGA_FIFO_DEFAULT_CREDIT;
  cs_high();
}

static int exchange(fpga_link_t *link, uint16_t length)
{
  uint32_t start;
  if (link == NULL || link->spi == NULL || length == 0u || length > FPGA_TX_BUFFER_SIZE) return -1;
  link->dma_done = 0u;
  link->dma_error = 0u;
  link->tx_length = length;
  cs_low();
  if (HAL_SPI_TransmitReceive_DMA(link->spi, link->tx, link->rx, length) != HAL_OK) {
    cs_high();
    mark_link_fault(link);
    system_status_fault(UMH_FAULT_FPGA_SPI_START, HAL_SPI_GetError(link->spi), UMH_FAULT_CRITICAL);
    return -2;
  }
  start = HAL_GetTick();
  while (link->dma_done == 0u && link->dma_error == 0u) {
    if ((HAL_GetTick() - start) > 100u) {
      (void)HAL_SPI_Abort(link->spi);
      cs_high();
      mark_link_fault(link);
      system_status_fault(UMH_FAULT_FPGA_SPI_TIMEOUT, 100u, UMH_FAULT_CRITICAL);
      return -3;
    }
    osDelay(1u);
  }
  cs_high();
  if (link->dma_error != 0u) {
    mark_link_fault(link);
    system_status_fault(UMH_FAULT_FPGA_SPI_DMA, HAL_SPI_GetError(link->spi), UMH_FAULT_CRITICAL);
    return -4;
  }
  return 0;
}

static uint16_t pack_common(fpga_link_t *link, uint8_t command, const umh_output_frame_t *frame)
{
  uint16_t pos = 0u;
  uint16_t i;
  uint16_t extension_length = 0u;
  uint8_t mask[UMH_DEVICE_CHANNEL_BITMAP_BYTES] = {0};
  uint8_t update = frame != NULL ? (uint8_t)frame->update_flags : 0u;
  if (link == NULL) return 0u;
  if (frame != NULL && (update & UMH_FRAME_FLAG_EXTENDED) != 0u) {
    extension_length = frame->extension_length;
    if (extension_length > sizeof(frame->extension)) return 0u;
  }
  link->tx[pos++] = command;
  link->tx[pos++] = FPGA_PROTOCOL_VERSION;
  put_u32(&link->tx[pos], link->transaction_sequence++); pos += 4u;
  put_u32(&link->tx[pos], frame != NULL ? frame->sequence : 0u); pos += 4u;
  put_u64(&link->tx[pos], frame != NULL ? frame->deadline : 0u); pos += 8u;
  put_u16(&link->tx[pos], frame != NULL ? frame->update_flags : 0u); pos += 2u;
  if ((update & UMH_FRAME_FLAG_ULTRASOUND) != 0u) {
    memset(mask, 0xFF, sizeof(mask));
    mask[sizeof(mask) - 1u] = FPGA_ULTRASOUND_BITMAP_LAST_MASK;
  }
  memcpy(&link->tx[pos], mask, sizeof(mask)); pos += sizeof(mask);
  link->tx[pos++] = (update & UMH_FRAME_FLAG_RGB) != 0u ? 0x0Fu : 0u;
  link->tx[pos++] = frame != NULL ? frame->digital_mask : 0u;
  link->tx[pos++] = frame != NULL ? frame->digital_state : 0u;
  put_u16(&link->tx[pos], extension_length); pos += 2u;
  if (frame != NULL && (update & UMH_FRAME_FLAG_ULTRASOUND) != 0u) {
    for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) {
      link->tx[pos++] = frame->channels[i].phase;
      link->tx[pos++] = frame->channels[i].level;
    }
  }
  if (frame != NULL && (update & UMH_FRAME_FLAG_RGB) != 0u) {
    memcpy(&link->tx[pos], frame->rgb, sizeof(frame->rgb));
    pos = (uint16_t)(pos + sizeof(frame->rgb));
  }
  if (extension_length != 0u) {
    memcpy(&link->tx[pos], frame->extension, extension_length);
    pos = (uint16_t)(pos + extension_length);
  }
  if (pos > FPGA_TX_BUFFER_SIZE) return 0u;
  return pos;
}

static int unpack_status(fpga_link_t *link)
{
  if (link == NULL || link->tx_length < sizeof(fpga_status_wire_t)) return -1;
  link->status.protocol_version = link->rx[0];
  link->status.fifo_credit = get_u16(&link->rx[2]);
  link->status.fifo_depth = get_u16(&link->rx[4]);
  link->status.status_flags = get_u16(&link->rx[6]);
  link->status.fpga_time = get_u32(&link->rx[8]);
  link->status.accepted_sequence = get_u32(&link->rx[12]);
  if (link->status.protocol_version != FPGA_PROTOCOL_VERSION) {
    link->running = 0u;
    system_status_clear(UMH_SYSTEM_FPGA_READY);
    system_status_set(UMH_SYSTEM_ERROR);
    system_status_fault(UMH_FAULT_FPGA_PROTOCOL, link->status.protocol_version, UMH_FAULT_CRITICAL);
    return -2;
  }
  system_status_set(UMH_SYSTEM_FPGA_READY);
  return 0;
}

static void apply_digital(const umh_output_frame_t *frame)
{
  if (frame == NULL || (frame->update_flags & UMH_FRAME_FLAG_DIGITAL) == 0u) return;
  if ((frame->digital_mask & FPGA_DIGITAL_TRIGGER_BIT) != 0u) {
    HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin,
                      (frame->digital_state & FPGA_DIGITAL_TRIGGER_BIT) != 0u ?
                      GPIO_PIN_SET : GPIO_PIN_RESET);
  }
}

int fpga_link_submit(fpga_link_t *link, const umh_output_frame_t *frame)
{
  uint16_t length;
  if (link == NULL || frame == NULL || link->status.fifo_credit == 0u || link->mutex == NULL) return -1;
  if (osMutexAcquire(link->mutex, osWaitForever) != osOK) return -1;
  length = pack_common(link, FPGA_CMD_FRAME, frame);
  if (length == 0u) { osMutexRelease(link->mutex); return -2; }
  if (exchange(link, length) != 0) { osMutexRelease(link->mutex); return -3; }
  if (unpack_status(link) != 0) { osMutexRelease(link->mutex); return -4; }
  apply_digital(frame);
  link->running = 1u;
  osMutexRelease(link->mutex);
  return 0;
}

int fpga_link_poll_status(fpga_link_t *link)
{
  uint16_t length;
  if (link == NULL || link->mutex == NULL) return -1;
  if (osMutexAcquire(link->mutex, osWaitForever) != osOK) return -1;
  length = pack_common(link, FPGA_CMD_STATUS, NULL);
  if (exchange(link, length) != 0) { osMutexRelease(link->mutex); return -2; }
  (void)length;
  {
    int result = unpack_status(link);
    osMutexRelease(link->mutex);
    return result;
  }
}

int fpga_link_safe_stop(fpga_link_t *link)
{
  uint16_t length;
  if (link == NULL || link->mutex == NULL) return -1;
  if (osMutexAcquire(link->mutex, osWaitForever) != osOK) return -1;
  HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);
  length = pack_common(link, FPGA_CMD_STOP, NULL);
  if (exchange(link, length) != 0) { osMutexRelease(link->mutex); return -2; }
  if (unpack_status(link) != 0) { osMutexRelease(link->mutex); return -3; }
  link->running = 0u;
  osMutexRelease(link->mutex);
  return 0;
}

const fpga_status_wire_t *fpga_link_status(const fpga_link_t *link)
{
  return link != NULL ? &link->status : NULL;
}

void fpga_link_spi_txrx_complete(fpga_link_t *link) { if (link != NULL) link->dma_done = 1u; }
void fpga_link_spi_error(fpga_link_t *link) { if (link != NULL) link->dma_error = 1u; }
