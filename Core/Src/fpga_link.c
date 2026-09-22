#include "fpga_link.h"
#include "main.h"
#include "cmsis_os.h"
#include "system_status.h"
#include <string.h>

uint8_t fpga_logical_to_physical[UMH_DEVICE_CHANNEL_COUNT];

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
  for (uint16_t map_i = 0u; map_i < UMH_DEVICE_CHANNEL_COUNT; ++map_i)
    fpga_logical_to_physical[map_i] = (uint8_t)map_i;
  cs_high();
}

/* Try the DMA fast path first; if it ever times out, fall back permanently to
 * the register loop below and report the switch.  Both paths produce exactly
 * the same wire transaction (CS low for the whole frame, full duplex). */
static uint8_t exchange_use_polling;

static int exchange_dma(fpga_link_t *link, uint16_t length)
{
  SPI_TypeDef *spi = link->spi->Instance;
  DMA_Channel_TypeDef *tx = DMA1_Channel1;
  DMA_Channel_TypeDef *rx = DMA1_Channel2;
  uint32_t start_cycles;
  uint32_t timeout_cycles;
  uint32_t isr;
  if (spi == NULL || tx == NULL || rx == NULL) return -1;
  /* The HAL initialised both DMA channels for SPI1 TX/RX byte streams.  The
   * old code polled TXE/RXNE per byte and inserted a ~50-cycle turn-around
   * gap between bytes, which made a 220-byte frame take ~150 us instead of
   * the 83 us allowed by the 21.25 MHz wire rate.  Channel-config registers
   * (DMAMUX request, direction, increments, size) are already programmed. */
  spi->CR1 |= SPI_CR1_SPE;
  spi->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
  (void)spi->SR;
  (void)*(__IO uint8_t *)&spi->DR;

  tx->CCR &= ~DMA_CCR_EN;
  rx->CCR &= ~DMA_CCR_EN;
  DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;
  tx->CPAR = (uint32_t)&spi->DR;
  tx->CMAR = (uint32_t)link->tx;
  tx->CNDTR = length;
  rx->CPAR = (uint32_t)&spi->DR;
  rx->CMAR = (uint32_t)link->rx;
  rx->CNDTR = length;

  cs_low();
  tx->CCR |= DMA_CCR_EN;
  rx->CCR |= DMA_CCR_EN;

  start_cycles = DWT->CYCCNT;
  timeout_cycles = SystemCoreClock / 25u; /* 40 ms, same order as the old path */
  if (timeout_cycles == 0u) timeout_cycles = 8000000u;
  for (;;) {
    isr = DMA1->ISR;
    if ((isr & (DMA_ISR_TEIF1 | DMA_ISR_TEIF2)) != 0u) {
      break;
    }
    if ((isr & (DMA_ISR_TCIF1 | DMA_ISR_TCIF2)) == (DMA_ISR_TCIF1 | DMA_ISR_TCIF2)) {
      break;
    }
    if ((DWT->CYCCNT - start_cycles) > timeout_cycles) {
      isr = 0u;
      break;
    }
  }
  tx->CCR &= ~DMA_CCR_EN;
  rx->CCR &= ~DMA_CCR_EN;
  DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;
  {
    uint32_t guard_cycles = SystemCoreClock / 1000u; /* 1 ms */
    uint32_t guard_start = DWT->CYCCNT;
    while ((spi->SR & SPI_SR_BSY) != 0u) {
      if ((DWT->CYCCNT - guard_start) > guard_cycles) return -3;
    }
  }
  cs_high();
  if ((isr & (DMA_ISR_TCIF1 | DMA_ISR_TCIF2)) != (DMA_ISR_TCIF1 | DMA_ISR_TCIF2)) {
    mark_link_fault(link);
    return -3;
  }
  return 0;
}

static int exchange(fpga_link_t *link, uint16_t length)
{
  static uint32_t last_report;
  SPI_TypeDef *spi;
  uint32_t start_cycles;
  uint32_t timeout_cycles;
  uint16_t i;
  if (link == NULL || link->spi == NULL || length == 0u || length > FPGA_TX_BUFFER_SIZE) return -1;
  spi = link->spi->Instance;
  if (spi == NULL) return -1;
  link->tx_length = length;
  if (exchange_use_polling == 0u) {
    int dma_result = exchange_dma(link, length);
    if (dma_result == 0) return 0;
    exchange_use_polling = 1u;
    system_status_fault(UMH_FAULT_FPGA_SPI_TIMEOUT, 21u, UMH_FAULT_WARNING);
  }

  /* HAL_SPI_Init() leaves SPE cleared; the HAL transfer functions enable it
   * lazily.  This direct path must do the same on its first use. */
  spi->CR1 |= SPI_CR1_SPE;

  /* Direct register service keeps the SPI shift register continuously fed.
   * HAL_SPI_TransmitReceive() restarts its timeout bookkeeping for every byte,
   * which made a 204-byte frame take ~200 us on this 170 MHz part instead of
   * the 77 us required by the 21.25 MHz wire rate.  At 4800 frames/s that
   * difference was the entire ULM timing budget. */
  (void)spi->SR;
  (void)*(__IO uint8_t *)&spi->DR;
  cs_low();
  start_cycles = DWT->CYCCNT;
  timeout_cycles = SystemCoreClock / 50u; /* 20 ms */
  if (timeout_cycles == 0u) timeout_cycles = 4000000u;
  for (i = 0u; i < length; ++i) {
    while ((spi->SR & SPI_SR_TXE) == 0u) {
      if ((DWT->CYCCNT - start_cycles) > timeout_cycles) goto exchange_timeout;
    }
    *(__IO uint8_t *)&spi->DR = link->tx[i];
    while ((spi->SR & SPI_SR_RXNE) == 0u) {
      if ((DWT->CYCCNT - start_cycles) > timeout_cycles) goto exchange_timeout;
    }
    link->rx[i] = *(__IO uint8_t *)&spi->DR;
  }
  while ((spi->SR & SPI_SR_BSY) != 0u) {
    if ((DWT->CYCCNT - start_cycles) > timeout_cycles) goto exchange_timeout;
  }
  cs_high();
  return 0;

exchange_timeout:
  cs_high();
  mark_link_fault(link);
  /* A missing FPGA clock/MISO must not turn the render task into a fault
   * storm.  Report at most once per second. */
  if ((HAL_GetTick() - last_report) >= 1000u) {
    last_report = HAL_GetTick();
    system_status_fault(UMH_FAULT_FPGA_SPI_TIMEOUT, 20u, UMH_FAULT_CRITICAL);
  }
  return -3;
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
    /* Logical/protocol channel i is element E(i+1).  The FPGA output bit
     * order follows the PCB netlist, so serialization applies the fixed
     * E## -> us_tx bit permutation here, once, for every producer. */
    uint16_t channel_base = pos;
    pos = (uint16_t)(pos + 2u * UMH_DEVICE_CHANNEL_COUNT);
    for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) {
      uint8_t physical = fpga_logical_to_physical[i];
      link->tx[channel_base + 2u * (uint16_t)physical] = frame->channels[i].phase;
      link->tx[channel_base + 2u * (uint16_t)physical + 1u] = frame->channels[i].level;
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

int fpga_link_set_ws2812(fpga_link_t *link, uint8_t r, uint8_t g, uint8_t b)
{
  uint16_t length;
  umh_output_frame_t frame;
  if (link == NULL || link->mutex == NULL) return -1;
  if (osMutexAcquire(link->mutex, osWaitForever) != osOK) return -1;

  memset(&frame, 0, sizeof(frame));
  frame.update_flags = UMH_FRAME_FLAG_RGB;
  frame.rgb[0].red = r;    frame.rgb[0].green = g;    frame.rgb[0].blue = b;  /* LED 0 */
  frame.rgb[1].red = r;    frame.rgb[1].green = g;    frame.rgb[1].blue = b;  /* LED 1 */
  frame.rgb[2].red = r;    frame.rgb[2].green = g;    frame.rgb[2].blue = b;  /* LED 2 */
  frame.rgb[3].red = r;    frame.rgb[3].green = g;    frame.rgb[3].blue = b;  /* LED 3 */

  length = pack_common(link, FPGA_CMD_WS2812, &frame);
  if (length == 0u) { osMutexRelease(link->mutex); return -2; }
  if (exchange(link, length) != 0) { osMutexRelease(link->mutex); return -3; }
  if (unpack_status(link) != 0) { osMutexRelease(link->mutex); return -4; }

  osMutexRelease(link->mutex);
  return 0;
}

static int compact_audio_command(fpga_link_t *link, uint8_t command,
                                  uint8_t data, uint32_t sequence)
{
  int result = 0;
  if (link == NULL || link->mutex == NULL) return -1;
  if (osMutexAcquire(link->mutex, osWaitForever) != osOK) return -1;
  memset(link->tx, 0, 16u);
  link->tx[0] = command;
  link->tx[1] = FPGA_PROTOCOL_VERSION;
  link->tx[2] = data;
  put_u32(&link->tx[6], sequence);
  if (exchange(link, 16u) != 0) result = -2;
  else if (unpack_status(link) != 0) result = -3;
  osMutexRelease(link->mutex);
  return result;
}

static int wait_for_credit(fpga_link_t *link)
{
  uint8_t attempt;
  for (attempt = 0u; attempt < 20u; ++attempt) {
    if (fpga_link_poll_status(link) == 0 && link->status.fifo_credit != 0u)
      return 0;
    osDelay(1u);
  }
  return -1;
}

int fpga_link_audio_begin(fpga_link_t *link, const uint8_t *phases,
                          const uint8_t *enables, uint32_t sequence)
{
  umh_output_frame_t frame;
  uint16_t i;
  int result;
  if (link == NULL || phases == NULL || enables == NULL) return -1;
  /* Three safe steps:
   *   1. submit one silent full frame so the staging RAM and old enable
   *      markers cannot leak into the first audio table;
   *   2. enter audio mode at common level 0 (the FPGA still holds phases
   *      from step 1 and rebuilds a silent table);
   *   3. submit the real aperture phases plus enable markers while audio
   *      mode is active, so the rebuild still runs at common level 0.
   * Audio level commands then only exchange 16 bytes each. */
  (void)fpga_link_safe_stop(link);
  if (wait_for_credit(link) != 0) return -1;
  memset(&frame, 0, sizeof(frame));
  frame.sequence = sequence;
  frame.update_flags = UMH_FRAME_FLAG_ULTRASOUND;
  for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) {
    frame.channels[i].phase = phases[i];
    frame.channels[i].level = 0u;
  }
  result = fpga_link_submit(link, &frame);
  if (result != 0) return result;

  result = fpga_link_audio_mode(link, 1u, sequence + 1u);
  if (result != 0) {
    (void)fpga_link_safe_stop(link);
    return result;
  }
  link->audio_short_supported =
      (link->status.status_flags & FPGA_STATUS_AUDIO_SHORT) != 0u ? 1u : 0u;
  /* Entering audio mode starts one silent rebuild.  Wait until that builder
   * has released FIFO credit again before loading the enable markers. */
  if (wait_for_credit(link) != 0) {
    (void)fpga_link_audio_mode(link, 0u, sequence + 3u);
    (void)fpga_link_safe_stop(link);
    return -1;
  }

  frame.sequence = sequence + 2u;
  for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) {
    /* The staging low byte doubles as the enable mask in focused-AM mode.
     * A non-zero marker keeps the channel active at the common envelope
     * level; zero mutes exactly the channels that spatial rendering would
     * have disabled. */
    frame.channels[i].level = enables[i] != 0u ? 255u : 0u;
  }
  result = fpga_link_submit(link, &frame);
  if (result != 0) {
    (void)fpga_link_audio_mode(link, 0u, sequence + 3u);
    (void)fpga_link_safe_stop(link);
    return result;
  }
  return 0;
}

int fpga_link_audio_level(fpga_link_t *link, uint8_t level, uint32_t sequence)
{
  return compact_audio_command(link, FPGA_CMD_AUDIO_LEVEL, level, sequence);
}

int fpga_link_audio_level_fast(fpga_link_t *link, uint8_t level)
{
  if (link == NULL || link->spi == NULL) return -1;
  link->tx[0] = FPGA_CMD_AUDIO_LEVEL_SHORT;
  link->tx[1] = FPGA_PROTOCOL_VERSION;
  link->tx[2] = level;
  if (exchange(link, 3u) != 0) return -2;
  return 0;
}

int fpga_link_audio_mode(fpga_link_t *link, uint8_t enable, uint32_t sequence)
{
  uint8_t attempt;
  int result = compact_audio_command(link, FPGA_CMD_AUDIO_MODE,
                                     enable != 0u ? 1u : 0u, sequence);
  if (result != 0 || enable == 0u) return result;
  /* The status word in the compact transaction is latched before CS falls,
   * so poll until the new bitstream confirms that AUDIO_MODE=1 was adopted.
   * Old FPGA images silently ignore the compact command; without this check
   * the following silent FRAME could be built with the 0/255 enable marker
   * mistaken for a real level. */
  for (attempt = 0u; attempt < 20u; ++attempt) {
    if (fpga_link_poll_status(link) == 0 &&
        (link->status.status_flags & FPGA_STATUS_AUDIO_MODE) != 0u)
      return 0;
    osDelay(1u);
  }
  return -4;
}

static int mic_command(fpga_link_t *link, uint8_t command, uint32_t sequence,
                       const uint8_t *extension, uint16_t extension_length,
                       uint8_t *response, uint16_t response_length)
{
  umh_output_frame_t frame;
  uint16_t length;
  int result = 0;
  if (link == NULL || link->mutex == NULL) return -1;
  if (extension_length > sizeof(frame.extension)) return -2;
  if (osMutexAcquire(link->mutex, osWaitForever) != osOK) return -1;
  memset(&frame, 0, sizeof(frame));
  frame.update_flags = UMH_FRAME_FLAG_EXTENDED;
  frame.sequence = sequence;
  frame.extension_length = extension_length;
  if (extension != NULL && extension_length != 0u)
    memcpy(frame.extension, extension, extension_length);
  length = pack_common(link, command, &frame);
  if (length == 0u) result = -2;
  else {
    if (response_length > length) {
      uint16_t pad = (uint16_t)(response_length - length);
      if (pad > (uint16_t)(FPGA_TX_BUFFER_SIZE - length)) result = -2;
      else {
        memset(&link->tx[length], 0, pad);
        length = response_length;
      }
    }
    if (result == 0 && length > 0u) {
      if (exchange(link, length) != 0) result = -3;
      else if (unpack_status(link) != 0) result = -4;
      else if (response != NULL && response_length != 0u)
        memcpy(response, link->rx, response_length);
    }
  }
  osMutexRelease(link->mutex);
  return result;
}

static uint16_t get_be16(const uint8_t *p)
{
  return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

int fpga_link_mic_config(fpga_link_t *link, uint8_t gate_count, uint16_t start,
                         uint16_t step, uint8_t width)
{
  uint8_t ext[6];
  ext[0] = gate_count;
  ext[1] = (uint8_t)start;
  ext[2] = (uint8_t)(start >> 8);
  ext[3] = (uint8_t)step;
  ext[4] = (uint8_t)(step >> 8);
  ext[5] = width;
  return mic_command(link, FPGA_CMD_MIC_CONFIG, 0u, ext, sizeof(ext), NULL, 0u);
}

int fpga_link_mic_read(fpga_link_t *link, uint8_t gate, fpga_mic_gate_wire_t *result)
{
  uint8_t rx[FPGA_MIC_READ_BYTES];
  uint8_t k;
  int rc;
  if (result == NULL) return -2;
  rc = mic_command(link, FPGA_CMD_MIC_READ, (uint32_t)gate, NULL, 0u,
                   rx, sizeof(rx));
  if (rc != 0) return rc;
  result->status = get_be16(&rx[16]);
  result->block_count = get_be16(&rx[18]);
  result->gate_count = get_be16(&rx[20]);
  result->reserved = get_be16(&rx[22]);
  for (k = 0u; k < UMH_DEVICE_MIC_COUNT; ++k) {
    result->i[k] = (int16_t)get_be16(&rx[24u + 2u * k]);
    result->q[k] = (int16_t)get_be16(&rx[32u + 2u * k]);
  }
  return 0;
}
const fpga_status_wire_t *fpga_link_status(const fpga_link_t *link)
{
  return link != NULL ? &link->status : NULL;
}

void fpga_link_spi_txrx_complete(fpga_link_t *link) { if (link != NULL) link->dma_done = 1u; }
void fpga_link_spi_error(fpga_link_t *link) { if (link != NULL) link->dma_error = 1u; }
