#include "fpga_link.h"
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "system_status.h"
#include "umh_utils.h"
#include <string.h>

uint8_t fpga_logical_to_physical[UMH_DEVICE_CHANNEL_COUNT];
static uint8_t fpga_channel_map_identity = 1u;

static void cs_low(void) { HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_RESET); }
static void cs_high(void) { HAL_GPIO_WritePin(FPGA_CS_GPIO_Port, FPGA_CS_Pin, GPIO_PIN_SET); }

static void mark_link_fault(fpga_link_t *link)
{
  if (link == NULL) return;
  link->running = 0u;
  link->levels_valid = 0u;
  link->hold_valid = 0u;
  link->status.status_flags |= FPGA_STATUS_OUTPUT_FAULT;
}

/* Use unified serialization helpers from umh_utils.h */
#define put_u16(p, v)  umh_write_u16_le(p, v)
#define put_u32(p, v)  umh_write_u32_le(p, v)
#define put_u64(p, v)  umh_write_u64_le(p, v)
#define get_u16(p)     umh_read_u16_le(p)
#define get_u32(p)     umh_read_u32_le(p)

static void spi_dma_blocking_init(void);
static uint16_t pack_header(fpga_link_t *link, uint8_t *tx, uint8_t command,
                            const umh_output_frame_t *frame,
                            uint16_t extension_length);
static int link_complete_deferred(fpga_link_t *link);
static SPI_TypeDef *link_spi_hw;

void fpga_link_init(fpga_link_t *link, SPI_HandleTypeDef *spi)
{
  osMutexAttr_t mutex_attributes;
  if (link == NULL) return;
  memset(link, 0, sizeof(*link));
  link->levels_valid = 0u;
  link->hold_valid = 0u;
  link->spi = spi;
  memset(&mutex_attributes, 0, sizeof(mutex_attributes));
  mutex_attributes.name = "fpga-link";
  mutex_attributes.cb_mem = &link->mutex_memory;
  mutex_attributes.cb_size = sizeof(link->mutex_memory);
  link->mutex = osMutexNew(&mutex_attributes);
  link_spi_hw = spi != NULL ? spi->Instance : NULL;
  link->status.protocol_version = FPGA_PROTOCOL_VERSION;
  link->status.fifo_credit = FPGA_FIFO_DEFAULT_CREDIT;
  for (uint16_t map_i = 0u; map_i < UMH_DEVICE_CHANNEL_COUNT; ++map_i)
    fpga_logical_to_physical[map_i] = (uint8_t)map_i;
  fpga_channel_map_identity = 1u;
  {
    umh_output_frame_t phase_frame;
    memset(&phase_frame, 0, sizeof(phase_frame));
    phase_frame.update_flags = UMH_FRAME_FLAG_ULTRASOUND;
    (void)pack_header(link, link->tx, FPGA_CMD_FRAME_PHASE, &phase_frame, 0u);
    memcpy(link->phase_header, link->tx, FPGA_PHASE_HEADER_BYTES);
    link->transaction_sequence = 0u;
  }
  cs_high();
  spi_dma_blocking_init();
}

/* Calibration runs own the FPGA link and do not need the normal 21.25 MHz
 * control throughput.  The microphone EBR read path (and the 84-channel
 * FRAME/MIC_CONFIG setup used with it) has a marginal first-bit setup window
 * at 21.25 MHz: a single MSB sampling error turns small signed I/Q values
 * into near-full-scale numbers and makes the phase solver fail.  Calibration
 * switches the whole link to /32 for the duration of the run, then restores
 * the exact previous CR1 value so motion/stream throughput is untouched. */
uint32_t fpga_link_calibration_link_begin(fpga_link_t *link)
{
  uint32_t saved_cr1 = 0u;
  SPI_TypeDef *spi;
  if (link == NULL || link->spi == NULL || link->spi->Instance == NULL ||
      link->mutex == NULL)
    return 0u;
  spi = link->spi->Instance;
  if (osMutexAcquire(link->mutex, osWaitForever) != osOK) return 0u;
  /* The clock divider must not change under a frame that is still shifting out. */
  (void)link_complete_deferred(link);
  saved_cr1 = spi->CR1;
  spi->CR1 = (saved_cr1 & ~(SPI_CR1_BR | SPI_CR1_SPE)) |
             SPI_BAUDRATEPRESCALER_32;
  osMutexRelease(link->mutex);
  return saved_cr1;
}

void fpga_link_calibration_link_end(fpga_link_t *link, uint32_t saved_cr1)
{
  if (link == NULL || link->spi == NULL || link->spi->Instance == NULL ||
      link->mutex == NULL || saved_cr1 == 0u)
    return;
  if (osMutexAcquire(link->mutex, osWaitForever) != osOK) return;
  (void)link_complete_deferred(link);
  link->spi->Instance->CR1 = saved_cr1;
  osMutexRelease(link->mutex);
}


static uint8_t exchange_use_polling;
static uint8_t exchange_dma_failures;
#define FPGA_SPI_DMA_FAILURES_LATCH 3u

/* 2 ms bounds every SPI frame: the longest legal frame (256 B at the /32
 * calibration clock) takes 0.39 ms, and the FPGA link watchdog fires after
 * ~30 ms without a CS edge. */
#define FPGA_SPI_TIMEOUT_CYCLES (SystemCoreClock / 500u)

/* Deferred frame transmit.  A frame transaction is armed and left to the DMA
 * engine; the RX transfer-complete interrupt only deasserts CS (the FPGA needs
 * that edge to close the frame) and never wakes a task.  The next link
 * operation, whichever task issues it, reaps the result before touching the
 * bus.  At the demo's 4800 fps the 22.6 us wire time is overlapped with the
 * caller's own work instead of being blocked out and then paid again as a
 * 6.4 us wake plus a scheduler round trip. */
#define FPGA_XFER_NONE  0u
#define FPGA_XFER_PHASE 1u
#define FPGA_XFER_FULL  2u

static volatile uint8_t xfer_cs_isr;
static volatile uint8_t xfer_block_wait;
static volatile uint8_t xfer_done;
static volatile uint8_t xfer_err;

/* Drop RX bytes an aborted frame left in the FIFO and clear OVR (DR reads
 * followed by an SR read), so every frame starts byte-aligned. */
static void spi_flush_rx(SPI_TypeDef *spi)
{
  while ((spi->SR & SPI_SR_FRLVL) != 0u) (void)*(__IO uint8_t *)&spi->DR;
}

extern DMA_HandleTypeDef hdma_spi1_rx;

static SemaphoreHandle_t spi_dma_sem;
static StaticSemaphore_t spi_dma_sem_mem;
static uint8_t spi_dma_blocking_ready;

/* RX transfer-complete: the last byte has been clocked in, so the frame is
 * over from the FPGA's point of view.  Only the deferred path lets this
 * interrupt own CS; command transactions deselect in the caller, exactly as
 * before, so their byte timing is unchanged. */
static void spi_dma_complete_isr(DMA_HandleTypeDef *hdma)
{
  BaseType_t woken = pdFALSE;
  volatile uint8_t defer = xfer_cs_isr;
  (void)hdma;
  xfer_done = 1u;
  if (defer != 0u) {
    /* CS must not rise before the final MOSI bit has left the shifter, and BSY
     * clears within one SCK period of the last edge; the bound keeps a stuck
     * BSY from hanging the interrupt. */
    uint32_t start = DWT->CYCCNT;
    if (link_spi_hw != NULL) {
      while ((link_spi_hw->SR & SPI_SR_BSY) != 0u &&
             (DWT->CYCCNT - start) <= 400u) {
      }
    }
    cs_high();
    xfer_cs_isr = 0u;
  }
  if (xfer_block_wait != 0u && spi_dma_sem != NULL) {
    (void)xSemaphoreGiveFromISR(spi_dma_sem, &woken);
    portYIELD_FROM_ISR(woken);
  }
}

static void spi_dma_error_isr(DMA_HandleTypeDef *hdma)
{
  BaseType_t woken = pdFALSE;
  (void)hdma;
  xfer_err = 1u;
  if (xfer_cs_isr != 0u) {
    cs_high();
    xfer_cs_isr = 0u;
  }
  if (xfer_block_wait != 0u && spi_dma_sem != NULL) {
    (void)xSemaphoreGiveFromISR(spi_dma_sem, &woken);
    portYIELD_FROM_ISR(woken);
  }
}

static void spi_dma_blocking_init(void)
{
  if (spi_dma_blocking_ready != 0u) return;
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) return;
  if (spi_dma_sem == NULL) spi_dma_sem = xSemaphoreCreateBinaryStatic(&spi_dma_sem_mem);
  if (spi_dma_sem == NULL) return;
  hdma_spi1_rx.XferCpltCallback = spi_dma_complete_isr;
  hdma_spi1_rx.XferErrorCallback = spi_dma_error_isr;
  spi_dma_blocking_ready = 1u;
}

/* Result of one armed transaction: 0 not finished, 1 finished, 2 failed.
 *
 * The RX transfer-complete interrupt runs HAL_DMA_IRQHandler, and that handler
 * clears TCIF2 itself, so the raw TC flag pair cannot be the completion source
 * once the handler has run - the ISR latches xfer_done instead.  CNDTR still
 * discriminates: a latch left behind by an interrupt that was pending while the
 * next transfer was armed reads CNDTR != 0 (just reloaded) and is ignored. */
static uint8_t xfer_state(void)
{
  uint32_t isr = DMA1->ISR;
  if (xfer_err != 0u) return 2u;
  if ((isr & (DMA_ISR_TEIF1 | DMA_ISR_TEIF2)) != 0u) return 2u;
  if (DMA1_Channel2->CNDTR != 0u) return 0u;
  if (xfer_done != 0u) return 1u;
  if ((isr & (DMA_ISR_TCIF1 | DMA_ISR_TCIF2)) == (DMA_ISR_TCIF1 | DMA_ISR_TCIF2)) return 1u;
  return 0u;
}

/* Arm one byte-stream transaction.  The HAL already programmed the channel
 * configuration (DMAMUX request, direction, increments, size); only the
 * address/COUNT pair and the enables change per transfer.  RX is enabled before
 * TX starts the clock: with TX first, an ISR landing between the two enables
 * let the 4-byte RX FIFO overrun, and OVR then stalled RX DMA for the whole
 * timeout with CS low, long enough for the FPGA link watchdog to drop
 * MOD_FIXED. */
static int exchange_arm(fpga_link_t *link, const uint8_t *tx, uint16_t length,
                        uint8_t defer_cs)
{
  SPI_TypeDef *spi = link->spi->Instance;
  DMA_Channel_TypeDef *txch = DMA1_Channel1;
  DMA_Channel_TypeDef *rxch = DMA1_Channel2;
  if (spi == NULL) return -1;
  spi->CR1 |= SPI_CR1_SPE;
  spi->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
  spi_flush_rx(spi);

  txch->CCR &= ~DMA_CCR_EN;
  rxch->CCR &= ~DMA_CCR_EN;
  DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;
  txch->CPAR = (uint32_t)&spi->DR;
  txch->CMAR = (uint32_t)tx;
  txch->CNDTR = length;
  rxch->CPAR = (uint32_t)&spi->DR;
  rxch->CMAR = (uint32_t)link->rx;
  rxch->CNDTR = length;

  /* A deferred transfer never parks a task on the semaphore, so it can never
   * leave a stale token behind and does not have to clear one here. */
  if (spi_dma_blocking_ready != 0u && defer_cs == 0u) (void)xSemaphoreTake(spi_dma_sem, 0);
  rxch->CCR |= DMA_CCR_TCIE | DMA_CCR_TEIE;
  cs_low();
  xfer_cs_isr = defer_cs;
  /* Latched results are cleared with interrupts masked so a completion from the
   * previous transfer can never be read as this one's. */
  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    xfer_done = 0u;
    xfer_err = 0u;
    __set_PRIMASK(primask);
  }
  rxch->CCR |= DMA_CCR_EN;
  txch->CCR |= DMA_CCR_EN;
  return 0;
}

/* Complete the armed transaction.  block == 0 is the deferred path: the frame
 * has almost always finished while the caller was busy, so this is a latch read
 * plus a few register writes. */
static int exchange_wait(fpga_link_t *link, uint8_t block)
{
  SPI_TypeDef *spi = link->spi->Instance;
  uint32_t start_cycles;
  uint8_t state;
  if (spi == NULL) return -1;
  state = xfer_state();
  if (state == 0u) {
    if (block != 0u && spi_dma_blocking_ready != 0u) {
      xfer_block_wait = 1u;
      state = xfer_state();
      if (state == 0u) (void)xSemaphoreTake(spi_dma_sem, pdMS_TO_TICKS(2u));
      xfer_block_wait = 0u;
    } else {
      start_cycles = DWT->CYCCNT;
      while ((state = xfer_state()) == 0u &&
             (DWT->CYCCNT - start_cycles) <= FPGA_SPI_TIMEOUT_CYCLES) {
      }
    }
    state = xfer_state();
  }
  DMA1_Channel1->CCR &= ~DMA_CCR_EN;
  DMA1_Channel2->CCR &= ~DMA_CCR_EN;
  DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;
  start_cycles = DWT->CYCCNT;
  while ((spi->SR & SPI_SR_BSY) != 0u &&
         (DWT->CYCCNT - start_cycles) <= FPGA_SPI_TIMEOUT_CYCLES) {
  }
  xfer_cs_isr = 0u;
  /* CS returns high on every path so the FPGA always sees the frame end. */
  cs_high();
  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    xfer_done = 0u;
    xfer_err = 0u;
    __set_PRIMASK(primask);
  }
  return state == 1u ? 0 : -3;
}

/* Transfers at or below this length use the register path.  Arming two DMA
 * channels costs several register writes, a clear and a poll loop, which is
 * more than the transfer itself for the 2-byte FIFO fill polls that
 * aperture_mod_service() issues many times per millisecond. */
#define FPGA_SPI_POLL_MAX_BYTES 17u

/* Byte-at-a-time fallback.  It keeps the SPI shift register continuously fed:
 * HAL_SPI_TransmitReceive() restarts its timeout bookkeeping for every byte,
 * which made a 204-byte frame take ~200 us on this 170 MHz part instead of the
 * 77 us allowed by the wire rate.  TXE only means the transmit buffer is empty,
 * so RXNE is checked before DR is read; without that the first status byte of
 * the 16-byte MOD_FIXED transaction reads back 0. */
static int exchange_register(fpga_link_t *link, const uint8_t *tx, uint16_t length)
{
  static uint32_t last_report;
  SPI_TypeDef *spi = link->spi->Instance;
  uint32_t start_cycles;
  uint32_t timeout_cycles;
  uint16_t i;
  if (spi == NULL) return -1;
  spi->CR1 |= SPI_CR1_SPE;
  spi_flush_rx(spi);
  cs_low();
  start_cycles = DWT->CYCCNT;
  timeout_cycles = FPGA_SPI_TIMEOUT_CYCLES;
  for (i = 0u; i < length; ++i) {
    while ((spi->SR & SPI_SR_TXE) == 0u) {
      if ((DWT->CYCCNT - start_cycles) > timeout_cycles) goto exchange_timeout;
    }
    *(__IO uint8_t *)&spi->DR = tx[i];
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

static int exchange(fpga_link_t *link, uint16_t length)
{
  if (link == NULL || link->spi == NULL || length == 0u || length > FPGA_TX_BUFFER_SIZE) return -1;
  link->tx_length = length;
  if (exchange_use_polling == 0u && length > FPGA_SPI_POLL_MAX_BYTES) {
    if (exchange_arm(link, link->tx, length, 0u) == 0 && exchange_wait(link, 1u) == 0) {
      exchange_dma_failures = 0u;
      return 0;
    }
    link->levels_valid = 0u;
    link->hold_valid = 0u;
    if (++exchange_dma_failures >= FPGA_SPI_DMA_FAILURES_LATCH) {
      exchange_use_polling = 1u;
      system_status_fault(UMH_FAULT_FPGA_SPI_TIMEOUT, 21u, UMH_FAULT_WARNING);
    }
  }
  return exchange_register(link, link->tx, length);
}

static uint16_t pack_header(fpga_link_t *link, uint8_t *tx, uint8_t command,
                            const umh_output_frame_t *frame,
                            uint16_t extension_length)
{
  uint16_t pos = 0u;
  uint16_t update = frame != NULL ? frame->update_flags : 0u;
  uint8_t mask[UMH_DEVICE_CHANNEL_BITMAP_BYTES] = {0};
  tx[pos++] = command;
  tx[pos++] = FPGA_PROTOCOL_VERSION;
  put_u32(&tx[pos], link->transaction_sequence++); pos += 4u;
  put_u32(&tx[pos], frame != NULL ? frame->sequence : 0u); pos += 4u;
  put_u64(&tx[pos], frame != NULL ? frame->deadline : 0u); pos += 8u;
  put_u16(&tx[pos], update); pos += 2u;
  if ((update & UMH_FRAME_FLAG_ULTRASOUND) != 0u) {
    memset(mask, 0xFF, sizeof(mask));
    mask[sizeof(mask) - 1u] = FPGA_ULTRASOUND_BITMAP_LAST_MASK;
  }
  memcpy(&tx[pos], mask, sizeof(mask)); pos += sizeof(mask);
  tx[pos++] = (update & UMH_FRAME_FLAG_RGB) != 0u ? 0x0Fu : 0u;
  /* The digital byte pair stays in the wire frame for protocol compatibility,
   * but it has no consumer on either side any more: the FPGA ignores it and
   * PA9, the one pin it used to drive, is the trigger input. */
  tx[pos++] = frame != NULL ? frame->digital_mask : 0u;
  tx[pos++] = frame != NULL ? frame->digital_state : 0u;
  put_u16(&tx[pos], extension_length); pos += 2u;
  return pos;
}

static uint16_t pack_common(fpga_link_t *link, uint8_t *tx, uint8_t command,
                            const umh_output_frame_t *frame)
{
  uint16_t pos;
  uint16_t i;
  uint16_t extension_length = 0u;
  uint8_t update = frame != NULL ? (uint8_t)frame->update_flags : 0u;
  if (link == NULL) return 0u;
  if (frame != NULL && (update & UMH_FRAME_FLAG_EXTENDED) != 0u) {
    extension_length = frame->extension_length;
    if (extension_length > sizeof(frame->extension)) return 0u;
  }
  pos = pack_header(link, tx, command, frame, extension_length);
  if (frame != NULL && (update & UMH_FRAME_FLAG_ULTRASOUND) != 0u) {
    /* Logical/protocol channel i is element E(i+1).  The FPGA output bit
     * order follows the PCB netlist, so serialization applies the fixed
     * E## -> us_tx bit permutation here, once, for every producer. */
    uint16_t channel_base = pos;
    pos = (uint16_t)(pos + 2u * UMH_DEVICE_CHANNEL_COUNT);
    if (fpga_channel_map_identity != 0u) {
      memcpy(&tx[channel_base], frame->channels, sizeof(frame->channels));
    } else {
      for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) {
        uint8_t physical = fpga_logical_to_physical[i];
        tx[channel_base + 2u * (uint16_t)physical] = frame->channels[i].phase;
        tx[channel_base + 2u * (uint16_t)physical + 1u] = frame->channels[i].level;
      }
    }
  }
  if (frame != NULL && (update & UMH_FRAME_FLAG_RGB) != 0u) {
    memcpy(&tx[pos], frame->rgb, sizeof(frame->rgb));
    pos = (uint16_t)(pos + sizeof(frame->rgb));
  }
  if (extension_length != 0u) {
    memcpy(&tx[pos], frame->extension, extension_length);
    pos = (uint16_t)(pos + extension_length);
  }
  if (pos > FPGA_TX_BUFFER_SIZE) return 0u;
  return pos;
}

/* Pack the 84 phases of a phase-only frame.  Four channels are de-interleaved
 * per iteration: the channel array is {phase, level} pairs, so a 32-bit load
 * holds two channels and UXTB16 splits the phase bytes out of two loads at once.
 * The "levels unchanged" test rides on the same loads: the previous channel
 * words are kept, so masking out the level bytes and comparing decides PHASE_ONLY
 * eligibility in two instructions per four channels. */
static uint8_t pack_phase_fused(fpga_link_t *link, const umh_output_frame_t *frame,
                                uint8_t *tx)
{
  uint16_t i;
  uint8_t *out = &tx[FPGA_PHASE_HEADER_BYTES];
  if (fpga_channel_map_identity == 0u) {
    for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) {
      uint32_t word;
      memcpy(&word, &frame->channels[i & (uint16_t)~1u], sizeof(word));
      if (frame->channels[i].level !=
          (uint8_t)(word >> ((i & 1u) != 0u ? 24u : 8u)))
        return 0u;
    }
    for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i)
      out[fpga_logical_to_physical[i]] = frame->channels[i].phase;
    memcpy(link->prev_channels, frame->channels, sizeof(link->prev_channels));
    return 1u;
  }
  for (i = 0u; i < (uint16_t)(UMH_DEVICE_CHANNEL_COUNT / 4u); ++i) {
    uint32_t a, b, la, lb, phases;
    memcpy(&a, &frame->channels[4u * i], sizeof(a));
    memcpy(&b, &frame->channels[4u * i + 2u], sizeof(b));
    if (((a ^ link->prev_channels[2u * i]) & 0xFF00FF00u) != 0u ||
        ((b ^ link->prev_channels[2u * i + 1u]) & 0xFF00FF00u) != 0u)
      return 0u;
    la = __UXTB16(a);
    lb = __UXTB16(b);
    phases = (la & 0x000000FFu) | ((la >> 8) & 0x0000FF00u) |
             ((lb << 16) & 0x00FF0000u) | ((lb << 8) & 0xFF000000u);
    memcpy(&out[4u * i], &phases, sizeof(phases));
    memcpy(&link->prev_channels[2u * i], &a, sizeof(a));
    memcpy(&link->prev_channels[2u * i + 1u], &b, sizeof(b));
  }
  return 1u;
}

static void phase_header_patch(fpga_link_t *link, const umh_output_frame_t *frame,
                               uint8_t *tx)
{
  uint32_t transaction = link->transaction_sequence++;
  memcpy(tx, link->phase_header, FPGA_PHASE_HEADER_BYTES);
  memcpy(&tx[2], &transaction, sizeof(transaction));
  memcpy(&tx[6], &frame->sequence, sizeof(frame->sequence));
  memcpy(&tx[10], &frame->deadline, sizeof(frame->deadline));
  tx[32] = frame->digital_mask;
  tx[33] = frame->digital_state;
}

static uint8_t phase_only_eligible(const fpga_link_t *link,
                                   const umh_output_frame_t *frame)
{
  if (link->levels_valid == 0u) return 0u;
  if ((frame->update_flags & UMH_FRAME_FLAG_ULTRASOUND) == 0u) return 0u;
  if ((frame->update_flags & ~UMH_FRAME_FLAG_ULTRASOUND) != 0u) return 0u;
  if ((link->status.status_flags & FPGA_STATUS_PHASE_ONLY) == 0u) return 0u;
  return 1u;
}

static uint8_t hold_match(const fpga_link_t *link, const umh_output_frame_t *frame)
{
  uint16_t i;
  if (link->hold_valid == 0u) return 0u;
  if ((link->hold_flags & UMH_FRAME_FLAG_ULTRASOUND) == 0u) return 0u;
  if ((frame->update_flags &
       (UMH_FRAME_FLAG_DIGITAL | UMH_FRAME_FLAG_EXTENDED)) != 0u)
    return 0u;
  for (i = 0u; i < (uint16_t)(UMH_DEVICE_CHANNEL_COUNT / 2u); ++i) {
    uint32_t word;
    memcpy(&word, &frame->channels[2u * i], sizeof(word));
    if (word != link->prev_channels[i]) return 0u;
  }
  if ((frame->update_flags & UMH_FRAME_FLAG_RGB) != 0u &&
      ((link->hold_flags & UMH_FRAME_FLAG_RGB) == 0u ||
       memcmp(link->hold_rgb, frame->rgb, sizeof(link->hold_rgb)) != 0))
    return 0u;
  return 1u;
}

static void cache_levels(fpga_link_t *link, const umh_output_frame_t *frame)
{
  memcpy(link->prev_channels, frame->channels, sizeof(link->prev_channels));
  link->levels_valid = 1u;
}

static int unpack_status(fpga_link_t *link)
{
  if (link == NULL || link->tx_length < sizeof(fpga_status_wire_t)) return -1;
  link->status.protocol_version = link->rx[0];
  link->status.mod_fill = link->rx[1];
  link->status.fifo_credit = get_u16(&link->rx[2]);
  link->status.fifo_depth = get_u16(&link->rx[4]);
  link->status.status_flags = get_u16(&link->rx[6]);
  link->status.fpga_time = get_u32(&link->rx[8]);
  link->status.accepted_sequence = get_u32(&link->rx[12]);
  if (link->status.protocol_version != FPGA_PROTOCOL_VERSION) {
    link->running = 0u;
    link->levels_valid = 0u;
    link->hold_valid = 0u;
    system_status_clear(UMH_SYSTEM_FPGA_READY);
    system_status_set(UMH_SYSTEM_ERROR);
    system_status_fault(UMH_FAULT_FPGA_PROTOCOL, link->status.protocol_version, UMH_FAULT_CRITICAL);
    return -2;
  }
  if ((link->status.status_flags &
       (FPGA_STATUS_PHASE_ONLY | FPGA_STATUS_RUNNING)) !=
      (FPGA_STATUS_PHASE_ONLY | FPGA_STATUS_RUNNING)) {
    link->levels_valid = 0u;
    link->hold_valid = 0u;
  }
  system_status_set(UMH_SYSTEM_FPGA_READY);
  return 0;
}

/* Reap the frame transaction armed by an earlier call.  Every entry point calls
 * this with the link mutex held, so a frame armed by one task is accounted for
 * by whichever task next touches the link.  Consequence: the status returned by
 * a submit describes the previous frame, and a frame's counters/state are only
 * committed once the transfer is known to have completed. */
static int link_complete_deferred(fpga_link_t *link)
{
  uint8_t *buf;
  int result;
  if (link->xfer_pending == 0u) return 0;
  /* Above ~40 kHz the next frame can arrive while the previous one is still on
   * the wire; exchange_wait() bounds the wait so at most one transaction is ever
   * in flight and the buffer a transfer reads is never rewritten underneath it. */
  buf = link->xfer_buf;
  result = exchange_wait(link, 0u);
  link->xfer_pending = 0u;
  if (result != 0) {
    /* levels/hold were committed optimistically when the frame was armed; a
     * frame the FPGA never received must not survive in them. */
    link->levels_valid = 0u;
    link->hold_valid = 0u;
    if (++exchange_dma_failures >= FPGA_SPI_DMA_FAILURES_LATCH) {
      exchange_use_polling = 1u;
      system_status_fault(UMH_FAULT_FPGA_SPI_TIMEOUT, 21u, UMH_FAULT_WARNING);
    }
    /* Same single retry the blocking path has always done: the frame bytes are
     * still in the buffer it was armed from, so the register path can put the
     * frame on the wire. */
    if (buf == NULL || exchange_register(link, buf, link->tx_length) != 0) {
      link->xfer_kind = FPGA_XFER_NONE;
      return -3;
    }
  }
  result = unpack_status(link);
  if (result != 0) {
    link->xfer_kind = FPGA_XFER_NONE;
    link->levels_valid = 0u;
    link->hold_valid = 0u;
    return -4;
  }
  if (link->xfer_kind == FPGA_XFER_PHASE) ++link->phase_frames;
  else if (link->xfer_kind == FPGA_XFER_FULL) ++link->full_frames;
  link->xfer_kind = FPGA_XFER_NONE;
  link->running = 1u;
  exchange_dma_failures = 0u;
  return 0;
}

static int fpga_send_rgb_locked(fpga_link_t *link, const umh_rgb_value_t *rgb)
{
  umh_output_frame_t rgb_frame;
  uint16_t length;
  if (link == NULL || rgb == NULL) return -1;
  memset(&rgb_frame, 0, sizeof(rgb_frame));
  rgb_frame.update_flags = UMH_FRAME_FLAG_RGB;
  memcpy(rgb_frame.rgb, rgb, sizeof(rgb_frame.rgb));
  length = pack_common(link, link->tx, FPGA_CMD_WS2812, &rgb_frame);
  if (length == 0u) return -1;
  if (exchange(link, length) != 0) return -2;
  if (unpack_status(link) != 0) return -3;
  return 0;
}
static int fpga_status_locked(fpga_link_t *link)
{
  uint16_t length = pack_common(link, link->tx, FPGA_CMD_STATUS, NULL);
  if (length == 0u) return -1;
  if (exchange(link, length) != 0) return -1;
  return unpack_status(link);
}

int fpga_link_submit(fpga_link_t *link, const umh_output_frame_t *frame)
{
  return fpga_link_submit_allow_hold(link, frame, 0u);
}

int fpga_link_submit_allow_hold(fpga_link_t *link, const umh_output_frame_t *frame,
                                uint8_t allow_hold)
{
  uint8_t *tx;
  uint16_t length;
  uint8_t frame_has_ultrasound;
  uint8_t phase_only;
  uint8_t tx_done = 0u;
  uint8_t can_defer;
  int reap;
  if (link == NULL || frame == NULL || link->mutex == NULL) return -1;
  frame_has_ultrasound = (frame->update_flags & UMH_FRAME_FLAG_ULTRASOUND) != 0u;
  if (osMutexAcquire(link->mutex, osWaitForever) != osOK) return -1;
  reap = link_complete_deferred(link);
  if (reap != 0) {
    osMutexRelease(link->mutex);
    return reap;
  }

  if (allow_hold != 0u && frame_has_ultrasound != 0u && hold_match(link, frame) != 0u) {
    if ((uint32_t)(HAL_GetTick() - link->hold_last_tx_tick) >= 10u) {
      (void)fpga_status_locked(link);
      link->hold_last_tx_tick = HAL_GetTick();
    }
    osMutexRelease(link->mutex);
    return 0;
  }

  /* RGB-only frames must not command the event builder. */
  if (allow_hold != 0u && frame_has_ultrasound == 0u) {
    if ((frame->update_flags & UMH_FRAME_FLAG_RGB) != 0u &&
        (link->hold_valid == 0u ||
         (link->hold_flags & UMH_FRAME_FLAG_RGB) == 0u ||
         memcmp(link->hold_rgb, frame->rgb, sizeof(link->hold_rgb)) != 0)) {
      if (fpga_send_rgb_locked(link, frame->rgb) != 0) {
        osMutexRelease(link->mutex);
        return -4;
      }
      link->hold_last_tx_tick = HAL_GetTick();
      tx_done = 1u;
    }
    if ((frame->update_flags & UMH_FRAME_FLAG_RGB) != 0u) {
      memcpy(link->hold_rgb, frame->rgb, sizeof(link->hold_rgb));
      link->hold_flags |= UMH_FRAME_FLAG_RGB;
    }
    if (tx_done == 0u &&
        (uint32_t)(HAL_GetTick() - link->hold_last_tx_tick) >= 10u) {
      (void)fpga_status_locked(link);
      link->hold_last_tx_tick = HAL_GetTick();
    }
    osMutexRelease(link->mutex);
    return 0;
  }

  /* The credit in the reaped status was sampled while the previous frame was
   * still on the wire, so a zero here can be an obsolete "busy" sample and would
   * reject a frame the FPGA can take.  One fresh 36-byte STATUS transaction
   * tells the truth; a builder that really is busy still returns -1 and the
   * caller retries. */
  if (link->status.fifo_credit == 0u) {
    if (fpga_status_locked(link) != 0 || link->status.fifo_credit == 0u) {
      osMutexRelease(link->mutex);
      return -1;
    }
  }
  link->hold_valid = 0u;
  phase_only = phase_only_eligible(link, frame);
  /* Alternate the frame buffer so an in-flight transfer never reads the buffer
   * being packed.  Command traffic keeps tx[] and always reaps first; when the
   * DMA path is latched off the frame is packed into tx[] and sent the old way. */
  can_defer = exchange_use_polling == 0u;
  link->frame_buf ^= 1u;
  tx = can_defer && link->frame_buf != 0u ? link->tx_frame : link->tx;
  if (phase_only != 0u) {
    phase_only = pack_phase_fused(link, frame, tx);
    if (phase_only != 0u) phase_header_patch(link, frame, tx);
  }
  if (phase_only != 0u) {
    length = (uint16_t)(FPGA_PHASE_HEADER_BYTES + UMH_DEVICE_CHANNEL_COUNT);
  } else {
    length = pack_common(link, tx, FPGA_CMD_FRAME, frame);
  }
  if (length == 0u) { osMutexRelease(link->mutex); return -2; }
  if (phase_only == 0u && frame_has_ultrasound != 0u) cache_levels(link, frame);
  if (allow_hold != 0u) {
    link->hold_flags = (uint16_t)(frame->update_flags | UMH_FRAME_FLAG_ULTRASOUND);
    memcpy(link->hold_rgb, frame->rgb, sizeof(link->hold_rgb));
    link->hold_valid = 1u;
    link->hold_last_tx_tick = HAL_GetTick();
  }
  link->tx_length = length;
  if (can_defer != 0u && length > FPGA_SPI_POLL_MAX_BYTES) {
    link->xfer_kind = phase_only != 0u ? FPGA_XFER_PHASE : FPGA_XFER_FULL;
    link->xfer_buf = tx;
    if (exchange_arm(link, tx, length, 1u) == 0) {
      link->xfer_pending = 1u;
      osMutexRelease(link->mutex);
      return 0;
    }
    link->xfer_kind = FPGA_XFER_NONE;
    link->xfer_buf = NULL;
    if (exchange_register(link, tx, length) != 0) {
      link->levels_valid = 0u;
      link->hold_valid = 0u;
      osMutexRelease(link->mutex);
      return -3;
    }
  } else if (exchange(link, length) != 0) {
    link->levels_valid = 0u;
    link->hold_valid = 0u;
    osMutexRelease(link->mutex);
    return -3;
  }
  if (unpack_status(link) != 0) {
    link->levels_valid = 0u;
    link->hold_valid = 0u;
    osMutexRelease(link->mutex);
    return -4;
  }
  if (phase_only != 0u) {
    ++link->phase_frames;
  } else {
    ++link->full_frames;
  }
  link->running = 1u;
  osMutexRelease(link->mutex);
  return 0;
}

int fpga_link_poll_status(fpga_link_t *link)
{
  int result;
  if (link == NULL || link->mutex == NULL) return -1;
  if (osMutexAcquire(link->mutex, osWaitForever) != osOK) return -1;
  result = link_complete_deferred(link);
  if (result == 0) result = fpga_status_locked(link);
  osMutexRelease(link->mutex);
  return result;
}

int fpga_link_safe_stop(fpga_link_t *link)
{
  uint16_t length;
  if (link == NULL || link->mutex == NULL) return -1;
  if (osMutexAcquire(link->mutex, osWaitForever) != osOK) return -1;
  /* A STOP must go out even if the previous frame died on the bus, so the reap
   * result is not allowed to short-circuit it: the pending transfer is finished
   * here, which is enough to keep the two off the wire at the same time. */
  (void)link_complete_deferred(link);
  length = pack_common(link, link->tx, FPGA_CMD_STOP, NULL);
  if (exchange(link, length) != 0) { osMutexRelease(link->mutex); return -2; }
  if (unpack_status(link) != 0) { osMutexRelease(link->mutex); return -3; }
  link->levels_valid = 0u;
  link->hold_valid = 0u;
  {
    umh_rgb_value_t black[UMH_DEVICE_RGB_COUNT];
    memset(black, 0, sizeof(black));
    if (fpga_send_rgb_locked(link, black) != 0) {
      osMutexRelease(link->mutex);
      return -4;
    }
  }
  link->running = 0u;
  osMutexRelease(link->mutex);
  return 0;
}

int fpga_link_set_ws2812(fpga_link_t *link, uint8_t r, uint8_t g, uint8_t b)
{
  umh_rgb_value_t rgb[UMH_DEVICE_RGB_COUNT];
  uint8_t i;
  int result;
  if (link == NULL || link->mutex == NULL) return -1;
  for (i = 0u; i < UMH_DEVICE_RGB_COUNT; ++i) {
    rgb[i].red = r;
    rgb[i].green = g;
    rgb[i].blue = b;
  }
  if (osMutexAcquire(link->mutex, osWaitForever) != osOK) return -1;
  result = link_complete_deferred(link);
  if (result == 0) result = fpga_send_rgb_locked(link, rgb);
  if (result == 0 && link->hold_valid != 0u) {
    memcpy(link->hold_rgb, rgb, sizeof(link->hold_rgb));
    link->hold_flags |= UMH_FRAME_FLAG_RGB;
  }
  osMutexRelease(link->mutex);
  return result;
}

static int compact_mod_command(fpga_link_t *link, uint8_t command,
                                  uint8_t data, uint32_t sequence)
{
  int result = 0;
  if (link == NULL || link->mutex == NULL) return -1;
  if (osMutexAcquire(link->mutex, osWaitForever) != osOK) return -1;
  result = link_complete_deferred(link);
  if (result != 0) { osMutexRelease(link->mutex); return result; }
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

int fpga_link_mod_begin(fpga_link_t *link, const uint8_t *phases,
                          const uint8_t *enables, uint32_t sequence)
{
  umh_output_frame_t frame;
  uint16_t i;
  int result;
  if (link == NULL || phases == NULL || enables == NULL) return -1;
  /* Three safe steps:
   *   1. submit one silent full frame so the staging RAM and old enable
   *      markers cannot leak into the first common-level table;
   *   2. enter common-level mode at level 0 (the FPGA still holds phases
   *      from step 1 and rebuilds a silent table);
   *   3. submit the real aperture phases plus enable markers while
   *      mode is active, so the rebuild still runs at common level 0.
   * Envelope levels then travel in short MOD_FEED transactions. */
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

  result = fpga_link_mod_mode(link, 1u, sequence + 1u);
  if (result != 0) {
    (void)fpga_link_safe_stop(link);
    return result;
  }
  /* Entering common-level mode starts one silent rebuild.  Wait until that builder
   * has released FIFO credit again before loading the enable markers.  An
   * image without the level FIFO cannot stream, so refuse it here. */
  if ((link->status.status_flags & FPGA_STATUS_MOD_FEED) == 0u ||
      wait_for_credit(link) != 0) {
    (void)fpga_link_mod_mode(link, 0u, sequence + 3u);
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
    (void)fpga_link_mod_mode(link, 0u, sequence + 3u);
    (void)fpga_link_safe_stop(link);
    return result;
  }
  return 0;
}

int fpga_link_mod_feed(fpga_link_t *link, const uint8_t *levels, uint8_t count)
{
  int result;
  if (link == NULL || link->mutex == NULL || count > FPGA_MOD_BLOCK_MAX ||
      (count != 0u && levels == NULL)) return -1;
  /* The FIFO covers several milliseconds, so waiting a few ticks behind a
   * lower-priority transaction is harmless; failing would drop levels. */
  if (osMutexAcquire(link->mutex, 5u) != osOK) return -3;
  result = link_complete_deferred(link);
  if (result != 0) { osMutexRelease(link->mutex); return result; }
  link->tx[0] = FPGA_CMD_MOD_FEED;
  link->tx[1] = FPGA_PROTOCOL_VERSION;
  if (count != 0u) memcpy(&link->tx[2], levels, count);
  if (exchange(link, (uint16_t)(2u + count)) != 0) result = -2;
  else if (link->rx[0] != FPGA_PROTOCOL_VERSION) {
    link->levels_valid = 0u;
    link->hold_valid = 0u;
    result = -4;
  } else result = link->status.mod_fill = link->rx[1];
  osMutexRelease(link->mutex);
  return result;
}

int fpga_link_mod_mode(fpga_link_t *link, uint8_t enable, uint32_t sequence)
{
  uint8_t attempt;
  int result = compact_mod_command(link, FPGA_CMD_MOD_FIXED,
                                     enable != 0u ? 1u : 0u, sequence);
  if (result != 0 || enable == 0u) return result;
  /* The status word in the compact transaction is latched before CS falls,
   * so poll until the new bitstream confirms that MOD_FIXED=1 was adopted.
   * Old FPGA images silently ignore the compact command; without this check
   * the following silent FRAME could be built with the 0/255 enable marker
   * mistaken for a real level. */
  for (attempt = 0u; attempt < 20u; ++attempt) {
    if (fpga_link_poll_status(link) == 0 &&
        (link->status.status_flags & FPGA_STATUS_MOD_MODE) != 0u)
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
  uint32_t saved_cr1 = 0u;
  uint8_t slow_read = 0u;
  SPI_TypeDef *spi;
  if (link == NULL || link->mutex == NULL) return -1;
  spi = link->spi != NULL ? link->spi->Instance : NULL;
  if (extension_length > sizeof(frame.extension)) return -2;
  if (osMutexAcquire(link->mutex, osWaitForever) != osOK) return -1;
  result = link_complete_deferred(link);
  if (result != 0) { osMutexRelease(link->mutex); return result; }
  memset(&frame, 0, sizeof(frame));
  frame.update_flags = UMH_FRAME_FLAG_EXTENDED;
  frame.sequence = sequence;
  frame.extension_length = extension_length;
  if (extension != NULL && extension_length != 0u)
    memcpy(frame.extension, extension, extension_length);
  length = pack_common(link, link->tx, command, &frame);
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
      /* The FPGA microphone EBR read path needs a longer SCK setup/hold
       * window than a normal frame transaction.  At the 21.25 MHz control
       * link rate the first bit of a freshly addressed 16-bit word can be
       * sampled while the EBR output is still changing, which corrupts the
       * sign bit and makes every H-matrix phase useless.  MIC_READ is a
       * low-rate calibration-only transaction, so slow it down a little and
       * restore the normal bus speed afterwards. */
      if (spi != NULL && command == FPGA_CMD_MIC_READ) {
        saved_cr1 = spi->CR1;
        spi->CR1 = (saved_cr1 & ~(SPI_CR1_BR | SPI_CR1_SPE)) |
                   SPI_BAUDRATEPRESCALER_256;
        slow_read = 1u;
      }
      if (exchange(link, length) != 0) result = -3;
      else if (unpack_status(link) != 0) result = -4;
      else if (response != NULL && response_length != 0u)
        memcpy(response, link->rx, response_length);
      if (slow_read != 0u) {
        spi->CR1 = saved_cr1;
      }
    }
  }
  osMutexRelease(link->mutex);
  return result;
}

static uint16_t get_be16(const uint8_t *p)
{
  return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

int fpga_link_mic_audio_reset(fpga_link_t *link)
{
  return compact_mod_command(link, FPGA_CMD_MIC_AUDIO_RESET, 0u, HAL_GetTick());
}

int fpga_link_mic_audio_read(fpga_link_t *link, int16_t *samples, uint8_t capacity,
                             uint8_t *actual)
{
  uint8_t i;
  uint8_t count;
  int result;
  if (link == NULL || samples == NULL || actual == NULL ||
      capacity == 0u || capacity > FPGA_MIC_AUDIO_BATCH) return -1;
  if (osMutexAcquire(link->mutex, 5u) != osOK) return -2;
  result = link_complete_deferred(link);
  if (result != 0) { osMutexRelease(link->mutex); return result; }
  memset(link->tx, 0, FPGA_MIC_AUDIO_READ_BYTES);
  link->tx[0] = FPGA_CMD_MIC_AUDIO_READ;
  link->tx[1] = FPGA_PROTOCOL_VERSION;
  link->tx[2] = capacity;
  if (exchange(link, FPGA_MIC_AUDIO_READ_BYTES) != 0) result = -3;
  else if (unpack_status(link) != 0) result = -4;
  else {
    count = link->rx[16];
    if (count > capacity) count = capacity;
    for (i = 0u; i < count; ++i)
      samples[i] = (int16_t)get_be16(&link->rx[17u + (uint16_t)i * 2u]);
    *actual = count;
    result = 0;
  }
  osMutexRelease(link->mutex);
  return result;
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
