#include "oled_ssd1315.h"
#include "i2c_bus.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

#define OLED_I2C_ADDRESS 0x78u

/* Page payloads move by DMA.
 *
 * A 129-byte page takes ~4 ms on this bus, and HAL_I2C_Master_Transmit() spends
 * that time polling the status register.  With the HOME page showing live frame
 * counters, one page changes every repaint, and the display task measured 14 %
 * of the CPU while idle and 26 % during playback -- all of it waiting for bytes
 * to shift out.  HAL_I2C_Master_Transmit_DMA() hands the same buffer to DMA2
 * channel 3 and blocks the task on a semaphore, so the transfer costs no CPU and
 * the idle task (or the protocol task) runs in the gap.  Short command writes
 * stay on the polling path: arming DMA for four bytes is slower than the
 * transfer itself. */
/* Every transfer uses DMA once the semaphore exists.  The four-byte page-address
 * command still occupies the bus for ~136 us at this clock; moving it to DMA is
 * what removes the last measurable slice of the display task's CPU share. */
#define OLED_DMA_MIN_BYTES 2u
#define OLED_DMA_TIMEOUT_MS 100u

static SemaphoreHandle_t oled_dma_sem;
static StaticSemaphore_t oled_dma_sem_mem;
static volatile uint8_t oled_dma_error;

static uint8_t oled_dma_available(void)
{
  if (oled_dma_sem == NULL) {
    oled_dma_sem = xSemaphoreCreateBinaryStatic(&oled_dma_sem_mem);
    if (oled_dma_sem != NULL) {
      /* Completion is signalled from the DMA/I2C interrupt, which must be able
       * to call FreeRTOS.  MX_I2C1_Init() leaves both lines disabled. */
      HAL_NVIC_SetPriority(I2C1_EV_IRQn, 6u, 0u);
      HAL_NVIC_SetPriority(I2C1_ER_IRQn, 6u, 0u);
    }
  }
  return oled_dma_sem != NULL ? 1u : 0u;
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c1 && oled_dma_sem != NULL) {
    BaseType_t woken = pdFALSE;
    (void)xSemaphoreGiveFromISR(oled_dma_sem, &woken);
    portYIELD_FROM_ISR(woken);
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c1 && oled_dma_sem != NULL) {
    BaseType_t woken = pdFALSE;
    oled_dma_error = 1u;
    (void)xSemaphoreGiveFromISR(oled_dma_sem, &woken);
    portYIELD_FROM_ISR(woken);
  }
}

/* Consecutive failed page transfers before the panel is left alone, and how long
 * it is left alone for.  One failure costs a full 100 ms I2C timeout, so three of
 * them are ~300 ms of the UI task -- and the UI task is also what dispatches key
 * events, which is why a dead or missing display used to look like dead buttons
 * (the gun menu never opened even though the long press had been detected). */
#define OLED_FAILURES_BEFORE_BACKOFF 3u
#define OLED_BACKOFF_MS 2000u

/* Compact 5x7 glyphs for the device UI.  The UI intentionally uses ASCII so
 * the firmware does not need a large font or a locale-specific text resource. */
static const uint8_t font_upper[26][5] = {
  {0x7Eu,0x11u,0x11u,0x11u,0x7Eu},{0x7Fu,0x49u,0x49u,0x49u,0x36u},
  {0x3Eu,0x41u,0x41u,0x41u,0x22u},{0x7Fu,0x41u,0x41u,0x22u,0x1Cu},
  {0x7Fu,0x49u,0x49u,0x49u,0x41u},{0x7Fu,0x09u,0x09u,0x09u,0x01u},
  {0x3Eu,0x41u,0x49u,0x49u,0x7Au},{0x7Fu,0x08u,0x08u,0x08u,0x7Fu},
  {0x00u,0x41u,0x7Fu,0x41u,0x00u},{0x20u,0x40u,0x41u,0x3Fu,0x01u},
  {0x7Fu,0x08u,0x14u,0x22u,0x41u},{0x7Fu,0x40u,0x40u,0x40u,0x40u},
  {0x7Fu,0x02u,0x0Cu,0x02u,0x7Fu},{0x7Fu,0x04u,0x08u,0x10u,0x7Fu},
  {0x3Eu,0x41u,0x41u,0x41u,0x3Eu},{0x7Fu,0x09u,0x09u,0x09u,0x06u},
  {0x3Eu,0x41u,0x51u,0x21u,0x5Eu},{0x7Fu,0x09u,0x19u,0x29u,0x46u},
  {0x46u,0x49u,0x49u,0x49u,0x31u},{0x01u,0x01u,0x7Fu,0x01u,0x01u},
  {0x3Fu,0x40u,0x40u,0x40u,0x3Fu},{0x1Fu,0x20u,0x40u,0x20u,0x1Fu},
  {0x7Fu,0x20u,0x18u,0x20u,0x7Fu},{0x63u,0x14u,0x08u,0x14u,0x63u},
  {0x07u,0x08u,0x70u,0x08u,0x07u},{0x61u,0x51u,0x49u,0x45u,0x43u}
};
/* Stored already bit-reversed into the framebuffer's LSB-first column order.
 * Deriving that with a 7-bit loop per glyph column cost more than the rest of
 * the character blit together on the 33 Hz repaint path. */
static const uint8_t font_digit[10][5] = {
  {0x3Eu,0x51u,0x49u,0x45u,0x3Eu},{0x00u,0x42u,0x7Fu,0x40u,0x00u},
  {0x62u,0x51u,0x49u,0x45u,0x42u},{0x21u,0x41u,0x45u,0x4Bu,0x31u},
  {0x18u,0x14u,0x12u,0x7Fu,0x10u},{0x27u,0x45u,0x45u,0x45u,0x39u},
  {0x3Cu,0x4Au,0x49u,0x49u,0x30u},{0x01u,0x71u,0x09u,0x05u,0x03u},
  {0x36u,0x49u,0x49u,0x49u,0x36u},{0x06u,0x49u,0x49u,0x29u,0x1Eu}
};

static uint8_t glyph_column(char character, uint8_t column)
{
  if (column >= 5u) return 0u;
  if (character >= 'A' && character <= 'Z') return font_upper[(uint8_t)character - 'A'][column];
  if (character >= 'a' && character <= 'z') return font_upper[(uint8_t)character - 'a'][column];
  if (character >= '0' && character <= '9')
    return font_digit[(uint8_t)character - '0'][column];
  switch (character) {
    case ':': return column == 1u || column == 3u ? 0x14u : 0u;
    case '.': return column == 2u ? 0x40u : 0u;
    case '-': return column >= 1u && column <= 3u ? 0x08u : 0u;
    case '/': return (uint8_t)(0x40u >> column);
    case '>': return column == 0u ? 0x08u : (column == 1u ? 0x14u : (column == 2u ? 0x22u : 0x41u));
    case '#': return (uint8_t)((column == 1u || column == 3u) ? 0x7Fu : 0x14u);
    case '%': return column == 0u ? 0x63u : (column == 1u ? 0x14u : (column == 2u ? 0x08u : (column == 3u ? 0x14u : 0x63u)));
    default: return 0u;
  }
}

static int send(oled_ssd1315_t *oled, uint8_t control, const uint8_t *data, uint16_t length)
{
  if (oled == NULL || oled->i2c == NULL || length > OLED_WIDTH) return -1;
  oled->tx_buffer[0] = control;
  if (length != 0u && data != NULL) memcpy(&oled->tx_buffer[1], data, length);
  /* Never start a transfer on a bus that a slave/reset left low: with a
   * 129-byte OLED page and a slow per-byte HAL timeout that turns into a
   * multi-second UI freeze.  If the bus stays dead, fail fast so the rest of
   * the system (especially calibration) keeps running. */
  if (i2c_bus_is_dead() != 0) return -1;
  (void)i2c_bus_recover_if_stuck();
  if (i2c_bus_lock(100u) != 0) return -1;
  /* The bus lock is held until the transfer really finished, so the shared
   * tx_buffer and the EEPROM transactions stay serialised in both modes. */
  HAL_StatusTypeDef result;
  if (length + 1u >= OLED_DMA_MIN_BYTES && oled_dma_available() != 0u) {
    (void)xSemaphoreTake(oled_dma_sem, 0);
    oled_dma_error = 0u;
    /* MX_I2C1_Init() disables both lines, and a bus recovery re-runs it. */
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
    result = HAL_I2C_Master_Transmit_DMA(oled->i2c, OLED_I2C_ADDRESS,
                                         oled->tx_buffer, (uint16_t)(length + 1u));
    if (result == HAL_OK &&
        (xSemaphoreTake(oled_dma_sem, pdMS_TO_TICKS(OLED_DMA_TIMEOUT_MS)) != pdTRUE ||
         oled_dma_error != 0u))
      result = HAL_TIMEOUT;
  } else {
    result = HAL_I2C_Master_Transmit(oled->i2c, OLED_I2C_ADDRESS,
                                     oled->tx_buffer,
                                     (uint16_t)(length + 1u), OLED_DMA_TIMEOUT_MS);
  }
  i2c_bus_unlock();
  if (result != HAL_OK) {
    /* A timed-out slave can hold SDA low; release it before the next frame,
     * otherwise every following refresh would keep failing and the UI display
     * would stay frozen on its last image. */
    (void)i2c_bus_recover();
    return -1;
  }
  return 0;
}

void oled_ssd1315_init(oled_ssd1315_t *oled, I2C_HandleTypeDef *i2c)
{
  static const uint8_t commands[] = {0xAEu, 0xD5u, 0x80u, 0xA8u, 0x3Fu, 0xD3u, 0x00u, 0x40u,
                                     0x8Du, 0x14u, 0x20u, 0x02u, 0xA0u, 0xC0u, 0xDAu, 0x12u,
                                     0x81u, 0x8Fu, 0xD9u, 0xF1u, 0xDBu, 0x40u, 0xA4u, 0xA6u};
  static const uint8_t display_on = 0xAFu;
  int init_ok;
  if (oled == NULL) return;
  memset(oled, 0, sizeof(*oled));
  oled->i2c = i2c;
  i2c_bus_init();
  init_ok = send(oled, 0x00u, commands, sizeof(commands));
  oled->initialized = init_ok == 0 ? 1u : 0u;
  oled_ssd1315_clear(oled);
  /* The controller's RAM is not guaranteed to reset with the MCU.  Force a
   * complete first refresh even though the local framebuffer starts at zero. */
  if (oled->initialized != 0u) {
    /* A0/C0 rotates both axes relative to the original A1/C8 mounting.
     * Keep the display off until all power-on RAM has been overwritten. */
    oled->dirty_pages = 0xFFu;
    oled->force_full = 1u;
    if (oled_ssd1315_refresh(oled) != 0 || send(oled, 0x00u, &display_on, 1u) != 0)
      oled->initialized = 0u;
  }
}

/* Clear the backing buffer only.
 *
 * This used to mark every page that held a pixel as dirty, and device_gui_render()
 * calls it on every repaint (clear-then-redraw).  A page whose content was about
 * to be redrawn identical was therefore re-sent to the panel 33 times a second:
 * 8 pages x 129 bytes of blocking I2C per frame, which measured as ~70 % of the
 * whole CPU while the device was sitting idle.  Change detection now lives in
 * oled_ssd1315_refresh(), which compares against the panel shadow, so this can be
 * a plain memset. */
void oled_ssd1315_clear(oled_ssd1315_t *oled)
{
  if (oled == NULL) return;
  memset(oled->framebuffer, 0, sizeof(oled->framebuffer));
}

void oled_ssd1315_set_pixel(oled_ssd1315_t *oled, uint8_t x, uint8_t y, uint8_t on)
{
  uint16_t index;
  uint8_t mask;
  if (oled == NULL || x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
  index = (uint16_t)((y / 8u) * OLED_WIDTH + x);
  mask = (uint8_t)(1u << (y % 8u));
  if (on != 0u) {
    if ((oled->framebuffer[index] & mask) == 0u) {
      oled->framebuffer[index] |= mask;
      oled->dirty_pages |= (uint8_t)(1u << (y / 8u));
    }
  } else if ((oled->framebuffer[index] & mask) != 0u) {
    oled->framebuffer[index] &= (uint8_t)~mask;
    oled->dirty_pages |= (uint8_t)(1u << (y / 8u));
  }
}

/* Blit one 6x8 character cell.
 *
 * The glyph is stored as six vertical bytes, exactly the SSD1315 page layout, so
 * a character is six read-modify-writes of one byte (plus six more when the cell
 * straddles a page boundary) instead of 48 calls into set_pixel(), each with its
 * own bounds test, bit mask and read-modify-write.  Repainting the whole screen
 * at 33 Hz measured as ~4.7 % of the CPU with the per-pixel loop; this is the
 * hot path of the UI task and of every menu repaint. */
void oled_ssd1315_draw_char(oled_ssd1315_t *oled, uint8_t x, uint8_t y,
                            char character, uint8_t inverted)
{
  uint8_t page;
  uint8_t shift;
  uint8_t column;
  if (oled == NULL || x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
  page = (uint8_t)(y / 8u);
  shift = (uint8_t)(y % 8u);
  for (column = 0u; column < 6u; ++column) {
    uint16_t index;
    uint8_t value;
    if ((uint16_t)x + column >= OLED_WIDTH) break;
    value = column < 5u ? glyph_column(character, column) : 0u;
    if (inverted != 0u) value = (uint8_t)~value;
    index = (uint16_t)((uint16_t)page * OLED_WIDTH + x + column);
    if (shift == 0u) {
      oled->framebuffer[index] = value;
    } else {
      uint8_t low_keep = (uint8_t)((1u << shift) - 1u);
      oled->framebuffer[index] =
          (uint8_t)((oled->framebuffer[index] & low_keep) | (uint8_t)(value << shift));
      if ((uint16_t)page + 1u < OLED_PAGES) {
        uint16_t next = (uint16_t)(index + OLED_WIDTH);
        oled->framebuffer[next] =
            (uint8_t)((oled->framebuffer[next] & (uint8_t)~low_keep) |
                      (uint8_t)(value >> (8u - shift)));
      }
    }
  }
}

void oled_ssd1315_draw_text(oled_ssd1315_t *oled, uint8_t x, uint8_t y,
                            const char *text, uint8_t inverted)
{
  if (oled == NULL || text == NULL) return;
  while (*text != '\0' && x < OLED_WIDTH) {
    oled_ssd1315_draw_char(oled, x, y, *text++, inverted);
    x = (uint8_t)(x + 6u);
  }
}

void oled_ssd1315_set_status(oled_ssd1315_t *oled, uint32_t flags,
                             uint16_t frames, uint16_t free_frames,
                             uint16_t fpga_credit, uint32_t errors)
{
  uint8_t x;
  if (oled == NULL) return;
  oled_ssd1315_clear(oled);
  for (x = 0u; x < 32u; ++x) if ((flags & (1u << (x % 8u))) != 0u) oled_ssd1315_set_pixel(oled, x, 0u, 1u);
  for (x = 0u; x < frames && x < 64u; ++x) oled_ssd1315_set_pixel(oled, x, 16u, 1u);
  for (x = 0u; x < free_frames && x < 64u; ++x) oled_ssd1315_set_pixel(oled, x, 32u, 1u);
  for (x = 0u; x < fpga_credit && x < 64u; ++x) oled_ssd1315_set_pixel(oled, x, 48u, 1u);
  for (x = 0u; x < (errors > 64u ? 64u : errors); ++x) oled_ssd1315_set_pixel(oled, x, 63u, 1u);
}

int oled_ssd1315_refresh(oled_ssd1315_t *oled)
{
  uint8_t page;
  uint8_t commands[3];
  if (oled == NULL || oled->initialized == 0u) return -1;
  /* A panel that is not answering must not hold the UI task: that task is also
   * what dispatches key events, so eight 100 ms I2C timeouts per refresh read as
   * dead buttons and the gun menu would never open even though the long press had
   * been detected.  After a few consecutive failures, leave the display alone for
   * a while and let the rest of the UI keep running. */
  if (oled->retry_after_ms != 0u) {
    if ((int32_t)(oled->retry_after_ms - HAL_GetTick()) > 0) return -3;
    oled->retry_after_ms = 0u;
    oled->failures = 0u;
  }
  for (page = 0u; page < OLED_PAGES; ++page) {
    uint16_t offset = (uint16_t)page * OLED_WIDTH;
    uint8_t first;
    uint8_t last;
    uint8_t span;
    /* Content compare, not the dirty hint: device_gui_render() clears the
     * framebuffer and immediately redraws the same picture, so every lit pixel
     * goes 0 -> 1 again and re-arms its page bit on every repaint.  Only the
     * columns that actually changed are sent, which on a live status page is a
     * handful of digit cells instead of the whole 128-byte page. */
    if (oled->force_full != 0u) {
      first = 0u;
      last = (uint8_t)(OLED_WIDTH - 1u);
    } else {
      first = 0u;
      while (first < OLED_WIDTH &&
             oled->framebuffer[offset + first] == oled->shadow[offset + first]) ++first;
      if (first >= OLED_WIDTH) continue;
      last = (uint8_t)(OLED_WIDTH - 1u);
      while (last > first &&
             oled->framebuffer[offset + last] == oled->shadow[offset + last]) --last;
    }
    span = (uint8_t)(last - first + 1u);
    commands[0] = (uint8_t)(0xB0u | page);
    commands[1] = (uint8_t)(0x00u | (first & 0x0Fu));
    commands[2] = (uint8_t)(0x10u | (first >> 4u));
    if (send(oled, 0x00u, commands, sizeof(commands)) != 0 ||
        send(oled, 0x40u, &oled->framebuffer[offset + first], span) != 0) {
      if (++oled->failures >= OLED_FAILURES_BEFORE_BACKOFF) {
        oled->retry_after_ms = HAL_GetTick() + OLED_BACKOFF_MS;
        oled->failures = 0u;
      }
      return -2;
    }
    memcpy(&oled->shadow[offset + first], &oled->framebuffer[offset + first], span);
    oled->dirty_pages &= (uint8_t)~(1u << page);
  }
  oled->force_full = 0u;
  oled->failures = 0u;
  return 0;
}
