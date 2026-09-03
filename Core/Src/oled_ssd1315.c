#include "oled_ssd1315.h"
#include "i2c_bus.h"
#include "cmsis_os.h"
#include <string.h>

#define OLED_I2C_ADDRESS 0x78u

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
static const uint8_t font_digit[10][5] = {
  {0x3Eu,0x45u,0x49u,0x51u,0x3Eu},{0x00u,0x21u,0x7Fu,0x01u,0x00u},
  {0x23u,0x45u,0x49u,0x51u,0x21u},{0x42u,0x41u,0x51u,0x69u,0x46u},
  {0x0Cu,0x14u,0x24u,0x7Fu,0x04u},{0x72u,0x51u,0x51u,0x51u,0x4Eu},
  {0x1Eu,0x29u,0x49u,0x49u,0x06u},{0x40u,0x47u,0x48u,0x50u,0x60u},
  {0x36u,0x49u,0x49u,0x49u,0x36u},{0x30u,0x49u,0x49u,0x4Au,0x3Cu}
};

static uint8_t glyph_column(char character, uint8_t column)
{
  if (column >= 5u) return 0u;
  if (character >= 'A' && character <= 'Z') return font_upper[(uint8_t)character - 'A'][column];
  if (character >= 'a' && character <= 'z') return font_upper[(uint8_t)character - 'a'][column];
  if (character >= '0' && character <= '9') return font_digit[(uint8_t)character - '0'][column];
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
  uint32_t start;
  if (oled == NULL || oled->i2c == NULL || length > OLED_WIDTH) return -1;
  oled->tx_buffer[0] = control;
  if (length != 0u && data != NULL) memcpy(&oled->tx_buffer[1], data, length);
  if (i2c_bus_lock(100u) != 0) return -1;
  if (HAL_I2C_Master_Transmit_DMA(oled->i2c, OLED_I2C_ADDRESS, oled->tx_buffer,
                                 (uint16_t)(length + 1u)) != HAL_OK) {
    i2c_bus_unlock();
    return -1;
  }
  start = HAL_GetTick();
  while (HAL_I2C_GetState(oled->i2c) != HAL_I2C_STATE_READY) {
    if ((HAL_GetTick() - start) > 100u) {
      (void)HAL_I2C_Master_Abort_IT(oled->i2c, OLED_I2C_ADDRESS);
      i2c_bus_unlock();
      return -1;
    }
    if (osKernelGetState() == osKernelRunning) osDelay(1u);
  }
  i2c_bus_unlock();
  return 0;
}

void oled_ssd1315_init(oled_ssd1315_t *oled, I2C_HandleTypeDef *i2c)
{
  static const uint8_t commands[] = {0xAEu, 0xD5u, 0x80u, 0xA8u, 0x3Fu, 0xD3u, 0x00u, 0x40u,
                                     0x8Du, 0x14u, 0x20u, 0x02u, 0xA1u, 0xC8u, 0xDAu, 0x12u,
                                     0x81u, 0x8Fu, 0xD9u, 0xF1u, 0xDBu, 0x40u, 0xA4u, 0xA6u, 0xAFu};
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
  if (oled->initialized != 0u) oled->dirty_pages = 0xFFu;
}

void oled_ssd1315_clear(oled_ssd1315_t *oled)
{
  if (oled == NULL) return;
  uint8_t page;
  for (page = 0u; page < OLED_PAGES; ++page) {
    uint16_t offset = (uint16_t)page * OLED_WIDTH;
    if (memcmp(&oled->framebuffer[offset], (const uint8_t[OLED_WIDTH]){0}, OLED_WIDTH) != 0 ||
        memcmp(&oled->framebuffer[offset], &oled->shadow[offset], OLED_WIDTH) != 0) {
      memset(&oled->framebuffer[offset], 0, OLED_WIDTH);
      oled->dirty_pages |= (uint8_t)(1u << page);
    }
  }
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

void oled_ssd1315_draw_char(oled_ssd1315_t *oled, uint8_t x, uint8_t y,
                            char character, uint8_t inverted)
{
  uint8_t column;
  uint8_t row;
  if (oled == NULL || x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
  for (column = 0u; column < 6u && (uint16_t)x + column < OLED_WIDTH; ++column) {
    for (row = 0u; row < 8u && (uint16_t)y + row < OLED_HEIGHT; ++row) {
      uint8_t on = (column < 5u && (glyph_column(character, column) & (1u << row)) != 0u) ? 1u : 0u;
      oled_ssd1315_set_pixel(oled, (uint8_t)(x + column), (uint8_t)(y + row),
                             inverted != 0u ? (uint8_t)!on : on);
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
  for (page = 0u; page < OLED_PAGES; ++page) {
    uint16_t offset;
    if ((oled->dirty_pages & (1u << page)) == 0u) continue;
    offset = (uint16_t)page * OLED_WIDTH;
    commands[0] = (uint8_t)(0xB0u | page);
    commands[1] = 0x00u;
    commands[2] = 0x10u;
    if (send(oled, 0x00u, commands, sizeof(commands)) != 0 || send(oled, 0x40u, &oled->framebuffer[offset], OLED_WIDTH) != 0) return -2;
    memcpy(&oled->shadow[offset], &oled->framebuffer[offset], OLED_WIDTH);
    oled->dirty_pages &= (uint8_t)~(1u << page);
  }
  return 0;
}
