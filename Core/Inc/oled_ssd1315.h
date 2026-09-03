#ifndef OLED_SSD1315_H
#define OLED_SSD1315_H

#include <stdint.h>
#include "i2c.h"

#define OLED_WIDTH 128u
#define OLED_HEIGHT 64u
#define OLED_PAGES 8u

typedef struct {
  I2C_HandleTypeDef *i2c;
  uint8_t framebuffer[OLED_WIDTH * OLED_PAGES];
  uint8_t shadow[OLED_WIDTH * OLED_PAGES];
  uint8_t tx_buffer[OLED_WIDTH + 1u];
  uint8_t dirty_pages;
  uint8_t initialized;
} oled_ssd1315_t;

void oled_ssd1315_init(oled_ssd1315_t *oled, I2C_HandleTypeDef *i2c);
void oled_ssd1315_clear(oled_ssd1315_t *oled);
void oled_ssd1315_set_pixel(oled_ssd1315_t *oled, uint8_t x, uint8_t y, uint8_t on);
void oled_ssd1315_draw_char(oled_ssd1315_t *oled, uint8_t x, uint8_t y,
                            char character, uint8_t inverted);
void oled_ssd1315_draw_text(oled_ssd1315_t *oled, uint8_t x, uint8_t y,
                            const char *text, uint8_t inverted);
void oled_ssd1315_set_status(oled_ssd1315_t *oled, uint32_t flags,
                             uint16_t frames, uint16_t free_frames,
                             uint16_t fpga_credit, uint32_t errors);
int oled_ssd1315_refresh(oled_ssd1315_t *oled);

#endif
