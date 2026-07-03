#pragma once
#include "main.h"

#define SSD1306_WIDTH   128
#define SSD1306_HEIGHT   32
#define SSD1306_ADDR    (0x3C << 1)  /* 8-bit write = 0x78 */

/* Logical canvas: 128 wide × 32 tall (landscape, matches SSD1306 native) */
#define GUI_WIDTH      128
#define GUI_HEIGHT      32

typedef enum { BLACK = 0, WHITE = 1 } Colour;

int SSD1306_Init(void);
void SSD1306_DeInit(void);
void SSD1306_Fill(Colour c);
void SSD1306_DrawPixel(int16_t x, int16_t y, Colour c);
void SSD1306_DrawHLine(int16_t x, int16_t y, int16_t w, Colour c);
void SSD1306_DrawVLine(int16_t x, int16_t y, int16_t h, Colour c);
void SSD1306_DrawRect(int16_t x, int16_t y, int16_t w, int16_t h, Colour c);
void SSD1306_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, Colour c);
void SSD1306_Flush(void);
