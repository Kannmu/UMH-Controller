#pragma once
#include "ssd1306.h"

/* 5x7 ASCII font (6px advance including 1px spacing), printable chars 32-126 */
extern const uint8_t Font5x7[95][7];

/* 8x16 ASCII font (9px advance), printable chars 32-126 */
extern const uint8_t Font8x16[95][16];

void Font_DrawChar(int16_t x, int16_t y, char ch, uint8_t font, uint8_t scale, Colour c);
void Font_DrawStr(int16_t x, int16_t y, const char *s, uint8_t font, uint8_t scale, Colour c);
int16_t Font_StrWidth(const char *s, uint8_t font, uint8_t scale);
int16_t Font_CharWidth(uint8_t font, uint8_t scale);
int16_t Font_CharHeight(uint8_t font, uint8_t scale);
