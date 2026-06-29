#include "ssd1306.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c3;

static uint8_t fb_hw[SSD1306_WIDTH * SSD1306_HEIGHT / 8];  /* 128x32 native HW buffer 512B */
static uint8_t fb_log[GUI_WIDTH * GUI_HEIGHT / 8];          /* 32x128 logical (portrait) 512B */

static int ssd1306_write_cmd(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};  /* control byte 0x00 = command */
    return HAL_I2C_Master_Transmit(&hi2c3, SSD1306_ADDR, buf, 2, 100);
}

static int ssd1306_write_data_burst(const uint8_t *data, uint16_t len)
{
    return HAL_I2C_Master_Transmit(&hi2c3, SSD1306_ADDR, (uint8_t *)data, len, 100);
}

int SSD1306_Init(void)
{
    /* Wait for SSD1306 VCC to stabilize (~100ms after MCU power-up).
     * The charge pump needs this before accepting commands. */
    HAL_Delay(100);

    /* Verify device is present on I2C bus */
    if (HAL_I2C_IsDeviceReady(&hi2c3, SSD1306_ADDR, 3, 100) != HAL_OK)
        return -1;

    static const uint8_t init_cmds[] = {
        0xAE,           /* display off */
        0xD5, 0x80,     /* clock divide ratio = 0x80 (default) */
        0xA8, 0x1F,     /* multiplex = 31 (32 rows) */
        0xD3, 0x00,     /* display offset = 0 */
        0x40,           /* start line = 0 */
        0x8D, 0x14,     /* charge pump enable */
        0x20, 0x00,     /* horizontal addressing mode */
        0xA1,           /* segment remap (column 127 = SEG0) */
        0xC8,           /* COM output scan direction */
        0xDA, 0x02,     /* COM pins HW config: sequential, for 128x32 */
        0x81, 0x8F,     /* contrast = 0x8F */
        0xD9, 0xF1,     /* precharge period = 0xF1 */
        0xDB, 0x40,     /* VCOMH deselect level = 0x40 */
        0xA4,           /* display on resume (normal, not all-on) */
        0xA6,           /* normal (non-inverted) display */
        0xAF            /* display on */
    };

    /* Send init commands one-by-one; abort on first error */
    for (uint8_t i = 0; i < sizeof(init_cmds); i++)
    {
        if (ssd1306_write_cmd(init_cmds[i]) != HAL_OK)
            return -2;
    }

    SSD1306_Fill(BLACK);
    SSD1306_Flush();
    return 0;  /* success */
}

void SSD1306_DeInit(void)
{
    ssd1306_write_cmd(0xAE); /* display off */
}

void SSD1306_Fill(Colour c)
{
    memset(fb_log, c ? 0xFF : 0x00, sizeof(fb_log));
}

void SSD1306_DrawPixel(int16_t x, int16_t y, Colour c)
{
    if (x < 0 || x >= GUI_WIDTH || y < 0 || y >= GUI_HEIGHT) return;
    uint16_t idx = (uint16_t)y * GUI_WIDTH + (uint16_t)x;
    if (c == WHITE)
        fb_log[idx >> 3] |=  (1U << (idx & 7));
    else
        fb_log[idx >> 3] &= ~(1U << (idx & 7));
}

void SSD1306_DrawHLine(int16_t x, int16_t y, int16_t w, Colour c)
{
    for (int16_t i = 0; i < w; i++)
        SSD1306_DrawPixel(x + i, y, c);
}

void SSD1306_DrawVLine(int16_t x, int16_t y, int16_t h, Colour c)
{
    for (int16_t i = 0; i < h; i++)
        SSD1306_DrawPixel(x, y + i, c);
}

void SSD1306_DrawRect(int16_t x, int16_t y, int16_t w, int16_t h, Colour c)
{
    SSD1306_DrawHLine(x, y, w, c);
    SSD1306_DrawHLine(x, y + h - 1, w, c);
    SSD1306_DrawVLine(x, y, h, c);
    SSD1306_DrawVLine(x + w - 1, y, h, c);
}

void SSD1306_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, Colour c)
{
    for (int16_t yy = y; yy < y + h; yy++)
        SSD1306_DrawHLine(x, yy, w, c);
}

/* Transpose: logical 32x128 (portrait) → hardware 128x32 (native).
 * Logical (x,y) → Hardware (X=y, Y=31-x): 90° clockwise rotation. */
void SSD1306_Flush(void)
{
    memset(fb_hw, 0, sizeof(fb_hw));
    for (int y = 0; y < GUI_HEIGHT; y++)       /* logical row (0..127) */
    {
        for (int x = 0; x < GUI_WIDTH; x++)    /* logical col (0..31) */
        {
            uint16_t li = (uint16_t)y * GUI_WIDTH + (uint16_t)x;
            if (fb_log[li >> 3] & (1U << (li & 7)))
            {
                /* HW col = y (0..127), HW page = (31-x)/8, bit-in-page = (31-x)%8 */
                uint16_t hx = (uint16_t)y;
                uint16_t hy = (uint16_t)(GUI_WIDTH - 1 - x);
                uint16_t hi = (hy >> 3) * SSD1306_WIDTH + hx;
                fb_hw[hi] |= (1U << (hy & 7));
            }
        }
    }

    /* Set column range 0..127, page range 0..3 */
    ssd1306_write_cmd(0x21); ssd1306_write_cmd(0);
    ssd1306_write_cmd(SSD1306_WIDTH - 1);
    ssd1306_write_cmd(0x22); ssd1306_write_cmd(0);
    ssd1306_write_cmd((SSD1306_HEIGHT >> 3) - 1);

    /* Burst write with control byte 0x40 (data) prepended */
    uint8_t buf[SSD1306_WIDTH * SSD1306_HEIGHT / 8 + 1];
    buf[0] = 0x40;
    memcpy(&buf[1], fb_hw, sizeof(fb_hw));
    ssd1306_write_data_burst(buf, sizeof(buf));
}
