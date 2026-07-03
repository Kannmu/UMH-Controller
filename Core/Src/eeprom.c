#include "eeprom.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c3;

uint8_t EEPROM_WriteByte(uint16_t addr, uint8_t b)
{
    return HAL_I2C_Mem_Write(&hi2c3, EEPROM_DEV_ADDR, addr, I2C_MEMADD_SIZE_8BIT, &b, 1, 10);
}

uint8_t EEPROM_ReadByte(uint16_t addr, uint8_t *b)
{
    return HAL_I2C_Mem_Read(&hi2c3, EEPROM_DEV_ADDR, addr, I2C_MEMADD_SIZE_8BIT, b, 1, 10);
}

uint8_t EEPROM_WriteBuffer(uint16_t addr, const uint8_t *data, uint16_t len)
{
    while (len > 0)
    {
        uint16_t chunk = EEPROM_PAGE_SIZE - (addr % EEPROM_PAGE_SIZE);
        if (chunk > len) chunk = len;
        if (HAL_I2C_Mem_Write(&hi2c3, EEPROM_DEV_ADDR, addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, chunk, 100) != HAL_OK)
            return 0;
        HAL_Delay(EEPROM_WRITE_DELAY_MS);
        addr += chunk;
        data += chunk;
        len  -= chunk;
    }
    return 1;
}

uint8_t EEPROM_ReadBuffer(uint16_t addr, uint8_t *data, uint16_t len)
{
    if (HAL_I2C_Mem_Read(&hi2c3, EEPROM_DEV_ADDR, addr, I2C_MEMADD_SIZE_8BIT, data, len, 100) != HAL_OK)
        return 0;
    return 1;
}

/* Memory layout:
 *  0:   magic[4]  = {'U','M','H','C'}
 *  4:   version   (uint16_t) = 1
 *  6:   num       (uint16_t) = NUM_REAL_TRANSDUCER
 *  8:   calib_us[NUM_REAL_TRANSDUCER] (float, EEPROM_CAL_DATA bytes)
 *  EEPROM_CAL_CS_OFFSET: checksum  (uint32_t)
 *  Total EEPROM_CAL_SIZE bytes */
#define EEPROM_CAL_MAGIC  0x43484D55  /* 'UMHC' little-endian */
#define EEPROM_CAL_VERSION 1u
#define EEPROM_CAL_OFFSET  0u
#define EEPROM_CAL_HEADER_SIZE    8u
#define EEPROM_CAL_DATA_BYTES     (NUM_REAL_TRANSDUCER * sizeof(float))
#define EEPROM_CAL_CS_OFFSET      (EEPROM_CAL_HEADER_SIZE + EEPROM_CAL_DATA_BYTES)
#define EEPROM_CAL_SIZE           (EEPROM_CAL_CS_OFFSET + 4u)

static uint32_t eeprom_checksum(const uint8_t *data, uint16_t len)
{
    uint32_t c = 0;
    for (uint16_t i = 0; i < len; i++) c = (c << 1) ^ data[i];  /* simplified rolling XOR */
    return c;
}

uint8_t EEPROM_SaveCalibration(const float calib_us[NUM_REAL_TRANSDUCER])
{
    uint8_t buf[EEPROM_CAL_SIZE];
    memset(buf, 0, sizeof(buf));
    buf[0] = 'U'; buf[1] = 'M'; buf[2] = 'H'; buf[3] = 'C';
    uint16_t ver = EEPROM_CAL_VERSION;
    uint16_t num = NUM_REAL_TRANSDUCER;
    memcpy(&buf[4], &ver, 2);
    memcpy(&buf[6], &num, 2);
    memcpy(&buf[8], calib_us, EEPROM_CAL_DATA_BYTES);
    uint32_t cs = eeprom_checksum(buf, EEPROM_CAL_CS_OFFSET);
    memcpy(&buf[EEPROM_CAL_CS_OFFSET], &cs, 4);
    return EEPROM_WriteBuffer(EEPROM_CAL_OFFSET, buf, sizeof(buf));
}

uint8_t EEPROM_LoadCalibration(float calib_us_out[NUM_REAL_TRANSDUCER])
{
    uint8_t buf[EEPROM_CAL_SIZE];
    if (!EEPROM_ReadBuffer(EEPROM_CAL_OFFSET, buf, sizeof(buf)))
        return 0;
    /* validate */
    uint32_t magic;
    memcpy(&magic, &buf[0], 4);
    if (magic != EEPROM_CAL_MAGIC) return 0;
    uint16_t ver;
    memcpy(&ver, &buf[4], 2);
    if (ver != EEPROM_CAL_VERSION) return 0;
    uint32_t cs_stored, cs_calc;
    memcpy(&cs_stored, &buf[EEPROM_CAL_CS_OFFSET], 4);
    cs_calc = eeprom_checksum(buf, EEPROM_CAL_CS_OFFSET);
    if (cs_stored != cs_calc) return 0;
    memcpy(calib_us_out, &buf[8], EEPROM_CAL_DATA_BYTES);
    return 1;
}

/* ---- Asynchronous calibration save state machine ----
 * EEPROM_CAL_SIZE (252 bytes) is written in 16-byte pages; each page write
 * needs a ~5ms window before the device ACKs again. We avoid the 80ms
 * blocking HAL_Delay in EEPROM_SaveCalibration by writing one page per Poll
 * call and gating the next write on the page-write delay using HAL_GetTick. */
static struct {
    uint8_t  buf[EEPROM_CAL_SIZE];
    uint16_t offset;        /* byte offset into buf / EEPROM */
    uint32_t last_write_tick;
    uint8_t  waiting;       /* 1 = in the 5ms page-write settle window */
    EEPROM_SaveState state;
} eeprom_save;

void EEPROM_SaveCalibration_Start(const float calib_us[NUM_REAL_TRANSDUCER])
{
    memset(eeprom_save.buf, 0, sizeof(eeprom_save.buf));
    eeprom_save.buf[0] = 'U'; eeprom_save.buf[1] = 'M';
    eeprom_save.buf[2] = 'H'; eeprom_save.buf[3] = 'C';
    uint16_t ver = EEPROM_CAL_VERSION;
    uint16_t num = NUM_REAL_TRANSDUCER;
    memcpy(&eeprom_save.buf[4], &ver, 2);
    memcpy(&eeprom_save.buf[6], &num, 2);
    memcpy(&eeprom_save.buf[8], calib_us, EEPROM_CAL_DATA_BYTES);
    uint32_t cs = eeprom_checksum(eeprom_save.buf, EEPROM_CAL_CS_OFFSET);
    memcpy(&eeprom_save.buf[EEPROM_CAL_CS_OFFSET], &cs, 4);

    eeprom_save.offset          = 0;
    eeprom_save.last_write_tick = 0;
    eeprom_save.waiting         = 0;
    eeprom_save.state           = EEPROM_SAVE_BUSY;
}

EEPROM_SaveState EEPROM_SaveCalibration_Poll(void)
{
    if (eeprom_save.state != EEPROM_SAVE_BUSY)
        return eeprom_save.state;

    /* If we just wrote a page, wait for the EEPROM page-write cycle (5ms)
     * before issuing the next write. HAL_GetTick is driven by SysTick (1ms)
     * and runs in the main loop, so this does not block. */
    if (eeprom_save.waiting) {
        if ((HAL_GetTick() - eeprom_save.last_write_tick) < EEPROM_WRITE_DELAY_MS)
            return EEPROM_SAVE_BUSY;
        eeprom_save.waiting = 0;
    }

    if (eeprom_save.offset >= sizeof(eeprom_save.buf)) {
        eeprom_save.state = EEPROM_SAVE_DONE;
        return EEPROM_SAVE_DONE;
    }

    /* Write the next page-aligned chunk. */
    uint16_t addr  = EEPROM_CAL_OFFSET + eeprom_save.offset;
    uint16_t chunk = EEPROM_PAGE_SIZE - (addr % EEPROM_PAGE_SIZE);
    uint16_t remaining = (uint16_t)(sizeof(eeprom_save.buf) - eeprom_save.offset);
    if (chunk > remaining) chunk = remaining;

    if (HAL_I2C_Mem_Write(&hi2c3, EEPROM_DEV_ADDR, addr, I2C_MEMADD_SIZE_8BIT,
                          &eeprom_save.buf[eeprom_save.offset], chunk, 10) != HAL_OK) {
        eeprom_save.state = EEPROM_SAVE_FAIL;
        return EEPROM_SAVE_FAIL;
    }

    eeprom_save.offset         += chunk;
    eeprom_save.last_write_tick = HAL_GetTick();
    eeprom_save.waiting         = 1;
    return EEPROM_SAVE_BUSY;
}
