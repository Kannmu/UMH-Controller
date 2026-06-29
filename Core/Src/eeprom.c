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
 *  6:   num       (uint16_t) = 60
 *  8:   calib_us[60] (float, 240 bytes) → offset 8..247
 *  248: checksum  (uint32_t) = XOR over bytes 0..247
 *  Total 252 bytes */
#define EEPROM_CAL_MAGIC  0x43484D55  /* 'UMHC' little-endian */
#define EEPROM_CAL_VERSION 1u
#define EEPROM_CAL_OFFSET  0u
#define EEPROM_CAL_SIZE    252u

static uint32_t eeprom_checksum(const uint8_t *data, uint16_t len)
{
    uint32_t c = 0;
    for (uint16_t i = 0; i < len; i++) c = (c << 1) ^ data[i];  /* simplified rolling XOR */
    return c;
}

uint8_t EEPROM_SaveCalibration(const float calib_us[60])
{
    uint8_t buf[EEPROM_CAL_SIZE];
    memset(buf, 0, sizeof(buf));
    buf[0] = 'U'; buf[1] = 'M'; buf[2] = 'H'; buf[3] = 'C';
    uint16_t ver = EEPROM_CAL_VERSION;
    uint16_t num = 60;
    memcpy(&buf[4], &ver, 2);
    memcpy(&buf[6], &num, 2);
    memcpy(&buf[8], calib_us, 240);
    uint32_t cs = eeprom_checksum(buf, 248);
    memcpy(&buf[248], &cs, 4);
    return EEPROM_WriteBuffer(EEPROM_CAL_OFFSET, buf, sizeof(buf));
}

uint8_t EEPROM_LoadCalibration(float calib_us_out[60])
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
    memcpy(&cs_stored, &buf[248], 4);
    cs_calc = eeprom_checksum(buf, 248);
    if (cs_stored != cs_calc) return 0;
    memcpy(calib_us_out, &buf[8], 240);
    return 1;
}
