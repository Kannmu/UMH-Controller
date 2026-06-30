#pragma once
#include "main.h"
#include "transducer.h"

#define EEPROM_DEV_ADDR  (0x50 << 1)
#define EEPROM_SIZE       2048
#define EEPROM_PAGE_SIZE  16
#define EEPROM_WRITE_DELAY_MS 5

uint8_t EEPROM_WriteByte(uint16_t addr, uint8_t b);
uint8_t EEPROM_ReadByte(uint16_t addr, uint8_t *b);
uint8_t EEPROM_WriteBuffer(uint16_t addr, const uint8_t *data, uint16_t len);
uint8_t EEPROM_ReadBuffer(uint16_t addr, uint8_t *data, uint16_t len);

/* Calibration persistence helpers */
uint8_t EEPROM_SaveCalibration(const float calib_us[NUM_REAL_TRANSDUCER]);
uint8_t EEPROM_LoadCalibration(float calib_us_out[NUM_REAL_TRANSDUCER]);  /* returns 1 if valid */
