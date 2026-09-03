#ifndef EEPROM_PROFILE_H
#define EEPROM_PROFILE_H

#include <stdint.h>
#include "i2c.h"
#include "device_profile.h"

#define EEPROM_PROFILE_SIZE 2048u
#define EEPROM_PROFILE_COPY_SIZE 1024u
#define EEPROM_PROFILE_PAGE_SIZE 16u
#define EEPROM_PROFILE_I2C_ADDRESS 0x50u
#define EEPROM_PROFILE_MAGIC 0x45505237u

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint16_t version;
  uint16_t payload_length;
  uint32_t generation;
  uint16_t phase[UMH_DEVICE_CHANNEL_COUNT];
  uint8_t gain[UMH_DEVICE_CHANNEL_COUNT];
  uint8_t enabled[UMH_DEVICE_CHANNEL_BITMAP_BYTES];
  uint8_t rgb_gain[UMH_DEVICE_RGB_COUNT][3];
  uint8_t digital_default;
  uint8_t reserved[3];
  uint32_t crc32;
  uint32_t commit;
} eeprom_profile_record_t;

_Static_assert(sizeof(eeprom_profile_record_t) < EEPROM_PROFILE_COPY_SIZE,
               "AT24C16 profile copy must fit one half");

typedef struct {
  I2C_HandleTypeDef *i2c;
  eeprom_profile_record_t record;
  uint8_t active_copy;
  uint8_t valid;
  uint8_t initialized;
} eeprom_profile_t;

void eeprom_profile_init(eeprom_profile_t *profile, I2C_HandleTypeDef *i2c);
int eeprom_profile_load(eeprom_profile_t *profile);
int eeprom_profile_commit(eeprom_profile_t *profile, const eeprom_profile_record_t *record);
void eeprom_profile_defaults(eeprom_profile_record_t *record);
const eeprom_profile_record_t *eeprom_profile_current(const eeprom_profile_t *profile);

#endif
