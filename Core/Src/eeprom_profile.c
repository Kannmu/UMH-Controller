#include "eeprom_profile.h"
#include "flash_store.h"
#include "i2c_bus.h"
#include "cmsis_os.h"
#include <string.h>

#define EEPROM_COMMIT_VALUE 0x00000000u
#define EEPROM_READY_TIMEOUT 20u
#define EEPROM_IO_RETRIES 3u

static void eeprom_note_error(eeprom_profile_t *profile, uint16_t error)
{
  if (profile == NULL) return;
  profile->last_error = error;
  if (profile->io_errors != UINT8_MAX) ++profile->io_errors;
}

static uint16_t eeprom_address(uint16_t address)
{
  return (uint16_t)(EEPROM_PROFILE_I2C_ADDRESS | ((address >> 8) & 0x07u));
}

static int eeprom_read(I2C_HandleTypeDef *i2c, uint16_t address, void *data, uint16_t length)
{
  uint8_t *dst = (uint8_t *)data;
  uint16_t chunk;
  uint16_t block_remaining;
  while (length != 0u) {
    block_remaining = (uint16_t)(256u - (address & 0xFFu));
    chunk = length < block_remaining ? length : block_remaining;
    if (chunk > 255u) chunk = 255u;
    uint8_t attempt;
    for (attempt = 0u; attempt < EEPROM_IO_RETRIES; ++attempt) {
      if (HAL_I2C_Mem_Read(i2c, (uint16_t)(eeprom_address(address) << 1), address & 0xFFu,
                           I2C_MEMADD_SIZE_8BIT, dst, chunk, 100u) == HAL_OK) break;
      HAL_Delay(1u);
    }
    if (attempt == EEPROM_IO_RETRIES) return -1;
    address += chunk;
    dst += chunk;
    length = (uint16_t)(length - chunk);
  }
  return 0;
}

static int eeprom_write(I2C_HandleTypeDef *i2c, uint16_t address, const void *data, uint16_t length)
{
  const uint8_t *src = (const uint8_t *)data;
  uint16_t chunk;
  uint16_t page_remaining;
  uint16_t block_remaining;
  while (length != 0u) {
    page_remaining = (uint16_t)(EEPROM_PROFILE_PAGE_SIZE - (address % EEPROM_PROFILE_PAGE_SIZE));
    chunk = length < page_remaining ? length : page_remaining;
    block_remaining = (uint16_t)(256u - (address & 0xFFu));
    if (chunk > block_remaining) chunk = block_remaining;
    uint8_t attempt;
    for (attempt = 0u; attempt < EEPROM_IO_RETRIES; ++attempt) {
      if (HAL_I2C_Mem_Write(i2c, (uint16_t)(eeprom_address(address) << 1), address & 0xFFu,
                            I2C_MEMADD_SIZE_8BIT, (uint8_t *)src, chunk, 100u) == HAL_OK) break;
      HAL_Delay(1u);
    }
    if (attempt == EEPROM_IO_RETRIES) return -1;
    if (HAL_I2C_IsDeviceReady(i2c, (uint16_t)(eeprom_address(address) << 1),
                              EEPROM_IO_RETRIES, EEPROM_READY_TIMEOUT) != HAL_OK) return -1;
    address += chunk;
    src += chunk;
    length = (uint16_t)(length - chunk);
  }
  return 0;
}

void eeprom_profile_defaults(eeprom_profile_record_t *record)
{
  uint16_t i;
  if (record == NULL) return;
  memset(record, 0, sizeof(*record));
  record->magic = EEPROM_PROFILE_MAGIC;
  record->version = 1u;
  record->payload_length = (uint16_t)(offsetof(eeprom_profile_record_t, crc32));
  record->generation = 1u;
  for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) record->gain[i] = 255u;
  for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) record->enabled[i / 8u] |= (uint8_t)(1u << (i % 8u));
  for (i = 0u; i < UMH_DEVICE_RGB_COUNT; ++i) {
    record->rgb_gain[i][0] = 255u;
    record->rgb_gain[i][1] = 255u;
    record->rgb_gain[i][2] = 255u;
  }
  record->crc32 = flash_store_crc32(record, record->payload_length);
  record->commit = EEPROM_COMMIT_VALUE;
}

static int record_valid(const eeprom_profile_record_t *record)
{
  if (record->magic != EEPROM_PROFILE_MAGIC || record->commit != EEPROM_COMMIT_VALUE ||
      record->payload_length != offsetof(eeprom_profile_record_t, crc32)) return 0;
  return flash_store_crc32(record, record->payload_length) == record->crc32;
}

void eeprom_profile_init(eeprom_profile_t *profile, I2C_HandleTypeDef *i2c)
{
  if (profile == NULL) return;
  memset(profile, 0, sizeof(*profile));
  profile->i2c = i2c;
  profile->initialized = 1u;
}

int eeprom_profile_load(eeprom_profile_t *profile)
{
  eeprom_profile_record_t first;
  eeprom_profile_record_t second;
  int first_valid;
  int second_valid;
  if (profile == NULL || profile->i2c == NULL) return -1;
  profile->present = 0u;
  if (i2c_bus_lock(osWaitForever) != 0) {
    eeprom_note_error(profile, 0xFFFFu);
    return -2;
  }
  if (HAL_I2C_IsDeviceReady(profile->i2c, (uint16_t)(EEPROM_PROFILE_I2C_ADDRESS << 1),
                            EEPROM_IO_RETRIES, EEPROM_READY_TIMEOUT) != HAL_OK) {
    eeprom_note_error(profile, (uint16_t)HAL_I2C_GetError(profile->i2c));
    i2c_bus_unlock();
    return -1;
  }
  profile->present = 1u;
  first_valid = eeprom_read(profile->i2c, 0u, &first, sizeof(first)) == 0;
  if (!first_valid) eeprom_note_error(profile, (uint16_t)HAL_I2C_GetError(profile->i2c));
  else first_valid = record_valid(&first);
  second_valid = eeprom_read(profile->i2c, EEPROM_PROFILE_COPY_SIZE, &second, sizeof(second)) == 0;
  if (!second_valid) eeprom_note_error(profile, (uint16_t)HAL_I2C_GetError(profile->i2c));
  else second_valid = record_valid(&second);
  i2c_bus_unlock();
  if (!first_valid && !second_valid) {
    eeprom_profile_defaults(&profile->record);
    profile->active_copy = 0u;
    profile->valid = 0u;
    return 1;
  }
  if (second_valid && (!first_valid || second.generation > first.generation)) {
    profile->record = second;
    profile->active_copy = 1u;
  } else {
    profile->record = first;
    profile->active_copy = 0u;
  }
  profile->valid = 1u;
  return 0;
}

int eeprom_profile_commit(eeprom_profile_t *profile, const eeprom_profile_record_t *record)
{
  eeprom_profile_record_t staged;
  uint16_t base;
  uint32_t commit = EEPROM_COMMIT_VALUE;
  if (profile == NULL || profile->i2c == NULL || record == NULL) return -1;
  if (profile->present == 0u) return -2;
  staged = *record;
  staged.magic = EEPROM_PROFILE_MAGIC;
  staged.payload_length = (uint16_t)offsetof(eeprom_profile_record_t, crc32);
  staged.generation = profile->record.generation + 1u;
  staged.crc32 = flash_store_crc32(&staged, staged.payload_length);
  staged.commit = 0xFFFFFFFFu;
  base = profile->active_copy == 0u ? EEPROM_PROFILE_COPY_SIZE : 0u;
  if (i2c_bus_lock(osWaitForever) != 0) return -2;
  if (eeprom_write(profile->i2c, base, &staged, sizeof(staged)) != 0 ||
      eeprom_write(profile->i2c, (uint16_t)(base + offsetof(eeprom_profile_record_t, commit)), &commit, sizeof(commit)) != 0) {
    eeprom_note_error(profile, (uint16_t)HAL_I2C_GetError(profile->i2c));
    i2c_bus_unlock();
    return -3;
  }
  if (eeprom_read(profile->i2c, base, &staged, sizeof(staged)) != 0) {
    eeprom_note_error(profile, (uint16_t)HAL_I2C_GetError(profile->i2c));
    i2c_bus_unlock();
    return -4;
  }
  if (!record_valid(&staged)) {
    i2c_bus_unlock();
    return -4;
  }
  i2c_bus_unlock();
  profile->record = staged;
  profile->active_copy = (uint8_t)(base == 0u ? 0u : 1u);
  profile->valid = 1u;
  profile->present = 1u;
  return 0;
}

const eeprom_profile_record_t *eeprom_profile_current(const eeprom_profile_t *profile)
{
  return profile != NULL ? &profile->record : NULL;
}
