#include "eeprom_profile.h"
#include "flash_store.h"
#include "i2c_bus.h"
#include "cmsis_os.h"
#include <string.h>

#define EEPROM_COMMIT_VALUE 0x00000000u
#define EEPROM_READY_TIMEOUT 20u
#define EEPROM_IO_RETRIES 3u

/* First-generation record layout, kept only to migrate an existing EEPROM
 * in place.  The common prefix is byte-identical to version 2. */
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
} eeprom_profile_record_v1_t;

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

/* Bit-bang 9 clocks and a STOP to release a slave that is holding SDA/SCL,
 * then re-init the I2C peripheral.  Used only after a bus timeout. */
static void eeprom_i2c_recover(I2C_HandleTypeDef *i2c)
{
  GPIO_InitTypeDef gpio = {0};
  uint8_t i;
  if (i2c == NULL) return;
  (void)HAL_I2C_DeInit(i2c);
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOB, &gpio);
  gpio.Pin = GPIO_PIN_15;
  HAL_GPIO_Init(GPIOA, &gpio);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  for (i = 0u; i < 9u; ++i) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_Delay(1u);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_Delay(1u);
  }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_Delay(1u);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_Delay(1u);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(1u);
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);
  MX_I2C1_Init();
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
      eeprom_i2c_recover(i2c);
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
      HAL_StatusTypeDef wr;
      wr = HAL_I2C_Mem_Write(i2c, (uint16_t)(eeprom_address(address) << 1), address & 0xFFu,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t *)src, chunk, 100u);
      if (wr == HAL_OK) {
        HAL_Delay(6u); /* AT24C16 write-cycle guard */
        if (HAL_I2C_IsDeviceReady(i2c, (uint16_t)(eeprom_address(address) << 1),
                                  EEPROM_IO_RETRIES, EEPROM_READY_TIMEOUT * 4u) == HAL_OK) break;
      }
      eeprom_i2c_recover(i2c);
      HAL_Delay(1u);
    }
    if (attempt == EEPROM_IO_RETRIES) return -1;
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
  record->version = EEPROM_PROFILE_VERSION;
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

/* Returns the validated record version, 0 when the copy is not usable. */
static uint16_t record_version(const eeprom_profile_record_t *record)
{
  uint32_t legacy_crc;
  if (record->magic != EEPROM_PROFILE_MAGIC) return 0u;
  if (record->version >= EEPROM_PROFILE_VERSION) {
    if (record->commit != EEPROM_COMMIT_VALUE ||
        record->payload_length != offsetof(eeprom_profile_record_t, crc32)) return 0u;
    return flash_store_crc32(record, record->payload_length) == record->crc32 ?
           record->version : 0u;
  }
  if (record->version == 1u && record->payload_length == EEPROM_PROFILE_V1_PAYLOAD_LENGTH) {
    memcpy(&legacy_crc, ((const uint8_t *)record) + EEPROM_PROFILE_V1_PAYLOAD_LENGTH,
           sizeof(legacy_crc));
    return flash_store_crc32(record, EEPROM_PROFILE_V1_PAYLOAD_LENGTH) == legacy_crc ?
           1u : 0u;
  }
  return 0u;
}

static int record_valid(const eeprom_profile_record_t *record)
{
  return record_version(record) >= EEPROM_PROFILE_VERSION;
}

static void record_upgrade_v1(eeprom_profile_record_t *record)
{
  uint8_t *raw = (uint8_t *)record;
  memset(raw + EEPROM_PROFILE_V1_PAYLOAD_LENGTH, 0,
         offsetof(eeprom_profile_record_t, crc32) - EEPROM_PROFILE_V1_PAYLOAD_LENGTH);
  record->version = EEPROM_PROFILE_VERSION;
  record->payload_length = (uint16_t)offsetof(eeprom_profile_record_t, crc32);
  record->crc32 = flash_store_crc32(record, record->payload_length);
  record->commit = EEPROM_COMMIT_VALUE;
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
  else {
    uint16_t first_version = record_version(&first);
    if (first_version == 1u) { record_upgrade_v1(&first); first_valid = 1; }
    else first_valid = (first_version >= EEPROM_PROFILE_VERSION);
  }
  second_valid = eeprom_read(profile->i2c, EEPROM_PROFILE_COPY_SIZE, &second, sizeof(second)) == 0;
  if (!second_valid) eeprom_note_error(profile, (uint16_t)HAL_I2C_GetError(profile->i2c));
  else {
    uint16_t second_version = record_version(&second);
    if (second_version == 1u) { record_upgrade_v1(&second); second_valid = 1; }
    else second_valid = (second_version >= EEPROM_PROFILE_VERSION);
  }
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
  staged.version = EEPROM_PROFILE_VERSION;
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
