#ifndef FLASH_STORE_H
#define FLASH_STORE_H

#include <stdint.h>
#include "flash_nor.h"

#define FLASH_STORE_MAGIC 0x554D4837u
#define FLASH_STORE_METADATA_BASE 0u
#define FLASH_STORE_METADATA_SIZE (64u * 1024u)
#define FLASH_STORE_OBJECT_BASE FLASH_STORE_METADATA_SIZE
#define FLASH_STORE_DATA_REGION_SIZE (FLASH_NOR_TOTAL_SIZE - FLASH_STORE_OBJECT_BASE)
/* Two erase-aligned banks allow live objects to be copied before the old
 * bank is reclaimed. Playback data is never routed through this store. */
#define FLASH_STORE_DATA_BANK_SIZE \
  ((FLASH_STORE_DATA_REGION_SIZE / 2u) & ~(FLASH_NOR_BLOCK_SIZE - 1u))
#define FLASH_STORE_DATA_BANK_BASE(bank) \
  (FLASH_STORE_OBJECT_BASE + ((uint32_t)(bank) * FLASH_STORE_DATA_BANK_SIZE))
#define FLASH_STORE_OBJECT_SIZE FLASH_STORE_DATA_BANK_SIZE
#define FLASH_STORE_MAX_OBJECTS 64u
#define FLASH_STORE_TYPE_TOMBSTONE 0xFFFFu

_Static_assert(FLASH_STORE_DATA_BANK_SIZE >= FLASH_NOR_BLOCK_SIZE,
               "flash object bank must contain one erase block");

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint32_t object_id;
  uint16_t type;
  uint16_t version;
  uint32_t length;
  uint32_t address;
  uint32_t generation;
  uint32_t crc32;
  uint32_t commit;
} flash_store_record_t;

typedef struct {
  flash_store_record_t records[FLASH_STORE_MAX_OBJECTS];
  uint16_t count;
  uint32_t next_address;
  uint32_t next_metadata_address;
  uint32_t next_generation;
  uint8_t data_bank;
  uint8_t mounted;
} flash_store_t;

void flash_store_init(flash_store_t *store);
int flash_store_mount(flash_store_t *store);
int flash_store_read(const flash_store_t *store, uint32_t object_id,
                     void *data, uint32_t capacity, uint32_t *length);
int flash_store_read_range(const flash_store_t *store, uint32_t object_id,
                           uint32_t offset, void *data, uint32_t capacity,
                           uint32_t *length, uint32_t *total_length);
int flash_store_write(flash_store_t *store, uint32_t object_id, uint16_t type,
                      uint16_t version, const void *data, uint32_t length);
int flash_store_delete(flash_store_t *store, uint32_t object_id);
uint16_t flash_store_list(const flash_store_t *store, flash_store_record_t *records,
                          uint16_t capacity);
uint32_t flash_store_crc32(const void *data, uint32_t length);

#endif
