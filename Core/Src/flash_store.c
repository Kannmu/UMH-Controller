#include "flash_store.h"
#include <string.h>

#define FLASH_STORE_COMMIT_VALUE 0x00000000u

static uint32_t crc32_update(uint32_t crc, const uint8_t *bytes, uint32_t length)
{
  uint32_t i;
  uint8_t bit;
  for (i = 0u; i < length; ++i) {
    crc ^= bytes[i];
    for (bit = 0u; bit < 8u; ++bit)
      crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(crc & 1u));
  }
  return crc;
}

uint32_t flash_store_crc32(const void *data, uint32_t length)
{
  if (data == NULL) return 0u;
  return ~crc32_update(0xFFFFFFFFu, (const uint8_t *)data, length);
}

static int object_address_valid(uint32_t address, uint32_t length)
{
  uint32_t bank;
  uint32_t bank_base;
  if (length == 0u || length > FLASH_STORE_OBJECT_SIZE ||
      address < FLASH_STORE_OBJECT_BASE || address >= FLASH_NOR_TOTAL_SIZE)
    return 0;
  bank = (address - FLASH_STORE_OBJECT_BASE) / FLASH_STORE_DATA_BANK_SIZE;
  if (bank > 1u) return 0;
  bank_base = FLASH_STORE_DATA_BANK_BASE(bank);
  return address >= bank_base &&
         length <= FLASH_STORE_DATA_BANK_SIZE - (address - bank_base);
}

void flash_store_init(flash_store_t *store)
{
  if (store != NULL) {
    memset(store, 0, sizeof(*store));
  }
}

static int record_valid(const flash_store_record_t *record)
{
  uint32_t end;
  if (record == NULL) return 0;
  if (record->magic != FLASH_STORE_MAGIC || record->commit != FLASH_STORE_COMMIT_VALUE ||
      record->generation == 0u) return 0;
  if (record->type == FLASH_STORE_TYPE_TOMBSTONE) {
    return record->length == 0u && record->address == 0u;
  }
  if (!object_address_valid(record->address, record->length)) return 0;
  end = record->address + record->length;
  if (end < record->address || end > FLASH_NOR_TOTAL_SIZE) return 0;
  return 1;
}

static int object_index(const flash_store_t *store, uint32_t object_id)
{
  uint16_t i;
  for (i = 0u; i < store->count; ++i)
    if (store->records[i].object_id == object_id) return (int)i;
  return -1;
}

static void remove_index(flash_store_t *store, uint16_t index)
{
  if (index < store->count) store->records[index] = store->records[--store->count];
}

static int object_data_valid(const flash_store_record_t *record)
{
  static uint8_t buffer[FLASH_NOR_PAGE_SIZE];
  uint32_t address;
  uint32_t remaining;
  uint32_t chunk;
  uint32_t crc = 0xFFFFFFFFu;
  if (record == NULL || record->length == 0u) return 0;
  address = record->address;
  remaining = record->length;
  while (remaining != 0u) {
    chunk = remaining > sizeof(buffer) ? sizeof(buffer) : remaining;
    if (flash_nor_read_dma(address, buffer, chunk) != FLASH_NOR_OK) return 0;
    crc = crc32_update(crc, buffer, chunk);
    address += chunk;
    remaining -= chunk;
  }
  return ~crc == record->crc32;
}

static int append_metadata(flash_store_t *store, flash_store_record_t *record)
{
  uint32_t address;
  uint32_t commit = FLASH_STORE_COMMIT_VALUE;
  if (store == NULL || record == NULL) return -1;
  if (store->next_metadata_address + sizeof(*record) >
      FLASH_STORE_METADATA_BASE + FLASH_STORE_METADATA_SIZE) {
    return -2;
  }
  address = store->next_metadata_address;
  record->commit = 0xFFFFFFFFu;
  if (flash_nor_page_program(address, record, sizeof(*record)) != FLASH_NOR_OK) return -3;
  if (flash_nor_page_program(address + offsetof(flash_store_record_t, commit),
                             &commit, sizeof(commit)) != FLASH_NOR_OK) return -4;
  record->commit = commit;
  store->next_metadata_address = address + sizeof(*record);
  return 0;
}

static int rotate_metadata(flash_store_t *store)
{
  static flash_store_record_t snapshot[FLASH_STORE_MAX_OBJECTS];
  uint16_t count;
  uint16_t i;
  if (store == NULL) return -1;
  count = store->count;
  memcpy(snapshot, store->records, count * sizeof(snapshot[0]));
  if (flash_nor_erase_block(FLASH_STORE_METADATA_BASE) != FLASH_NOR_OK) return -2;
  store->next_metadata_address = FLASH_STORE_METADATA_BASE;
  for (i = 0u; i < count; ++i) {
    if (append_metadata(store, &snapshot[i]) != 0) return -3;
  }
  return 0;
}

static int compact_data(flash_store_t *store)
{
  static flash_store_record_t snapshot[FLASH_STORE_MAX_OBJECTS];
  static uint8_t buffer[FLASH_NOR_PAGE_SIZE];
  uint32_t source;
  uint32_t target;
  uint32_t remaining;
  uint32_t chunk;
  uint16_t i;
  uint8_t target_bank;

  if (store == NULL || store->count == 0u) return -1;
  target_bank = (uint8_t)(store->data_bank ^ 1u);
  target = FLASH_STORE_DATA_BANK_BASE(target_bank);
  for (i = 0u; i < store->count; ++i) {
    if ((uint8_t)((store->records[i].address - FLASH_STORE_OBJECT_BASE) /
                  FLASH_STORE_DATA_BANK_SIZE) != store->data_bank)
      return -6;
  }
  for (source = target; source < target + FLASH_STORE_DATA_BANK_SIZE;
       source += FLASH_NOR_BLOCK_SIZE) {
    if (flash_nor_erase_block(source) != FLASH_NOR_OK) return -2;
  }

  memcpy(snapshot, store->records, store->count * sizeof(snapshot[0]));
  for (i = 0u; i < store->count; ++i) {
    source = snapshot[i].address;
    remaining = snapshot[i].length;
    snapshot[i].address = target;
    while (remaining != 0u) {
      chunk = remaining > sizeof(buffer) ? sizeof(buffer) : remaining;
      if (flash_nor_read_dma(source, buffer, chunk) != FLASH_NOR_OK ||
          flash_nor_page_program(target, buffer, chunk) != FLASH_NOR_OK)
        return -3;
      source += chunk;
      target += chunk;
      remaining -= chunk;
    }
    snapshot[i].generation = store->next_generation++;
  }

  if (store->next_metadata_address + store->count * sizeof(snapshot[0]) >
      FLASH_STORE_METADATA_BASE + FLASH_STORE_METADATA_SIZE) {
    memcpy(store->records, snapshot, store->count * sizeof(snapshot[0]));
    if (rotate_metadata(store) != 0) return -4;
  } else {
    for (i = 0u; i < store->count; ++i)
      if (append_metadata(store, &snapshot[i]) != 0) return -5;
    memcpy(store->records, snapshot, store->count * sizeof(snapshot[0]));
  }
  store->data_bank = target_bank;
  store->next_address = target;
  return 0;
}

int flash_store_mount(flash_store_t *store)
{
  uint32_t address = FLASH_STORE_METADATA_BASE;
  flash_store_record_t record;
  uint16_t i;
  uint32_t newest_data_generation = 0u;
  uint32_t active_end;
  uint8_t active_bank = 0u;
  if (store == NULL) return -1;
  store->count = 0u;
  store->next_generation = 1u;
  store->data_bank = 0u;
  store->next_address = FLASH_STORE_DATA_BANK_BASE(0u);
  store->next_metadata_address = FLASH_STORE_METADATA_BASE;
  while (address + sizeof(record) <= FLASH_STORE_METADATA_BASE + FLASH_STORE_METADATA_SIZE) {
    if (flash_nor_read_dma(address, &record, sizeof(record)) != FLASH_NOR_OK) return -2;
    if (record.magic == 0xFFFFFFFFu) break;
    if (record_valid(&record)) {
      if (record.generation >= store->next_generation) store->next_generation = record.generation + 1u;
      if (record.type != FLASH_STORE_TYPE_TOMBSTONE && !object_data_valid(&record)) {
        address += sizeof(record);
        store->next_metadata_address = address;
        continue;
      }
      i = (uint16_t)object_index(store, record.object_id);
      if (record.type == FLASH_STORE_TYPE_TOMBSTONE) {
        if (i < store->count && store->records[i].generation < record.generation) remove_index(store, i);
      } else if (i < store->count) {
        if (store->records[i].generation < record.generation) store->records[i] = record;
      } else if (store->count < FLASH_STORE_MAX_OBJECTS) {
        store->records[store->count++] = record;
      }
      if (record.type != FLASH_STORE_TYPE_TOMBSTONE &&
          record.generation >= newest_data_generation) {
        newest_data_generation = record.generation;
        active_bank = (uint8_t)((record.address - FLASH_STORE_OBJECT_BASE) /
                                FLASH_STORE_DATA_BANK_SIZE);
      }
    }
    address += sizeof(record);
    store->next_metadata_address = address;
  }
  if (active_bank > 1u) active_bank = 0u;
  store->data_bank = active_bank;
  active_end = FLASH_STORE_DATA_BANK_BASE(active_bank);
  for (i = 0u; i < store->count; ++i) {
    if (((store->records[i].address - FLASH_STORE_OBJECT_BASE) /
         FLASH_STORE_DATA_BANK_SIZE) == active_bank &&
        store->records[i].address + store->records[i].length > active_end)
      active_end = store->records[i].address + store->records[i].length;
  }
  store->next_address = active_end;
  store->mounted = 1u;
  return 0;
}

static const flash_store_record_t *find_record(const flash_store_t *store, uint32_t object_id)
{
  uint16_t i;
  if (store == NULL) return NULL;
  for (i = 0u; i < store->count; ++i) if (store->records[i].object_id == object_id) return &store->records[i];
  return NULL;
}

int flash_store_read(const flash_store_t *store, uint32_t object_id,
                     void *data, uint32_t capacity, uint32_t *length)
{
  return flash_store_read_range(store, object_id, 0u, data, capacity, length, NULL);
}

int flash_store_read_range(const flash_store_t *store, uint32_t object_id,
                           uint32_t offset, void *data, uint32_t capacity,
                           uint32_t *length, uint32_t *total_length)
{
  const flash_store_record_t *record = find_record(store, object_id);
  uint32_t available;
  uint32_t read_length;
  if (record == NULL || data == NULL || capacity == 0u || offset >= record->length) return -1;
  available = record->length - offset;
  read_length = available < capacity ? available : capacity;
  if (flash_nor_read_dma(record->address + offset, data, read_length) != FLASH_NOR_OK) return -2;
  if (offset == 0u && read_length == record->length &&
      flash_store_crc32(data, read_length) != record->crc32) return -3;
  if (length != NULL) *length = read_length;
  if (total_length != NULL) *total_length = record->length;
  return 0;
}

int flash_store_write(flash_store_t *store, uint32_t object_id, uint16_t type,
                      uint16_t version, const void *data, uint32_t length)
{
  flash_store_record_t record;
  const flash_store_record_t *old;
  int old_index;
  uint32_t crc;
  if (store == NULL || data == NULL || length == 0u || length > FLASH_STORE_OBJECT_SIZE || store->mounted == 0u) return -1;
  if (type == FLASH_STORE_TYPE_TOMBSTONE) return -1;
  crc = flash_store_crc32(data, length);
  old = find_record(store, object_id);
  if (old != NULL && old->type == type && old->version == version &&
      old->length == length && old->crc32 == crc) return 0;
  old_index = object_index(store, object_id);
  if (old == NULL && store->count >= FLASH_STORE_MAX_OBJECTS) return -3;
  if (store->next_address > FLASH_STORE_DATA_BANK_BASE(store->data_bank) +
      FLASH_STORE_OBJECT_SIZE - length) {
    if (compact_data(store) != 0) return -2;
  }
  if (store->next_address + length > FLASH_STORE_DATA_BANK_BASE(store->data_bank) +
      FLASH_STORE_OBJECT_SIZE) return -2;
  if (store->next_metadata_address + sizeof(record) >
      FLASH_STORE_METADATA_BASE + FLASH_STORE_METADATA_SIZE && rotate_metadata(store) != 0) return -5;
  record.magic = FLASH_STORE_MAGIC;
  record.object_id = object_id;
  record.type = type;
  record.version = version;
  record.length = length;
  record.address = store->next_address;
  record.generation = store->next_generation;
  record.crc32 = crc;
  if (flash_nor_page_program(record.address, data, length) != FLASH_NOR_OK) return -4;
  /* The object area is append-only. Keep consumed space even if the
   * subsequent metadata commit fails, so a retry cannot overwrite orphaned
   * data that may already be valid in NOR. */
  store->next_address = record.address + record.length;
  if (append_metadata(store, &record) != 0) return -6;
  store->next_generation++;
  if (old_index >= 0) store->records[old_index] = record;
  else store->records[store->count++] = record;
  return 0;
}

int flash_store_delete(flash_store_t *store, uint32_t object_id)
{
  flash_store_record_t tombstone;
  int index;
  if (store == NULL) return -1;
  index = object_index(store, object_id);
  if (index < 0) return -2;
  memset(&tombstone, 0, sizeof(tombstone));
  tombstone.magic = FLASH_STORE_MAGIC;
  tombstone.object_id = object_id;
  tombstone.type = FLASH_STORE_TYPE_TOMBSTONE;
  tombstone.generation = store->next_generation;
  if (store->next_metadata_address + sizeof(tombstone) >
      FLASH_STORE_METADATA_BASE + FLASH_STORE_METADATA_SIZE && rotate_metadata(store) != 0) return -3;
  if (append_metadata(store, &tombstone) != 0) return -4;
  store->next_generation++;
  remove_index(store, (uint16_t)index);
  return 0;
}

uint16_t flash_store_list(const flash_store_t *store, flash_store_record_t *records,
                          uint16_t capacity)
{
  uint16_t count;
  if (store == NULL || records == NULL) return 0u;
  count = store->count < capacity ? store->count : capacity;
  memcpy(records, store->records, count * sizeof(records[0]));
  return count;
}
