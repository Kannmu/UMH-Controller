#include "flash_library.h"
#include "crc.h"
#include <string.h>

#define FLASH_LIBRARY_FAST_READ 0x0Bu
#define FLASH_LIBRARY_BLANK     0xFFFFFFFFu

static void flash_library_cs_low(void)
{
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
}

static void flash_library_cs_high(void)
{
  HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
}

static uint32_t flash_library_crc32(const void *data, uint32_t length)
{
  uint32_t state = umh_crc32_update(0xFFFFFFFFu, data, length);
  return umh_crc32_finish(state);
}

static uint32_t flash_library_sector_count(uint32_t bytes)
{
  return (bytes + FLASH_NOR_SECTOR_SIZE - 1u) / FLASH_NOR_SECTOR_SIZE;
}

static uint32_t flash_library_header_crc(const flash_library_header_t *header)
{
  uint8_t raw[offsetof(flash_library_header_t, reserved)];
  uint32_t zero = 0u;
  memcpy(raw, header, sizeof(raw));
  memcpy(&raw[offsetof(flash_library_header_t, commit)], &zero, sizeof(zero));
  memcpy(&raw[offsetof(flash_library_header_t, header_crc32)], &zero, sizeof(zero));
  return flash_library_crc32(raw, sizeof(raw));
}

static int flash_library_header_valid(const flash_library_header_t *header,
                                      uint32_t address, uint8_t bank)
{
  uint32_t bank_base = FLASH_LIBRARY_BANK_BASE(bank);
  uint32_t bank_end = bank_base + FLASH_LIBRARY_BANK_SIZE;
  uint32_t expected_sectors;
  if (header == NULL) return 0;
  if (header->magic != FLASH_LIBRARY_MAGIC ||
      header->version != FLASH_LIBRARY_FORMAT_VERSION ||
      header->header_bytes != FLASH_LIBRARY_HEADER_BYTES ||
      header->generation == 0u || header->object_id == 0u) return 0;
  if (address < bank_base || address >= bank_end ||
      (address % FLASH_NOR_SECTOR_SIZE) != 0u) return 0;
  if (header->header_crc32 != flash_library_header_crc(header)) return 0;
  if (header->commit != FLASH_LIBRARY_COMMIT_VALUE &&
      header->commit != FLASH_LIBRARY_PROVISIONAL) return 0;
  if (header->sector_count == 0u || header->sector_count > FLASH_LIBRARY_BANK_SECTORS)
    return 0;
  if (address + header->sector_count * FLASH_NOR_SECTOR_SIZE > bank_end) return 0;
  if (header->object_type == FLASH_LIBRARY_OBJECT_TOMBSTONE) {
    if (header->record_count != 0u || header->records_bytes != 0u ||
        header->record_size != 0u || header->level_mode != 0u ||
        header->channel_count != 0u || header->level_offset != 0u ||
        header->records_offset != 0u || header->frame_period_us != 0u ||
        header->sector_count != 1u) return 0;
    return 1;
  }
  if (header->object_type != FLASH_LIBRARY_OBJECT_SEQUENCE) return 0;
  if (header->level_mode > FLASH_LIBRARY_LEVEL_UNIFORM) return 0;
  if (header->channel_count != FLASH_LIBRARY_CHANNEL_COUNT) return 0;
  if (header->record_size != (header->level_mode == FLASH_LIBRARY_LEVEL_UNIFORM ?
                              FLASH_LIBRARY_RECORD_PHASE_BYTES :
                              FLASH_LIBRARY_RECORD_FULL_BYTES)) return 0;
  if (header->level_offset != (header->level_mode == FLASH_LIBRARY_LEVEL_UNIFORM ?
                               FLASH_LIBRARY_LEVEL_OFFSET : 0u)) return 0;
  if (header->records_offset != FLASH_LIBRARY_RECORDS_OFFSET ||
      header->record_count == 0u || header->frame_period_us == 0u) return 0;
  if (header->record_count >
      (FLASH_LIBRARY_BANK_SIZE - FLASH_LIBRARY_RECORDS_OFFSET -
       FLASH_LIBRARY_TRAILER_BYTES) / header->record_size) return 0;
  if (header->records_bytes != header->record_count * header->record_size) return 0;
  expected_sectors = flash_library_sector_count(
      FLASH_LIBRARY_RECORDS_OFFSET + header->records_bytes +
      FLASH_LIBRARY_TRAILER_BYTES);
  if (header->sector_count != expected_sectors) return 0;
  return 1;
}

static int flash_library_entry_index(const flash_library_t *library, uint32_t object_id)
{
  uint16_t i;
  for (i = 0u; i < library->entry_count; ++i)
    if (library->entries[i].object_id == object_id) return (int)i;
  return -1;
}

static void flash_library_entry_remove(flash_library_t *library, uint16_t index)
{
  if (index < library->entry_count)
    library->entries[index] = library->entries[--library->entry_count];
}

static void flash_library_entry_fill(flash_library_entry_t *entry,
                                     const flash_library_header_t *header,
                                     uint32_t address, uint8_t bank,
                                     uint32_t payload_crc32)
{
  memset(entry, 0, sizeof(*entry));
  entry->object_id = header->object_id;
  entry->address = address;
  entry->generation = header->generation;
  entry->record_count = header->record_count;
  entry->record_size = header->record_size;
  entry->frame_period_us = header->frame_period_us;
  entry->records_bytes = header->records_bytes;
  entry->payload_crc32 = payload_crc32;
  entry->sector_count = header->sector_count;
  entry->bank = bank;
  entry->level_mode = header->level_mode;
  entry->flags = header->flags;
}

static void flash_library_directory_apply(flash_library_t *library,
                                          const flash_library_header_t *header,
                                          uint32_t address, uint8_t bank,
                                          uint32_t payload_crc32)
{
  int index = flash_library_entry_index(library, header->object_id);
  if (header->object_type == FLASH_LIBRARY_OBJECT_TOMBSTONE) {
    if (index >= 0 && library->entries[index].generation < header->generation)
      flash_library_entry_remove(library, (uint16_t)index);
    return;
  }
  if (index >= 0) {
    if (library->entries[index].generation < header->generation)
      flash_library_entry_fill(&library->entries[index], header, address, bank,
                               payload_crc32);
  } else if (library->entry_count < FLASH_LIBRARY_MAX_OBJECTS) {
    flash_library_entry_fill(&library->entries[library->entry_count++], header,
                             address, bank, payload_crc32);
  } else {
    library->truncated = 1u;
  }
}

static int flash_library_verify_payload(flash_library_t *library, uint32_t address,
                                        uint8_t level_mode, uint32_t records_bytes,
                                        uint32_t expected_crc32)
{
  uint32_t state = 0xFFFFFFFFu;
  uint32_t remaining;
  uint32_t chunk;
  uint32_t offset = 0u;
  if (level_mode == FLASH_LIBRARY_LEVEL_UNIFORM) {
    if (flash_nor_read_dma(address + FLASH_LIBRARY_LEVEL_OFFSET,
                           library->scratch,
                           FLASH_LIBRARY_RECORD_PHASE_BYTES) != FLASH_NOR_OK)
      return FLASH_LIBRARY_IO;
    state = umh_crc32_update(state, library->scratch,
                             FLASH_LIBRARY_RECORD_PHASE_BYTES);
  }
  remaining = records_bytes;
  while (remaining != 0u) {
    chunk = remaining > FLASH_NOR_PAGE_SIZE ? FLASH_NOR_PAGE_SIZE : remaining;
    if (flash_nor_read_dma(address + FLASH_LIBRARY_RECORDS_OFFSET + offset,
                           library->scratch, chunk) != FLASH_NOR_OK)
      return FLASH_LIBRARY_IO;
    state = umh_crc32_update(state, library->scratch, chunk);
    offset += chunk;
    remaining -= chunk;
  }
  return umh_crc32_finish(state) == expected_crc32 ? FLASH_LIBRARY_OK :
         FLASH_LIBRARY_CORRUPT;
}

static int flash_library_read_start(flash_library_t *library, uint32_t address,
                                    uint8_t *data, uint32_t length)
{
  uint8_t command[5];
  if (library == NULL || library->spi == NULL || data == NULL ||
      length == 0u || length > 0xFFFFu) return FLASH_LIBRARY_PARAM;
  if (address >= FLASH_NOR_TOTAL_SIZE ||
      length > FLASH_NOR_TOTAL_SIZE - address) return FLASH_LIBRARY_PARAM;
  if (HAL_SPI_GetState(library->spi) != HAL_SPI_STATE_READY)
    return FLASH_LIBRARY_BUSY;
  command[0] = FLASH_LIBRARY_FAST_READ;
  command[1] = (uint8_t)(address >> 16);
  command[2] = (uint8_t)(address >> 8);
  command[3] = (uint8_t)address;
  command[4] = 0u;
  flash_library_cs_low();
  if (HAL_SPI_Transmit(library->spi, command, sizeof(command), 10u) != HAL_OK) {
    flash_library_cs_high();
    return FLASH_LIBRARY_IO;
  }
  if (HAL_SPI_Receive_DMA(library->spi, data, (uint16_t)length) != HAL_OK) {
    (void)HAL_SPI_Abort(library->spi);
    flash_library_cs_high();
    return FLASH_LIBRARY_IO;
  }
  return FLASH_LIBRARY_OK;
}

static int flash_library_read_poll(flash_library_t *library)
{
  HAL_SPI_StateTypeDef state = HAL_SPI_GetState(library->spi);
  if (state == HAL_SPI_STATE_READY) {
    flash_library_cs_high();
    return 1;
  }
  if (state == HAL_SPI_STATE_ERROR || state == HAL_SPI_STATE_RESET) {
    (void)HAL_SPI_Abort(library->spi);
    flash_library_cs_high();
    return -1;
  }
  return 0;
}

static void flash_library_read_abort(flash_library_t *library)
{
  if (library != NULL && library->spi != NULL &&
      HAL_SPI_GetState(library->spi) != HAL_SPI_STATE_READY)
    (void)HAL_SPI_Abort(library->spi);
  flash_library_cs_high();
}

static int flash_library_reader_poll(flash_library_reader_t *reader)
{
  int result = flash_library_read_poll(reader->library);
  if (result < 0) {
    ++reader->library->stats.read_errors;
    return FLASH_LIBRARY_IO;
  }
  if (result == 0 &&
      (uint32_t)(HAL_GetTick() - reader->prefetch_tick) >
      FLASH_LIBRARY_READ_TIMEOUT_MS) {
    flash_library_read_abort(reader->library);
    ++reader->library->stats.read_errors;
    return FLASH_LIBRARY_IO;
  }
  return result;
}

static int flash_library_reader_prefetch(flash_library_reader_t *reader,
                                         uint32_t frame_index, uint8_t buffer)
{
  uint32_t frames;
  if (frame_index >= reader->record_count) return FLASH_LIBRARY_OK;
  frames = reader->record_count - frame_index;
  if (frames > FLASH_LIBRARY_CHUNK_FRAMES) frames = FLASH_LIBRARY_CHUNK_FRAMES;
  reader->prefetch_address = reader->object_address + FLASH_LIBRARY_RECORDS_OFFSET +
                             frame_index * reader->record_size;
  reader->prefetch_frames = frames;
  reader->prefetch_buffer = buffer;
  reader->prefetch_tick = HAL_GetTick();
  if (flash_library_read_start(reader->library, reader->prefetch_address,
                               reader->buffers[buffer],
                               frames * reader->record_size) != FLASH_LIBRARY_OK)
    return FLASH_LIBRARY_IO;
  reader->prefetch_in_flight = 1u;
  return FLASH_LIBRARY_OK;
}

static void flash_library_build_frame(const flash_library_reader_t *reader,
                                      const uint8_t *record, uint32_t frame_index,
                                      uint32_t sequence, umh_output_frame_t *frame)
{
  uint32_t i;
  memset(frame, 0, sizeof(*frame));
  frame->sequence = sequence;
  frame->deadline = reader->origin_us +
                    (uint64_t)frame_index * (uint64_t)reader->frame_period_us;
  frame->update_flags = UMH_FRAME_FLAG_ULTRASOUND;
  if (reader->level_mode == FLASH_LIBRARY_LEVEL_FULL) {
    /* The full record is exactly the 84 interleaved phase/level pairs of the
     * frame channel array, so one copy replaces the per-channel loop. */
    memcpy(frame->channels, record, sizeof(frame->channels));
    return;
  }
  for (i = 0u; i < FLASH_LIBRARY_CHANNEL_COUNT; ++i) {
    frame->channels[i].phase = record[i];
    frame->channels[i].level = reader->level_table[i];
  }
}

static int flash_library_copy_entry(flash_library_t *library, uint16_t index,
                                    uint8_t target_bank, uint32_t *dest_address)
{
  flash_library_entry_t *entry = &library->entries[index];
  flash_library_header_t *source = &library->scratch_header;
  flash_library_header_t destination;
  uint32_t source_address = entry->address;
  uint32_t target = *dest_address;
  uint32_t bank_end = FLASH_LIBRARY_BANK_BASE(target_bank) + FLASH_LIBRARY_BANK_SIZE;
  uint32_t trailer;
  uint32_t remaining;
  uint32_t chunk;
  uint32_t offset = 0u;
  uint32_t commit = FLASH_LIBRARY_PROVISIONAL;
  uint32_t check;
  if (flash_nor_read_dma(source_address, source, FLASH_LIBRARY_HEADER_BYTES) !=
      FLASH_NOR_OK) return FLASH_LIBRARY_IO;
  if (flash_library_header_valid(source, source_address, entry->bank) == 0 ||
      source->commit != FLASH_LIBRARY_COMMIT_VALUE ||
      source->object_type != FLASH_LIBRARY_OBJECT_SEQUENCE ||
      source->generation != entry->generation) return FLASH_LIBRARY_CORRUPT;
  if (target + source->sector_count * FLASH_NOR_SECTOR_SIZE > bank_end)
    return FLASH_LIBRARY_FULL;
  memcpy(&destination, source, sizeof(destination));
  destination.generation = library->next_generation;
  destination.commit = FLASH_LIBRARY_PROVISIONAL;
  destination.header_crc32 = 0u;
  destination.header_crc32 = flash_library_header_crc(&destination);
  if (flash_nor_page_program(target, &destination, FLASH_LIBRARY_HEADER_BYTES) !=
      FLASH_NOR_OK) return FLASH_LIBRARY_IO;
  library->stats.page_programs++;
  library->stats.bytes_programmed += FLASH_LIBRARY_HEADER_BYTES;
  if (source->level_mode == FLASH_LIBRARY_LEVEL_UNIFORM) {
    if (flash_nor_read_dma(source_address + FLASH_LIBRARY_LEVEL_OFFSET,
                           library->scratch,
                           FLASH_LIBRARY_RECORD_PHASE_BYTES) != FLASH_NOR_OK)
      return FLASH_LIBRARY_IO;
    if (flash_nor_page_program(target + FLASH_LIBRARY_LEVEL_OFFSET,
                               library->scratch,
                               FLASH_LIBRARY_RECORD_PHASE_BYTES) != FLASH_NOR_OK)
      return FLASH_LIBRARY_IO;
    library->stats.page_programs++;
    library->stats.bytes_programmed += FLASH_LIBRARY_RECORD_PHASE_BYTES;
  }
  remaining = source->records_bytes;
  while (remaining != 0u) {
    chunk = remaining > FLASH_NOR_PAGE_SIZE ? FLASH_NOR_PAGE_SIZE : remaining;
    if (flash_nor_read_dma(source_address + FLASH_LIBRARY_RECORDS_OFFSET + offset,
                           library->scratch, chunk) != FLASH_NOR_OK)
      return FLASH_LIBRARY_IO;
    if (flash_nor_page_program(target + FLASH_LIBRARY_RECORDS_OFFSET + offset,
                               library->scratch, chunk) != FLASH_NOR_OK)
      return FLASH_LIBRARY_IO;
    library->stats.page_programs++;
    library->stats.bytes_programmed += chunk;
    offset += chunk;
    remaining -= chunk;
  }
  if (flash_nor_read_dma(source_address + FLASH_LIBRARY_RECORDS_OFFSET +
                         source->records_bytes, &trailer,
                         FLASH_LIBRARY_TRAILER_BYTES) != FLASH_NOR_OK)
    return FLASH_LIBRARY_IO;
  if (flash_nor_page_program(target + FLASH_LIBRARY_RECORDS_OFFSET +
                             source->records_bytes, &trailer,
                             FLASH_LIBRARY_TRAILER_BYTES) != FLASH_NOR_OK)
    return FLASH_LIBRARY_IO;
  library->stats.page_programs++;
  library->stats.bytes_programmed += FLASH_LIBRARY_TRAILER_BYTES;
  if (flash_library_verify_payload(library, target, source->level_mode,
                                   source->records_bytes, trailer) !=
      FLASH_LIBRARY_OK) return FLASH_LIBRARY_CORRUPT;
  commit = FLASH_LIBRARY_COMMIT_VALUE;
  if (flash_nor_page_program(target + offsetof(flash_library_header_t, commit),
                             &commit, sizeof(commit)) != FLASH_NOR_OK)
    return FLASH_LIBRARY_IO;
  library->stats.page_programs++;
  library->stats.bytes_programmed += sizeof(commit);
  if (flash_nor_read_dma(target + offsetof(flash_library_header_t, commit), &check,
                         sizeof(check)) != FLASH_NOR_OK ||
      check != FLASH_LIBRARY_COMMIT_VALUE) return FLASH_LIBRARY_CORRUPT;
  entry->address = target;
  entry->bank = target_bank;
  entry->generation = destination.generation;
  *dest_address = target + source->sector_count * FLASH_NOR_SECTOR_SIZE;
  library->next_generation++;
  return FLASH_LIBRARY_OK;
}

static int flash_library_rotate(flash_library_t *library)
{
  uint8_t target_bank = (uint8_t)(library->active_bank ^ 1u);
  uint32_t target_base = FLASH_LIBRARY_BANK_BASE(target_bank);
  uint32_t target_end = target_base + FLASH_LIBRARY_BANK_SIZE;
  uint32_t dest = target_base;
  uint32_t address;
  uint16_t i;
  for (i = 0u; i < library->entry_count; ++i) {
    if (library->entries[i].bank == target_bank) {
      int result = flash_library_copy_entry(library, i, library->active_bank,
                                            &library->next_address);
      if (result != FLASH_LIBRARY_OK) {
        library->mounted = 0u;
        return result;
      }
    }
  }
  for (address = target_base; address < target_end;
       address += FLASH_NOR_BLOCK_SIZE) {
    if (flash_nor_erase_block(address) != FLASH_NOR_OK) {
      library->mounted = 0u;
      return FLASH_LIBRARY_IO;
    }
    library->stats.block_erases++;
  }
  for (i = 0u; i < library->entry_count; ++i) {
    int result = flash_library_copy_entry(library, i, target_bank, &dest);
    if (result != FLASH_LIBRARY_OK) {
      library->mounted = 0u;
      return result;
    }
  }
  library->active_bank = target_bank;
  library->next_address = dest;
  library->stats.rotations++;
  return FLASH_LIBRARY_OK;
}

static int flash_library_ensure_space(flash_library_t *library, uint32_t sectors)
{
  uint32_t bank_end = FLASH_LIBRARY_BANK_BASE(library->active_bank) +
                      FLASH_LIBRARY_BANK_SIZE;
  if (sectors > FLASH_LIBRARY_BANK_SECTORS) return FLASH_LIBRARY_FULL;
  if (library->next_address + sectors * FLASH_NOR_SECTOR_SIZE <= bank_end)
    return FLASH_LIBRARY_OK;
  if (flash_library_rotate(library) != FLASH_LIBRARY_OK)
    return FLASH_LIBRARY_FULL;
  if (library->next_address + sectors * FLASH_NOR_SECTOR_SIZE > bank_end)
    return FLASH_LIBRARY_FULL;
  return FLASH_LIBRARY_OK;
}

void flash_library_init(flash_library_t *library, SPI_HandleTypeDef *spi)
{
  if (library == NULL) return;
  memset(library, 0, sizeof(*library));
  library->spi = spi;
}

int flash_library_mount(flash_library_t *library)
{
  SPI_HandleTypeDef *spi;
  uint32_t bank_next[FLASH_LIBRARY_BANK_COUNT];
  uint32_t max_generation = 0u;
  uint32_t header_address;
  uint8_t max_bank = 0u;
  uint8_t have_generation = 0u;
  uint8_t bank;
  if (library == NULL || library->spi == NULL) return FLASH_LIBRARY_PARAM;
  if (flash_nor_is_present() == 0u) return FLASH_LIBRARY_IO;
  spi = library->spi;
  memset(library, 0, sizeof(*library));
  library->spi = spi;
  for (bank = 0u; bank < FLASH_LIBRARY_BANK_COUNT; ++bank) {
    uint32_t address = FLASH_LIBRARY_BANK_BASE(bank);
    uint32_t bank_end = address + FLASH_LIBRARY_BANK_SIZE;
    bank_next[bank] = address;
    while (address + FLASH_LIBRARY_HEADER_BYTES <= bank_end) {
      uint32_t magic;
      uint32_t payload_crc32 = 0u;
      bank_next[bank] = address;
      if (flash_nor_read_dma(address, &magic, sizeof(magic)) != FLASH_NOR_OK) {
        library->stats.read_errors++;
        return FLASH_LIBRARY_IO;
      }
      library->stats.mount_reads++;
      if (magic == FLASH_LIBRARY_BLANK) break;
      if (magic != FLASH_LIBRARY_MAGIC) {
        address += FLASH_NOR_SECTOR_SIZE;
        continue;
      }
      if (flash_nor_read_dma(address, &library->scratch_header,
                             FLASH_LIBRARY_HEADER_BYTES) != FLASH_NOR_OK) {
        library->stats.read_errors++;
        return FLASH_LIBRARY_IO;
      }
      library->stats.mount_reads++;
      if (flash_library_header_valid(&library->scratch_header, address, bank) == 0) {
        address += FLASH_NOR_SECTOR_SIZE;
        continue;
      }
      header_address = address;
      address += library->scratch_header.sector_count * FLASH_NOR_SECTOR_SIZE;
      if (library->scratch_header.commit != FLASH_LIBRARY_COMMIT_VALUE) continue;
      if (library->scratch_header.object_type == FLASH_LIBRARY_OBJECT_SEQUENCE) {
        if (flash_nor_read_dma(header_address + FLASH_LIBRARY_RECORDS_OFFSET +
                               library->scratch_header.records_bytes,
                               &payload_crc32, FLASH_LIBRARY_TRAILER_BYTES) !=
            FLASH_NOR_OK) {
          library->stats.read_errors++;
          return FLASH_LIBRARY_IO;
        }
        library->stats.mount_reads++;
      }
      flash_library_directory_apply(library, &library->scratch_header,
                                    header_address, bank, payload_crc32);
      if (have_generation == 0u ||
          library->scratch_header.generation > max_generation) {
        max_generation = library->scratch_header.generation;
        max_bank = bank;
        have_generation = 1u;
      }
    }
  }
  library->active_bank = have_generation != 0u ? max_bank : 0u;
  library->next_address = bank_next[library->active_bank];
  library->next_generation = have_generation != 0u ? max_generation + 1u : 1u;
  library->mounted = 1u;
  return FLASH_LIBRARY_OK;
}

uint16_t flash_library_object_count(const flash_library_t *library)
{
  return library != NULL ? library->entry_count : 0u;
}

const flash_library_entry_t *flash_library_object_at(const flash_library_t *library,
                                                     uint16_t index)
{
  if (library == NULL || index >= library->entry_count) return NULL;
  return &library->entries[index];
}

const flash_library_entry_t *flash_library_object_find(const flash_library_t *library,
                                                       uint32_t object_id)
{
  int index;
  if (library == NULL) return NULL;
  index = flash_library_entry_index(library, object_id);
  return index >= 0 ? &library->entries[index] : NULL;
}

const flash_library_stats_t *flash_library_stats(const flash_library_t *library)
{
  return library != NULL ? &library->stats : NULL;
}

uint32_t flash_library_free_bytes(const flash_library_t *library)
{
  uint32_t bank_end;
  uint32_t free_bytes;
  uint32_t standby_bytes = 0u;
  uint16_t i;
  if (library == NULL || library->mounted == 0u) return 0u;
  bank_end = FLASH_LIBRARY_BANK_BASE(library->active_bank) +
             FLASH_LIBRARY_BANK_SIZE;
  free_bytes = bank_end - library->next_address;
  for (i = 0u; i < library->entry_count; ++i)
    if (library->entries[i].bank != library->active_bank)
      standby_bytes += library->entries[i].sector_count * FLASH_NOR_SECTOR_SIZE;
  return free_bytes > standby_bytes ? free_bytes - standby_bytes : 0u;
}

uint32_t flash_library_frame_capacity(uint8_t level_mode)
{
  uint32_t record_size;
  if (level_mode == FLASH_LIBRARY_LEVEL_UNIFORM)
    record_size = FLASH_LIBRARY_RECORD_PHASE_BYTES;
  else if (level_mode == FLASH_LIBRARY_LEVEL_FULL)
    record_size = FLASH_LIBRARY_RECORD_FULL_BYTES;
  else
    return 0u;
  return (FLASH_LIBRARY_BANK_SIZE - FLASH_LIBRARY_RECORDS_OFFSET -
          FLASH_LIBRARY_TRAILER_BYTES) / record_size;
}

int flash_library_writer_begin(flash_library_t *library, flash_library_writer_t *writer,
                               const flash_library_object_info_t *info)
{
  uint32_t record_size;
  uint32_t records_bytes;
  uint32_t sectors;
  uint32_t address;
  int index;
  if (library == NULL || writer == NULL || info == NULL)
    return FLASH_LIBRARY_PARAM;
  if (library->mounted == 0u) return FLASH_LIBRARY_INVALID_STATE;
  if (info->object_id == 0u || info->record_count == 0u ||
      info->frame_period_us == 0u) return FLASH_LIBRARY_PARAM;
  if (info->level_mode > FLASH_LIBRARY_LEVEL_UNIFORM) return FLASH_LIBRARY_PARAM;
  if (info->object_type != FLASH_LIBRARY_OBJECT_SEQUENCE)
    return FLASH_LIBRARY_PARAM;
  record_size = info->level_mode == FLASH_LIBRARY_LEVEL_UNIFORM ?
                FLASH_LIBRARY_RECORD_PHASE_BYTES : FLASH_LIBRARY_RECORD_FULL_BYTES;
  if (info->record_count >
      (FLASH_LIBRARY_BANK_SIZE - FLASH_LIBRARY_RECORDS_OFFSET -
       FLASH_LIBRARY_TRAILER_BYTES) / record_size) return FLASH_LIBRARY_FULL;
  records_bytes = info->record_count * record_size;
  sectors = flash_library_sector_count(FLASH_LIBRARY_RECORDS_OFFSET +
                                       records_bytes + FLASH_LIBRARY_TRAILER_BYTES);
  index = flash_library_entry_index(library, info->object_id);
  if (index < 0 && library->entry_count >= FLASH_LIBRARY_MAX_OBJECTS)
    return FLASH_LIBRARY_FULL;
  if (flash_library_ensure_space(library, sectors) != FLASH_LIBRARY_OK)
    return FLASH_LIBRARY_FULL;
  address = library->next_address;
  memset(writer, 0, sizeof(*writer));
  writer->library = library;
  writer->object_address = address;
  writer->records_address = address + FLASH_LIBRARY_RECORDS_OFFSET;
  writer->records_bytes = records_bytes;
  writer->page_address = writer->records_address;
  writer->crc_state = 0xFFFFFFFFu;
  memset(&writer->header, 0, sizeof(writer->header));
  writer->header.magic = FLASH_LIBRARY_MAGIC;
  writer->header.version = FLASH_LIBRARY_FORMAT_VERSION;
  writer->header.header_bytes = FLASH_LIBRARY_HEADER_BYTES;
  writer->header.generation = library->next_generation;
  writer->header.object_id = info->object_id;
  writer->header.object_type = info->object_type;
  writer->header.level_mode = info->level_mode;
  writer->header.channel_count = FLASH_LIBRARY_CHANNEL_COUNT;
  writer->header.record_size = record_size;
  writer->header.record_count = info->record_count;
  writer->header.frame_period_us = info->frame_period_us;
  writer->header.level_offset = info->level_mode == FLASH_LIBRARY_LEVEL_UNIFORM ?
                                FLASH_LIBRARY_LEVEL_OFFSET : 0u;
  writer->header.records_offset = FLASH_LIBRARY_RECORDS_OFFSET;
  writer->header.records_bytes = records_bytes;
  writer->header.sector_count = sectors;
  writer->header.flags = info->repeat == FLASH_LIBRARY_REPEAT_LOOP ?
                         FLASH_LIBRARY_FLAG_LOOP : 0u;
  writer->header.commit = FLASH_LIBRARY_PROVISIONAL;
  writer->header.header_crc32 = flash_library_header_crc(&writer->header);
  library->next_address = address + sectors * FLASH_NOR_SECTOR_SIZE;
  if (flash_nor_page_program(address, &writer->header,
                             FLASH_LIBRARY_HEADER_BYTES) != FLASH_NOR_OK)
    return FLASH_LIBRARY_IO;
  library->stats.page_programs++;
  library->stats.bytes_programmed += FLASH_LIBRARY_HEADER_BYTES;
  if (info->level_mode == FLASH_LIBRARY_LEVEL_UNIFORM) {
    if (flash_nor_page_program(address + FLASH_LIBRARY_LEVEL_OFFSET,
                               info->level_table,
                               FLASH_LIBRARY_RECORD_PHASE_BYTES) != FLASH_NOR_OK)
      return FLASH_LIBRARY_IO;
    library->stats.page_programs++;
    library->stats.bytes_programmed += FLASH_LIBRARY_RECORD_PHASE_BYTES;
    writer->crc_state = umh_crc32_update(writer->crc_state, info->level_table,
                                         FLASH_LIBRARY_RECORD_PHASE_BYTES);
  }
  writer->active = 1u;
  return FLASH_LIBRARY_OK;
}

int flash_library_writer_write(flash_library_writer_t *writer, const void *data,
                               uint32_t length)
{
  const uint8_t *source = (const uint8_t *)data;
  if (writer == NULL || data == NULL || writer->active == 0u)
    return FLASH_LIBRARY_INVALID_STATE;
  if (length == 0u || length > writer->records_bytes - writer->received_bytes)
    return FLASH_LIBRARY_PARAM;
  writer->crc_state = umh_crc32_update(writer->crc_state, data, length);
  /* The record stream is page-aligned from records_address on, so full pages
   * can be programmed straight out of the caller's buffer.  The staging page
   * is then only needed for whole chunks that straddle a page boundary. */
  while (writer->page_used == 0u && length >= FLASH_NOR_PAGE_SIZE) {
    if (flash_nor_page_program(writer->page_address, source,
                               FLASH_NOR_PAGE_SIZE) != FLASH_NOR_OK) {
      writer->active = 0u;
      return FLASH_LIBRARY_IO;
    }
    writer->library->stats.page_programs++;
    writer->library->stats.bytes_programmed += FLASH_NOR_PAGE_SIZE;
    writer->page_address += FLASH_NOR_PAGE_SIZE;
    writer->received_bytes += FLASH_NOR_PAGE_SIZE;
    source += FLASH_NOR_PAGE_SIZE;
    length -= FLASH_NOR_PAGE_SIZE;
  }
  while (length != 0u) {
    uint32_t room = FLASH_NOR_PAGE_SIZE - writer->page_used;
    uint32_t take = length < room ? length : room;
    memcpy(&writer->page[writer->page_used], source, take);
    writer->page_used = (uint16_t)(writer->page_used + take);
    writer->received_bytes += take;
    source += take;
    length -= take;
    if (writer->page_used == FLASH_NOR_PAGE_SIZE) {
      if (flash_nor_page_program(writer->page_address, writer->page,
                                 FLASH_NOR_PAGE_SIZE) != FLASH_NOR_OK) {
        writer->active = 0u;
        return FLASH_LIBRARY_IO;
      }
      writer->library->stats.page_programs++;
      writer->library->stats.bytes_programmed += FLASH_NOR_PAGE_SIZE;
      writer->page_address += FLASH_NOR_PAGE_SIZE;
      writer->page_used = 0u;
    }
  }
  return FLASH_LIBRARY_OK;
}

int flash_library_writer_finish(flash_library_writer_t *writer)
{
  flash_library_t *library;
  uint32_t crc;
  uint32_t check;
  uint32_t commit = FLASH_LIBRARY_COMMIT_VALUE;
  int index;
  if (writer == NULL || writer->active == 0u)
    return FLASH_LIBRARY_INVALID_STATE;
  library = writer->library;
  if (writer->received_bytes != writer->records_bytes) return FLASH_LIBRARY_PARAM;
  crc = umh_crc32_finish(writer->crc_state);
  {
    uint32_t trailer_address = writer->records_address + writer->records_bytes;
    if (writer->page_used != 0u &&
        writer->page_used + FLASH_LIBRARY_TRAILER_BYTES <= FLASH_NOR_PAGE_SIZE) {
      memcpy(&writer->page[writer->page_used], &crc, FLASH_LIBRARY_TRAILER_BYTES);
      if (flash_nor_page_program(writer->page_address, writer->page,
                                 writer->page_used + FLASH_LIBRARY_TRAILER_BYTES) !=
          FLASH_NOR_OK) {
        writer->active = 0u;
        return FLASH_LIBRARY_IO;
      }
      library->stats.page_programs++;
      library->stats.bytes_programmed +=
          writer->page_used + FLASH_LIBRARY_TRAILER_BYTES;
      writer->page_used = 0u;
    } else {
      if (writer->page_used != 0u) {
        if (flash_nor_page_program(writer->page_address, writer->page,
                                   writer->page_used) != FLASH_NOR_OK) {
          writer->active = 0u;
          return FLASH_LIBRARY_IO;
        }
        library->stats.page_programs++;
        library->stats.bytes_programmed += writer->page_used;
        writer->page_used = 0u;
      }
      if (flash_nor_page_program(trailer_address, &crc,
                                 FLASH_LIBRARY_TRAILER_BYTES) != FLASH_NOR_OK) {
        writer->active = 0u;
        return FLASH_LIBRARY_IO;
      }
      library->stats.page_programs++;
      library->stats.bytes_programmed += FLASH_LIBRARY_TRAILER_BYTES;
    }
    if (flash_nor_read_dma(trailer_address, &check,
                           FLASH_LIBRARY_TRAILER_BYTES) != FLASH_NOR_OK ||
        check != crc) {
      writer->active = 0u;
      return FLASH_LIBRARY_CORRUPT;
    }
  }
  if (flash_library_verify_payload(library, writer->object_address,
                                   writer->header.level_mode,
                                   writer->records_bytes, crc) != FLASH_LIBRARY_OK) {
    writer->active = 0u;
    return FLASH_LIBRARY_CORRUPT;
  }
  if (flash_nor_page_program(writer->object_address +
                             offsetof(flash_library_header_t, commit),
                             &commit, sizeof(commit)) != FLASH_NOR_OK) {
    writer->active = 0u;
    return FLASH_LIBRARY_IO;
  }
  library->stats.page_programs++;
  library->stats.bytes_programmed += sizeof(commit);
  if (flash_nor_read_dma(writer->object_address +
                         offsetof(flash_library_header_t, commit),
                         &check, sizeof(check)) != FLASH_NOR_OK ||
      check != FLASH_LIBRARY_COMMIT_VALUE) {
    writer->active = 0u;
    return FLASH_LIBRARY_CORRUPT;
  }
  index = flash_library_entry_index(library, writer->header.object_id);
  if (index >= 0) {
    flash_library_entry_fill(&library->entries[index], &writer->header,
                             writer->object_address, library->active_bank, crc);
  } else if (library->entry_count < FLASH_LIBRARY_MAX_OBJECTS) {
    flash_library_entry_fill(&library->entries[library->entry_count++],
                             &writer->header, writer->object_address,
                             library->active_bank, crc);
  } else {
    writer->active = 0u;
    return FLASH_LIBRARY_FULL;
  }
  library->next_generation++;
  library->stats.uploads++;
  writer->active = 0u;
  return FLASH_LIBRARY_OK;
}

void flash_library_writer_abort(flash_library_writer_t *writer)
{
  if (writer != NULL) writer->active = 0u;
}

int flash_library_delete(flash_library_t *library, uint32_t object_id)
{
  flash_library_header_t header;
  uint32_t address;
  uint32_t check;
  uint32_t commit = FLASH_LIBRARY_COMMIT_VALUE;
  int index;
  if (library == NULL || library->mounted == 0u) return FLASH_LIBRARY_INVALID_STATE;
  index = flash_library_entry_index(library, object_id);
  if (index < 0) return FLASH_LIBRARY_NOT_FOUND;
  if (flash_library_ensure_space(library, 1u) != FLASH_LIBRARY_OK)
    return FLASH_LIBRARY_FULL;
  address = library->next_address;
  memset(&header, 0, sizeof(header));
  header.magic = FLASH_LIBRARY_MAGIC;
  header.version = FLASH_LIBRARY_FORMAT_VERSION;
  header.header_bytes = FLASH_LIBRARY_HEADER_BYTES;
  header.generation = library->next_generation;
  header.object_id = object_id;
  header.object_type = FLASH_LIBRARY_OBJECT_TOMBSTONE;
  header.sector_count = 1u;
  header.commit = FLASH_LIBRARY_PROVISIONAL;
  header.header_crc32 = flash_library_header_crc(&header);
  library->next_address = address + FLASH_NOR_SECTOR_SIZE;
  if (flash_nor_page_program(address, &header, FLASH_LIBRARY_HEADER_BYTES) !=
      FLASH_NOR_OK) return FLASH_LIBRARY_IO;
  library->stats.page_programs++;
  library->stats.bytes_programmed += FLASH_LIBRARY_HEADER_BYTES;
  if (flash_nor_read_dma(address, &library->scratch_header,
                         FLASH_LIBRARY_HEADER_BYTES) != FLASH_NOR_OK)
    return FLASH_LIBRARY_IO;
  if (flash_library_header_valid(&library->scratch_header, address,
                                 library->active_bank) == 0 ||
      library->scratch_header.commit != FLASH_LIBRARY_PROVISIONAL ||
      library->scratch_header.object_id != object_id)
    return FLASH_LIBRARY_CORRUPT;
  if (flash_nor_page_program(address + offsetof(flash_library_header_t, commit),
                             &commit, sizeof(commit)) != FLASH_NOR_OK)
    return FLASH_LIBRARY_IO;
  library->stats.page_programs++;
  library->stats.bytes_programmed += sizeof(commit);
  if (flash_nor_read_dma(address + offsetof(flash_library_header_t, commit),
                         &check, sizeof(check)) != FLASH_NOR_OK ||
      check != FLASH_LIBRARY_COMMIT_VALUE) return FLASH_LIBRARY_CORRUPT;
  flash_library_entry_remove(library, (uint16_t)index);
  library->next_generation++;
  library->stats.deletes++;
  return FLASH_LIBRARY_OK;
}

int flash_library_verify_object(flash_library_t *library, uint32_t object_id)
{
  const flash_library_entry_t *entry;
  uint32_t trailer;
  if (library == NULL || library->mounted == 0u)
    return FLASH_LIBRARY_INVALID_STATE;
  entry = flash_library_object_find(library, object_id);
  if (entry == NULL) return FLASH_LIBRARY_NOT_FOUND;
  if (flash_nor_read_dma(entry->address + FLASH_LIBRARY_RECORDS_OFFSET +
                         entry->records_bytes, &trailer,
                         FLASH_LIBRARY_TRAILER_BYTES) != FLASH_NOR_OK)
    return FLASH_LIBRARY_IO;
  return flash_library_verify_payload(library, entry->address,
                                      (uint8_t)entry->level_mode,
                                      entry->records_bytes, trailer);
}

int flash_library_reader_open(flash_library_reader_t *reader,
                              flash_library_t *library, uint32_t object_id,
                              uint8_t repeat, uint64_t origin_us)
{
  const flash_library_entry_t *entry;
  if (reader == NULL || library == NULL || library->mounted == 0u)
    return FLASH_LIBRARY_PARAM;
  entry = flash_library_object_find(library, object_id);
  if (entry == NULL) return FLASH_LIBRARY_NOT_FOUND;
  memset(reader, 0, sizeof(*reader));
  reader->library = library;
  reader->object_id = object_id;
  reader->object_address = entry->address;
  reader->record_count = entry->record_count;
  reader->record_size = entry->record_size;
  reader->records_bytes = entry->records_bytes;
  reader->payload_crc32 = entry->payload_crc32;
  reader->frame_period_us = entry->frame_period_us;
  reader->level_mode = (uint8_t)entry->level_mode;
  reader->flags = (uint16_t)entry->flags;
  reader->repeat = repeat != 0u ? 1u : 0u;
  reader->origin_us = origin_us;
  if (flash_nor_read_dma(entry->address, &library->scratch_header,
                         FLASH_LIBRARY_HEADER_BYTES) != FLASH_NOR_OK)
    return FLASH_LIBRARY_IO;
  if (flash_library_header_valid(&library->scratch_header, entry->address,
                                 (uint8_t)entry->bank) == 0 ||
      library->scratch_header.commit != FLASH_LIBRARY_COMMIT_VALUE ||
      library->scratch_header.object_type != FLASH_LIBRARY_OBJECT_SEQUENCE ||
      library->scratch_header.object_id != object_id ||
      library->scratch_header.generation != entry->generation)
    return FLASH_LIBRARY_CORRUPT;
  if (flash_nor_read_dma(entry->address + FLASH_LIBRARY_RECORDS_OFFSET +
                         entry->records_bytes, &reader->payload_crc32,
                         FLASH_LIBRARY_TRAILER_BYTES) != FLASH_NOR_OK)
    return FLASH_LIBRARY_IO;
  if (reader->level_mode == FLASH_LIBRARY_LEVEL_UNIFORM) {
    if (flash_nor_read_dma(entry->address + FLASH_LIBRARY_LEVEL_OFFSET,
                           reader->level_table,
                           FLASH_LIBRARY_RECORD_PHASE_BYTES) != FLASH_NOR_OK)
      return FLASH_LIBRARY_IO;
  }
  reader->opened = 1u;
  return FLASH_LIBRARY_OK;
}

int flash_library_reader_pump(flash_library_reader_t *reader,
                              flash_library_frame_sink_t sink, void *context,
                              uint32_t budget)
{
  uint32_t emitted = 0u;
  if (reader == NULL || sink == NULL || reader->opened == 0u)
    return FLASH_LIBRARY_PARAM;
  if (reader->library == NULL || reader->library->mounted == 0u)
    return FLASH_LIBRARY_INVALID_STATE;
  if (budget == 0u) budget = FLASH_LIBRARY_CHUNK_FRAMES;
  for (;;) {
    if (reader->prefetch_in_flight != 0u) {
      int poll = flash_library_reader_poll(reader);
      if (poll < 0) return poll;
      if (poll == 0) return (int)emitted;
      reader->buffer_frames[reader->prefetch_buffer] =
          (uint8_t)reader->prefetch_frames;
      reader->buffer_pos[reader->prefetch_buffer] = 0u;
      reader->prefetch_in_flight = 0u;
    }
    if (reader->active_valid == 0u) {
      uint8_t ready = 0xFFu;
      if (reader->buffer_frames[0] != 0u) ready = 0u;
      else if (reader->buffer_frames[1] != 0u) ready = 1u;
      if (ready == 0xFFu) {
        if (reader->finished != 0u) return (int)emitted;
        if (flash_library_reader_prefetch(reader, reader->cursor, 0u) !=
            FLASH_LIBRARY_OK) return FLASH_LIBRARY_IO;
        return (int)emitted;
      }
      reader->active_buffer = ready;
      reader->active_valid = 1u;
    }
    if (reader->prefetch_in_flight == 0u) {
      uint32_t after = reader->cursor +
          (uint32_t)(reader->buffer_frames[reader->active_buffer] -
                     reader->buffer_pos[reader->active_buffer]);
      uint8_t target = (uint8_t)(reader->active_buffer ^ 1u);
      if (after >= reader->record_count)
        after = reader->repeat != 0u ? 0u : reader->record_count;
      if (after < reader->record_count &&
          reader->buffer_frames[target] == 0u) {
        if (flash_library_reader_prefetch(reader, after, target) !=
            FLASH_LIBRARY_OK) return FLASH_LIBRARY_IO;
      }
    }
    while (reader->buffer_pos[reader->active_buffer] <
           reader->buffer_frames[reader->active_buffer] &&
           emitted < budget) {
      const uint8_t *record = &reader->buffers[reader->active_buffer]
          [(uint32_t)reader->buffer_pos[reader->active_buffer] *
           reader->record_size];
      umh_output_frame_t frame;
      flash_library_build_frame(reader, record, reader->cursor,
                                reader->sequence, &frame);
      if (sink(context, &frame) != 0) return (int)emitted;
      reader->sequence++;
      reader->cursor++;
      reader->buffer_pos[reader->active_buffer]++;
      emitted++;
      reader->library->stats.replay_frames++;
      if (reader->cursor >= reader->record_count) {
        if (reader->repeat != 0u) {
          reader->origin_us += (uint64_t)reader->record_count *
                               (uint64_t)reader->frame_period_us;
          reader->cursor = 0u;
        } else {
          reader->finished = 1u;
        }
      }
    }
    if (reader->buffer_pos[reader->active_buffer] >=
        reader->buffer_frames[reader->active_buffer]) {
      reader->buffer_frames[reader->active_buffer] = 0u;
      reader->buffer_pos[reader->active_buffer] = 0u;
      reader->active_valid = 0u;
    }
    if (emitted >= budget) return (int)emitted;
    if (reader->finished != 0u && reader->active_valid == 0u &&
        reader->prefetch_in_flight == 0u &&
        reader->buffer_frames[0] == 0u && reader->buffer_frames[1] == 0u)
      return (int)emitted;
  }
}

int flash_library_reader_read_frame(flash_library_reader_t *reader,
                                    uint32_t frame_index, umh_output_frame_t *frame)
{
  uint8_t record[FLASH_LIBRARY_RECORD_FULL_BYTES];
  if (reader == NULL || frame == NULL || reader->opened == 0u)
    return FLASH_LIBRARY_PARAM;
  if (frame_index >= reader->record_count) return FLASH_LIBRARY_PARAM;
  if (reader->prefetch_in_flight != 0u) return FLASH_LIBRARY_BUSY;
  if (flash_nor_read_dma(reader->object_address + FLASH_LIBRARY_RECORDS_OFFSET +
                         frame_index * reader->record_size, record,
                         reader->record_size) != FLASH_NOR_OK)
    return FLASH_LIBRARY_IO;
  flash_library_build_frame(reader, record, frame_index, frame_index, frame);
  return FLASH_LIBRARY_OK;
}

int flash_library_reader_seek(flash_library_reader_t *reader, uint32_t frame_index)
{
  if (reader == NULL || reader->opened == 0u) return FLASH_LIBRARY_PARAM;
  if (reader->repeat == 0u && frame_index > reader->record_count)
    return FLASH_LIBRARY_PARAM;
  if (reader->prefetch_in_flight != 0u) {
    flash_library_read_abort(reader->library);
    reader->prefetch_in_flight = 0u;
  }
  reader->active_valid = 0u;
  reader->buffer_frames[0] = 0u;
  reader->buffer_frames[1] = 0u;
  reader->buffer_pos[0] = 0u;
  reader->buffer_pos[1] = 0u;
  reader->cursor = frame_index;
  reader->finished = (reader->repeat == 0u &&
                      frame_index >= reader->record_count) ? 1u : 0u;
  return FLASH_LIBRARY_OK;
}

void flash_library_reader_close(flash_library_reader_t *reader)
{
  if (reader == NULL) return;
  if (reader->prefetch_in_flight != 0u) {
    flash_library_read_abort(reader->library);
    reader->prefetch_in_flight = 0u;
  }
  reader->opened = 0u;
  reader->active_valid = 0u;
  reader->buffer_frames[0] = 0u;
  reader->buffer_frames[1] = 0u;
}

uint8_t flash_library_reader_done(const flash_library_reader_t *reader)
{
  if (reader == NULL || reader->opened == 0u) return 0u;
  return reader->finished != 0u && reader->active_valid == 0u &&
         reader->prefetch_in_flight == 0u &&
         reader->buffer_frames[0] == 0u && reader->buffer_frames[1] == 0u;
}

uint64_t flash_library_reader_origin(const flash_library_reader_t *reader)
{
  return reader != NULL ? reader->origin_us : 0u;
}

uint32_t flash_library_reader_cursor(const flash_library_reader_t *reader)
{
  return reader != NULL ? reader->cursor : 0u;
}

int flash_library_ring_sink(void *context, const umh_output_frame_t *frame)
{
  flash_library_ring_sink_t *sink = (flash_library_ring_sink_t *)context;
  umh_output_frame_t *slot;
  if (sink == NULL || sink->ring == NULL || frame == NULL) return -1;
  slot = frame_ring_acquire_write(sink->ring);
  if (slot == NULL) return -1;
  memcpy(slot, frame, sizeof(*slot));
  return frame_ring_commit_write(sink->ring) == 0 ? 0 : -1;
}
