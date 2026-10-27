#ifndef FLASH_LIBRARY_H
#define FLASH_LIBRARY_H

#include <stddef.h>
#include <stdint.h>
#include "device_profile.h"
#include "flash_nor.h"
#include "frame_ring.h"
#include "spi.h"

#define FLASH_LIBRARY_MAGIC              0x554D4838u
#define FLASH_LIBRARY_FORMAT_VERSION     1u
#define FLASH_LIBRARY_OBJECT_SEQUENCE    1u
#define FLASH_LIBRARY_OBJECT_TOMBSTONE   0xFFFFu
#define FLASH_LIBRARY_COMMIT_VALUE       0x00000000u
#define FLASH_LIBRARY_PROVISIONAL        0xFFFFFFFFu

#ifndef FLASH_LIBRARY_REGION_BASE
#define FLASH_LIBRARY_REGION_BASE        (8u * 1024u * 1024u)
#endif
#ifndef FLASH_LIBRARY_REGION_SIZE
#define FLASH_LIBRARY_REGION_SIZE        (8u * 1024u * 1024u)
#endif

#define FLASH_LIBRARY_BANK_COUNT         2u
#define FLASH_LIBRARY_BANK_SIZE          (FLASH_LIBRARY_REGION_SIZE / FLASH_LIBRARY_BANK_COUNT)
#define FLASH_LIBRARY_BANK_BASE(bank)    (FLASH_LIBRARY_REGION_BASE + ((uint32_t)(bank) * FLASH_LIBRARY_BANK_SIZE))
#define FLASH_LIBRARY_BANK_SECTORS       (FLASH_LIBRARY_BANK_SIZE / FLASH_NOR_SECTOR_SIZE)

#define FLASH_LIBRARY_MAX_OBJECTS        8u
#define FLASH_LIBRARY_HEADER_BYTES       256u
#define FLASH_LIBRARY_LEVEL_OFFSET       256u
#define FLASH_LIBRARY_RECORDS_OFFSET     512u
#define FLASH_LIBRARY_TRAILER_BYTES      4u
#define FLASH_LIBRARY_CHANNEL_COUNT      UMH_DEVICE_CHANNEL_COUNT
#define FLASH_LIBRARY_RECORD_PHASE_BYTES UMH_DEVICE_CHANNEL_COUNT
#define FLASH_LIBRARY_RECORD_FULL_BYTES  (2u * UMH_DEVICE_CHANNEL_COUNT)

#define FLASH_LIBRARY_LEVEL_FULL         0u
#define FLASH_LIBRARY_LEVEL_UNIFORM      1u

#define FLASH_LIBRARY_REPEAT_ONCE        0u
#define FLASH_LIBRARY_REPEAT_LOOP        1u
#define FLASH_LIBRARY_FLAG_LOOP          (1u << 0)

#define FLASH_LIBRARY_CHUNK_FRAMES       4u
#define FLASH_LIBRARY_CHUNK_BYTES        (FLASH_LIBRARY_CHUNK_FRAMES * FLASH_LIBRARY_RECORD_FULL_BYTES)
#define FLASH_LIBRARY_READ_TIMEOUT_MS    100u

typedef enum {
  FLASH_LIBRARY_OK = 0,
  FLASH_LIBRARY_ERROR = -1,
  FLASH_LIBRARY_PARAM = -2,
  FLASH_LIBRARY_IO = -3,
  FLASH_LIBRARY_FULL = -4,
  FLASH_LIBRARY_NOT_FOUND = -5,
  FLASH_LIBRARY_BUSY = -6,
  FLASH_LIBRARY_CORRUPT = -7,
  FLASH_LIBRARY_INVALID_STATE = -8
} flash_library_status_t;

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint16_t version;
  uint16_t header_bytes;
  uint32_t generation;
  uint32_t object_id;
  uint16_t object_type;
  uint16_t level_mode;
  uint16_t channel_count;
  uint16_t reserved0;
  uint32_t record_size;
  uint32_t record_count;
  uint32_t frame_period_us;
  uint32_t level_offset;
  uint32_t records_offset;
  uint32_t records_bytes;
  uint32_t sector_count;
  uint32_t flags;
  uint32_t commit;
  uint32_t header_crc32;
  uint8_t reserved[FLASH_LIBRARY_HEADER_BYTES - 64u];
} flash_library_header_t;

_Static_assert(sizeof(flash_library_header_t) == FLASH_LIBRARY_HEADER_BYTES,
               "flash library header");
_Static_assert(offsetof(flash_library_header_t, commit) == 56u,
               "flash library commit offset");
_Static_assert(offsetof(flash_library_header_t, header_crc32) == 60u,
               "flash library header crc offset");
_Static_assert(sizeof(flash_library_header_t) == 256u, "flash library header bytes");
_Static_assert((FLASH_LIBRARY_REGION_BASE % FLASH_NOR_BLOCK_SIZE) == 0u,
               "flash library region must be block aligned");
_Static_assert((FLASH_LIBRARY_REGION_SIZE % (2u * FLASH_NOR_BLOCK_SIZE)) == 0u,
               "flash library banks must be block aligned");
_Static_assert(FLASH_LIBRARY_REGION_BASE + FLASH_LIBRARY_REGION_SIZE <= FLASH_NOR_TOTAL_SIZE,
               "flash library region must fit the NOR");
_Static_assert(FLASH_LIBRARY_BANK_SIZE >= (FLASH_NOR_BLOCK_SIZE * 2u),
               "flash library bank must hold erase blocks");

typedef struct {
  uint32_t object_id;
  uint32_t address;
  uint32_t generation;
  uint32_t record_count;
  uint32_t record_size;
  uint32_t frame_period_us;
  uint32_t records_bytes;
  uint32_t payload_crc32;
  uint32_t sector_count;
  uint16_t bank;
  uint16_t level_mode;
  uint32_t flags;
} flash_library_entry_t;

typedef struct {
  uint32_t page_programs;
  uint32_t bytes_programmed;
  uint32_t block_erases;
  uint32_t rotations;
  uint32_t uploads;
  uint32_t deletes;
  uint32_t mount_reads;
  uint32_t replay_frames;
  uint32_t read_errors;
} flash_library_stats_t;

typedef struct {
  flash_library_entry_t entries[FLASH_LIBRARY_MAX_OBJECTS];
  SPI_HandleTypeDef *spi;
  flash_library_stats_t stats;
  uint32_t next_generation;
  uint32_t next_address;
  uint16_t entry_count;
  uint8_t active_bank;
  uint8_t mounted;
  uint8_t truncated;
  uint8_t reserved;
  flash_library_header_t scratch_header;
  uint8_t scratch[FLASH_NOR_PAGE_SIZE];
} flash_library_t;

typedef struct {
  flash_library_t *library;
  flash_library_header_t header;
  uint32_t object_address;
  uint32_t records_address;
  uint32_t records_bytes;
  uint32_t received_bytes;
  uint32_t crc_state;
  uint32_t page_address;
  uint16_t page_used;
  uint8_t active;
  uint8_t page[FLASH_NOR_PAGE_SIZE];
} flash_library_writer_t;

typedef struct {
  flash_library_t *library;
  uint32_t object_address;
  uint32_t object_id;
  uint32_t record_count;
  uint32_t record_size;
  uint32_t records_bytes;
  uint32_t payload_crc32;
  uint32_t frame_period_us;
  uint32_t cursor;
  uint32_t sequence;
  uint32_t prefetch_address;
  uint32_t prefetch_frames;
  uint32_t prefetch_tick;
  uint64_t origin_us;
  uint16_t flags;
  uint8_t level_mode;
  uint8_t repeat;
  uint8_t opened;
  uint8_t finished;
  uint8_t prefetch_in_flight;
  uint8_t prefetch_buffer;
  uint8_t active_buffer;
  uint8_t active_valid;
  uint8_t buffer_pos[2];
  uint8_t buffer_frames[2];
  uint8_t level_table[FLASH_LIBRARY_CHANNEL_COUNT];
  uint8_t buffers[2][FLASH_LIBRARY_CHUNK_BYTES];
} flash_library_reader_t;

typedef struct {
  uint32_t object_id;
  uint32_t record_count;
  uint32_t frame_period_us;
  uint16_t object_type;
  uint8_t level_mode;
  uint8_t repeat;
  uint16_t flags;
  uint8_t level_table[FLASH_LIBRARY_CHANNEL_COUNT];
} flash_library_object_info_t;

typedef int (*flash_library_frame_sink_t)(void *context, const umh_output_frame_t *frame);

typedef struct {
  umh_frame_ring_t *ring;
} flash_library_ring_sink_t;

_Static_assert(FLASH_LIBRARY_CHUNK_BYTES <= 0xFFFFu, "flash library dma chunk");
_Static_assert(UINT16_MAX >= FLASH_LIBRARY_CHUNK_BYTES, "flash library chunk fits u16");
_Static_assert(FLASH_LIBRARY_CHANNEL_COUNT == 84u, "flash library channel contract");
_Static_assert(FLASH_LIBRARY_LEVEL_OFFSET + FLASH_LIBRARY_RECORD_PHASE_BYTES <=
               FLASH_LIBRARY_RECORDS_OFFSET, "flash library level table");
_Static_assert(FLASH_LIBRARY_CHUNK_FRAMES <= 0xFFu, "flash library chunk fits u8");

void flash_library_init(flash_library_t *library, SPI_HandleTypeDef *spi);
int flash_library_mount(flash_library_t *library);
int flash_library_delete(flash_library_t *library, uint32_t object_id);
int flash_library_verify_object(flash_library_t *library, uint32_t object_id);
uint32_t flash_library_free_bytes(const flash_library_t *library);
uint32_t flash_library_frame_capacity(uint8_t level_mode);
uint16_t flash_library_object_count(const flash_library_t *library);
const flash_library_entry_t *flash_library_object_at(const flash_library_t *library,
                                                     uint16_t index);
const flash_library_entry_t *flash_library_object_find(const flash_library_t *library,
                                                       uint32_t object_id);
const flash_library_stats_t *flash_library_stats(const flash_library_t *library);

int flash_library_writer_begin(flash_library_t *library, flash_library_writer_t *writer,
                               const flash_library_object_info_t *info);
int flash_library_writer_write(flash_library_writer_t *writer, const void *data,
                               uint32_t length);
int flash_library_writer_finish(flash_library_writer_t *writer);
void flash_library_writer_abort(flash_library_writer_t *writer);

int flash_library_reader_open(flash_library_reader_t *reader, flash_library_t *library,
                              uint32_t object_id, uint8_t repeat, uint64_t origin_us);
int flash_library_reader_pump(flash_library_reader_t *reader,
                              flash_library_frame_sink_t sink, void *context,
                              uint32_t budget);
int flash_library_reader_read_frame(flash_library_reader_t *reader, uint32_t frame_index,
                                    umh_output_frame_t *frame);
int flash_library_reader_seek(flash_library_reader_t *reader, uint32_t frame_index);
void flash_library_reader_close(flash_library_reader_t *reader);
uint8_t flash_library_reader_done(const flash_library_reader_t *reader);
uint64_t flash_library_reader_origin(const flash_library_reader_t *reader);
uint32_t flash_library_reader_cursor(const flash_library_reader_t *reader);
int flash_library_ring_sink(void *context, const umh_output_frame_t *frame);

#endif
