#ifndef FLASH_NOR_H
#define FLASH_NOR_H

#include <stdint.h>
#include "spi.h"

#define FLASH_NOR_PAGE_SIZE       256u
#define FLASH_NOR_SECTOR_SIZE     4096u
#define FLASH_NOR_BLOCK_SIZE      65536u
#define FLASH_NOR_TOTAL_SIZE      (16u * 1024u * 1024u)

typedef enum {
  FLASH_NOR_OK = 0,
  FLASH_NOR_TIMEOUT = -1,
  FLASH_NOR_IO = -2,
  FLASH_NOR_PARAM = -3,
  FLASH_NOR_BUSY = -4
} flash_nor_status_t;

void flash_nor_init(SPI_HandleTypeDef *spi);
flash_nor_status_t flash_nor_read_jedec(uint8_t id[3]);
flash_nor_status_t flash_nor_read(uint32_t address, void *data, uint32_t length);
flash_nor_status_t flash_nor_read_dma(uint32_t address, void *data, uint32_t length);
flash_nor_status_t flash_nor_page_program(uint32_t address, const void *data, uint32_t length);
flash_nor_status_t flash_nor_erase_sector(uint32_t address);
flash_nor_status_t flash_nor_erase_block(uint32_t address);
flash_nor_status_t flash_nor_reset(void);
uint8_t flash_nor_is_present(void);

#endif
