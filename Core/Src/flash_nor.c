#include "flash_nor.h"
#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <string.h>

#define CMD_READ_ID       0x9Fu
#define CMD_READ          0x03u
#define CMD_FAST_READ     0x0Bu
#define CMD_WRITE_ENABLE  0x06u
#define CMD_READ_STATUS   0x05u
#define CMD_WRITE_STATUS  0x01u
#define CMD_PAGE_PROGRAM  0x02u
#define CMD_SECTOR_ERASE  0x20u
#define CMD_BLOCK_ERASE   0xD8u
#define CMD_RESET_ENABLE  0x66u
#define CMD_RESET         0x99u

static SPI_HandleTypeDef *flash_spi;
static uint8_t flash_present;
static const uint8_t flash_dummy_byte = 0xFFu;
static SemaphoreHandle_t flash_dma_sem;
static StaticSemaphore_t flash_dma_sem_mem;

static void flash_dma_sem_init(void)
{
  if (flash_dma_sem != NULL) return;
  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) return;
  flash_dma_sem = xSemaphoreCreateBinaryStatic(&flash_dma_sem_mem);
}

static void flash_dma_sem_drain(void)
{
  if (flash_dma_sem == NULL) return;
  while (xSemaphoreTake(flash_dma_sem, 0u) == pdTRUE) {
  }
}

static void flash_dma_sem_give(void)
{
  BaseType_t woken = pdFALSE;
  if (flash_dma_sem == NULL) return;
  if (__get_IPSR() != 0u) {
    (void)xSemaphoreGiveFromISR(flash_dma_sem, &woken);
    portYIELD_FROM_ISR(woken);
  } else {
    (void)xSemaphoreGive(flash_dma_sem);
  }
}

void flash_nor_dma_complete(SPI_HandleTypeDef *spi)
{
  if (spi != NULL && spi == flash_spi) flash_dma_sem_give();
}

void flash_nor_dma_error(SPI_HandleTypeDef *spi)
{
  if (spi != NULL && spi == flash_spi) flash_dma_sem_give();
}

void flash_nor_dma_drain(void)
{
  flash_dma_sem_drain();
}

static void cs_low(void) { HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET); }
static void cs_high(void) { HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET); }

static void address_bytes(uint8_t *buffer, uint32_t address)
{
  buffer[0] = (uint8_t)(address >> 16);
  buffer[1] = (uint8_t)(address >> 8);
  buffer[2] = (uint8_t)address;
}

static flash_nor_status_t wait_ready(uint32_t timeout)
{
  uint32_t start = HAL_GetTick();
  uint8_t exchange[2];
  uint8_t status;
  if (flash_spi == NULL) return FLASH_NOR_IO;
  do {
    /* One full-duplex transaction per poll instead of a separate transmit and
     * receive: a 4 MiB upload issues one poll every few microseconds of SPI
     * time, so the second HAL call is a measurable slice of the write path. */
    exchange[0] = CMD_READ_STATUS;
    exchange[1] = 0u;
    cs_low();
    if (HAL_SPI_TransmitReceive(flash_spi, exchange, exchange, 2u, 10u) != HAL_OK) {
      cs_high();
      return FLASH_NOR_IO;
    }
    cs_high();
    status = exchange[1];
    if ((status & 0x01u) == 0u) return FLASH_NOR_OK;
  } while ((HAL_GetTick() - start) < timeout);
  return FLASH_NOR_TIMEOUT;
}

static flash_nor_status_t write_enable(void)
{
  uint8_t command = CMD_WRITE_ENABLE;
  cs_low();
  if (HAL_SPI_Transmit(flash_spi, &command, 1u, 10u) != HAL_OK) {
    cs_high();
    return FLASH_NOR_IO;
  }
  cs_high();
  return FLASH_NOR_OK;
}

void flash_nor_init(SPI_HandleTypeDef *spi)
{
  flash_spi = spi;
  flash_present = (spi != NULL) ? 1u : 0u;
  flash_dma_sem_init();
  cs_high();
}

flash_nor_status_t flash_nor_read_jedec(uint8_t id[3])
{
  uint8_t command = CMD_READ_ID;
  if (flash_spi == NULL || id == NULL) return FLASH_NOR_PARAM;
  cs_low();
  if (HAL_SPI_Transmit(flash_spi, &command, 1u, 10u) != HAL_OK ||
      HAL_SPI_Receive(flash_spi, id, 3u, 10u) != HAL_OK) {
    cs_high();
    flash_present = 0u;
    return FLASH_NOR_IO;
  }
  cs_high();
  flash_present = (id[0] != 0u && id[0] != 0xFFu) ? 1u : 0u;
  return FLASH_NOR_OK;
}

flash_nor_status_t flash_nor_read(uint32_t address, void *data, uint32_t length)
{
  uint8_t command[4];
  uint8_t *dst = (uint8_t *)data;
  uint32_t chunk;
  if (flash_spi == NULL || data == NULL || length == 0u || address >= FLASH_NOR_TOTAL_SIZE ||
      length > FLASH_NOR_TOTAL_SIZE - address) return FLASH_NOR_PARAM;
  while (length != 0u) {
    chunk = length > 1024u ? 1024u : length;
    command[0] = CMD_READ;
    address_bytes(&command[1], address);
    cs_low();
    if (HAL_SPI_Transmit(flash_spi, command, sizeof(command), 20u) != HAL_OK ||
        HAL_SPI_Receive(flash_spi, dst, chunk, 100u) != HAL_OK) {
      cs_high();
      return FLASH_NOR_IO;
    }
    cs_high();
    address += chunk;
    dst += chunk;
    length -= chunk;
  }
  return FLASH_NOR_OK;
}

flash_nor_status_t flash_nor_wait_dma(uint32_t timeout_ms)
{
  uint32_t start;
  if (flash_spi == NULL) return FLASH_NOR_IO;
  flash_dma_sem_init();
  if (flash_dma_sem == NULL) {
    start = HAL_GetTick();
    while (HAL_SPI_GetState(flash_spi) != HAL_SPI_STATE_READY) {
      if ((HAL_GetTick() - start) > timeout_ms) {
        (void)HAL_SPI_Abort(flash_spi);
        return FLASH_NOR_TIMEOUT;
      }
    }
    return HAL_SPI_GetError(flash_spi) == HAL_SPI_ERROR_NONE ? FLASH_NOR_OK :
           FLASH_NOR_IO;
  }
  if (xSemaphoreTake(flash_dma_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
    (void)HAL_SPI_Abort(flash_spi);
    flash_dma_sem_drain();
    return FLASH_NOR_TIMEOUT;
  }
  if (HAL_SPI_GetState(flash_spi) != HAL_SPI_STATE_READY ||
      HAL_SPI_GetError(flash_spi) != HAL_SPI_ERROR_NONE) {
    (void)HAL_SPI_Abort(flash_spi);
    flash_dma_sem_drain();
    return FLASH_NOR_IO;
  }
  return FLASH_NOR_OK;
}

flash_nor_status_t flash_nor_read_dma(uint32_t address, void *data, uint32_t length)
{
  uint8_t command[4];
  uint8_t *dst = (uint8_t *)data;
  uint32_t chunk;
  flash_nor_status_t result;
  if (flash_spi == NULL || data == NULL || length == 0u || address >= FLASH_NOR_TOTAL_SIZE ||
      length > FLASH_NOR_TOTAL_SIZE - address) return FLASH_NOR_PARAM;
  while (length != 0u) {
    chunk = length > FLASH_NOR_DMA_CHUNK ? FLASH_NOR_DMA_CHUNK : length;
    command[0] = CMD_FAST_READ;
    address_bytes(&command[1], address);
    flash_dma_sem_drain();
    cs_low();
    if (HAL_SPI_Transmit(flash_spi, command, sizeof(command), 20u) != HAL_OK ||
        HAL_SPI_Transmit(flash_spi, &flash_dummy_byte, 1u, 10u) != HAL_OK ||
        HAL_SPI_Receive_DMA(flash_spi, dst, chunk) != HAL_OK) {
      cs_high();
      return FLASH_NOR_IO;
    }
    result = flash_nor_wait_dma(100u);
    cs_high();
    if (result != FLASH_NOR_OK) return result;
    address += chunk;
    dst += chunk;
    length -= chunk;
  }
  return FLASH_NOR_OK;
}

flash_nor_status_t flash_nor_page_program(uint32_t address, const void *data, uint32_t length)
{
  uint8_t command[4];
  uint32_t chunk;
  const uint8_t *src = (const uint8_t *)data;
  if (flash_spi == NULL || data == NULL || length == 0u || address >= FLASH_NOR_TOTAL_SIZE ||
      length > FLASH_NOR_TOTAL_SIZE - address) return FLASH_NOR_PARAM;
  while (length != 0u) {
    chunk = FLASH_NOR_PAGE_SIZE - (address % FLASH_NOR_PAGE_SIZE);
    if (chunk > length) chunk = length;
    if (write_enable() != FLASH_NOR_OK) return FLASH_NOR_IO;
    command[0] = CMD_PAGE_PROGRAM;
    address_bytes(&command[1], address);
    cs_low();
    if (HAL_SPI_Transmit(flash_spi, command, sizeof(command), 20u) != HAL_OK ||
        HAL_SPI_Transmit(flash_spi, (uint8_t *)src, chunk, 100u) != HAL_OK) {
      cs_high();
      return FLASH_NOR_IO;
    }
    cs_high();
    if (wait_ready(500u) != FLASH_NOR_OK) return FLASH_NOR_TIMEOUT;
    address += chunk;
    src += chunk;
    length -= chunk;
  }
  return FLASH_NOR_OK;
}

static flash_nor_status_t erase(uint8_t opcode, uint32_t address, uint32_t timeout)
{
  uint8_t command[4];
  if (flash_spi == NULL || address >= FLASH_NOR_TOTAL_SIZE) return FLASH_NOR_PARAM;
  if (write_enable() != FLASH_NOR_OK) return FLASH_NOR_IO;
  command[0] = opcode;
  address_bytes(&command[1], address);
  cs_low();
  if (HAL_SPI_Transmit(flash_spi, command, sizeof(command), 20u) != HAL_OK) {
    cs_high();
    return FLASH_NOR_IO;
  }
  cs_high();
  return wait_ready(timeout);
}

flash_nor_status_t flash_nor_erase_sector(uint32_t address)
{
  return erase(CMD_SECTOR_ERASE, address & ~(FLASH_NOR_SECTOR_SIZE - 1u), 3000u);
}

flash_nor_status_t flash_nor_erase_block(uint32_t address)
{
  return erase(CMD_BLOCK_ERASE, address & ~(FLASH_NOR_BLOCK_SIZE - 1u), 10000u);
}

flash_nor_status_t flash_nor_reset(void)
{
  uint8_t command = CMD_RESET_ENABLE;
  if (flash_spi == NULL) return FLASH_NOR_IO;
  cs_low();
  if (HAL_SPI_Transmit(flash_spi, &command, 1u, 10u) != HAL_OK) { cs_high(); return FLASH_NOR_IO; }
  cs_high();
  command = CMD_RESET;
  cs_low();
  if (HAL_SPI_Transmit(flash_spi, &command, 1u, 10u) != HAL_OK) { cs_high(); return FLASH_NOR_IO; }
  cs_high();
  HAL_Delay(1u);
  return wait_ready(100u);
}

uint8_t flash_nor_is_present(void) { return flash_present; }
