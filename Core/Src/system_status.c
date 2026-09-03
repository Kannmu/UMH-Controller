#include "system_status.h"
#include "main.h"
#include <string.h>

static umh_system_status_t status;
static uint32_t time_last_cycles;
static uint64_t time_cycles;
static uint8_t time_ready;

void system_time_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  time_last_cycles = DWT->CYCCNT;
  time_cycles = 0u;
  time_ready = 1u;
}

uint64_t system_time_us(void)
{
  uint32_t cycles;
  uint32_t delta;
  if (time_ready == 0u || SystemCoreClock == 0u) return (uint64_t)HAL_GetTick() * 1000u;
  cycles = DWT->CYCCNT;
  delta = cycles - time_last_cycles;
  time_last_cycles = cycles;
  time_cycles += delta;
  return (time_cycles * 1000000ull) / SystemCoreClock;
}

void system_status_init(void) { memset(&status, 0, sizeof(status)); system_time_init(); }
umh_system_status_t *system_status_get(void) { return &status; }
void system_status_set(uint32_t flags) { status.flags |= flags; }
void system_status_clear(uint32_t flags) { status.flags &= ~flags; }
void system_status_error(uint32_t count) { status.protocol_errors += count; status.flags |= UMH_SYSTEM_ERROR; }

void configureTimerForRunTimeStats(void) { }

unsigned long getRunTimeCounterValue(void)
{
  return (unsigned long)HAL_GetTick();
}
