#include "system_status.h"
#include "main.h"
#include <string.h>

static umh_system_status_t status;
static uint32_t time_last_cycles;
static uint64_t time_cycles;
static uint8_t time_ready;
static uint8_t status_initialized;
static uint16_t boot_fault_code;
static uint32_t boot_fault_arg;
static uint8_t boot_fault_severity;
static uint16_t boot_critical_code;
static uint32_t boot_critical_arg;

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

void system_status_init(void)
{
  memset(&status, 0, sizeof(status));
  status_initialized = 1u;
  status.last_fault_code = boot_fault_code;
  status.last_fault_arg = boot_fault_arg;
  status.last_fault_severity = boot_fault_severity;
  status.fault_count = boot_fault_code == UMH_FAULT_NONE ? 0u : 1u;
  status.last_fault_time_ms = 0u;
  status.critical_fault_code = boot_critical_code;
  status.critical_fault_arg = boot_critical_arg;
  status.critical_fault_valid = boot_critical_code != UMH_FAULT_NONE ? 1u : 0u;
  if (status.critical_fault_valid != 0u) status.flags |= UMH_SYSTEM_ERROR;
  system_time_init();
}
umh_system_status_t *system_status_get(void) { return &status; }
void system_status_set(uint32_t flags) { status.flags |= flags; }
void system_status_clear(uint32_t flags) { status.flags &= ~flags; }
void system_status_error(uint32_t count)
{
  status.protocol_errors += count;
  system_status_fault(UMH_FAULT_PROTOCOL_NACK, count, UMH_FAULT_CRITICAL);
}

void system_status_fault(umh_fault_code_t code, uint32_t argument,
                         umh_fault_severity_t severity)
{
  if (code == UMH_FAULT_NONE || code >= UMH_FAULT_COUNT) return;
  if (status_initialized == 0u) {
    boot_fault_code = (uint16_t)code;
    boot_fault_arg = argument;
    boot_fault_severity = (uint8_t)severity;
    if (severity >= UMH_FAULT_CRITICAL) {
      boot_critical_code = (uint16_t)code;
      boot_critical_arg = argument;
    }
    return;
  }
  status.last_fault_code = (uint16_t)code;
  status.last_fault_arg = argument;
  status.last_fault_severity = (uint8_t)severity;
  status.last_fault_time_ms = HAL_GetTick();
  ++status.fault_count;
  if (severity >= UMH_FAULT_CRITICAL) {
    status.flags |= UMH_SYSTEM_ERROR;
    status.critical_fault_code = (uint16_t)code;
    status.critical_fault_arg = argument;
    status.critical_fault_time_ms = status.last_fault_time_ms;
    status.critical_fault_valid = 1u;
  }
}

void configureTimerForRunTimeStats(void) { }

unsigned long getRunTimeCounterValue(void)
{
  return (unsigned long)HAL_GetTick();
}
