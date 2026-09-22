#include "system_status.h"
#include "main.h"
#include <string.h>

static umh_system_status_t status;
static volatile uint32_t time_wrap_count;
static uint8_t time_ready;
static uint8_t status_initialized;
static uint16_t boot_fault_code;
static uint32_t boot_fault_arg;
static uint8_t boot_fault_severity;
static uint16_t boot_critical_code;
static uint32_t boot_critical_arg;

void system_time_init(void)
{
  uint32_t timer_clock;
  /* DWT cycle counter stays enabled for the per-frame service-time metrics;
   * the wall clock however must be multi-task safe.  The previous
   * DWT-delta accumulator was called from several tasks and could count the
   * same cycles twice when preempted between reading CYCCNT and committing
   * the delta, which made every time-based diagnostic (fps, missed
   * deadlines) drift fast under load.  TIM2 gives an independent 1 MHz
   * reference that needs one 32-bit read. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  timer_clock = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0u) timer_clock *= 2u;
  if (timer_clock < 1000000u) timer_clock = 1000000u;
  __HAL_RCC_TIM2_CLK_ENABLE();
  TIM2->CR1 = 0u;
  TIM2->PSC = (uint16_t)(timer_clock / 1000000u - 1u);
  TIM2->ARR = 0xFFFFFFFFu;
  TIM2->CNT = 0u;
  TIM2->EGR = TIM_EGR_UG;
  TIM2->SR = 0u;
  TIM2->CR1 = TIM_CR1_CEN;
  time_wrap_count = 0u;
  time_ready = 1u;
}

uint64_t system_time_us(void)
{
  uint32_t primask;
  uint32_t count;
  uint32_t wraps;
  if (time_ready == 0u) return (uint64_t)HAL_GetTick() * 1000u;
  primask = __get_PRIMASK();
  __disable_irq();
  count = TIM2->CNT;
  wraps = time_wrap_count;
  if ((TIM2->SR & TIM_SR_UIF) != 0u) {
    TIM2->SR = ~TIM_SR_UIF;
    wraps += 1u;
    time_wrap_count = wraps;
    count = TIM2->CNT;
  }
  __set_PRIMASK(primask);
  return ((uint64_t)wraps << 32) | (uint64_t)count;
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
