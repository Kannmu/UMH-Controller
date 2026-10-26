#include "task_stats.h"
#include "main.h"
#include <string.h>

static umh_task_stats_t stats;

/* Previous window state.  Matching by handle keeps the bookkeeping correct even
 * when the scheduler reports the tasks in a different order (a task created or
 * deleted between two samples). */
static TaskHandle_t previous_handle[UMH_TASK_STATS_MAX];
static uint32_t previous_counter[UMH_TASK_STATS_MAX];
static uint32_t previous_total;
static uint32_t previous_tick_ms;
static uint8_t  previous_valid;

/* Averages since the last reset.  Kept here rather than in `stats` so the
 * handles do not have to be mirrored into the public snapshot. */
static TaskHandle_t accum_handle[UMH_TASK_STATS_MAX];
static uint32_t accum_sum_x100[UMH_TASK_STATS_MAX];
static uint32_t accum_windows;
static uint32_t accum_busy_sum_x100;
static uint32_t accum_busy_peak_x100;

static const char *const idle_task_name = "IDLE";

static void accumulators_clear(void)
{
  memset(accum_handle, 0, sizeof(accum_handle));
  memset(accum_sum_x100, 0, sizeof(accum_sum_x100));
  accum_windows = 0u;
  accum_busy_sum_x100 = 0u;
  accum_busy_peak_x100 = 0u;
  stats.samples = 0u;
  stats.avg_busy_x100 = 0u;
  stats.peak_busy_x100 = 0u;
}

void umh_task_stats_init(void)
{
  memset(&stats, 0, sizeof(stats));
  memset(previous_handle, 0, sizeof(previous_handle));
  memset(previous_counter, 0, sizeof(previous_counter));
  previous_total = 0u;
  previous_tick_ms = HAL_GetTick();
  previous_valid = 0u;
  accumulators_clear();
}

void umh_task_stats_update(void)
{
  uint32_t raw_now[UMH_TASK_STATS_MAX];
  uint32_t total = 0u;
  uint32_t now_ms = HAL_GetTick();
  uint32_t window;
  uint32_t idle_x100 = 0u;
  uint32_t busy_x100;
  UBaseType_t i;
  UBaseType_t count;

  if (stats.reset_request != 0u) {
    stats.reset_request = 0u;
    accumulators_clear();
  }

  count = uxTaskGetSystemState(stats.entries, (UBaseType_t)UMH_TASK_STATS_MAX, &total);
  if (count > UMH_TASK_STATS_MAX) count = UMH_TASK_STATS_MAX;
  window = total - previous_total;

  for (i = 0u; i < count; ++i) {
    TaskStatus_t *entry = &stats.entries[i];
    uint32_t raw = entry->ulRunTimeCounter;
    uint32_t delta = 0u;
    uint32_t share = 0u;
    UBaseType_t j;
    raw_now[i] = raw;
    if (previous_valid != 0u) {
      for (j = 0u; j < UMH_TASK_STATS_MAX; ++j) {
        if (previous_handle[j] == entry->xHandle) {
          delta = raw - previous_counter[j];
          break;
        }
      }
    }
    if (window != 0u) share = (uint32_t)(((uint64_t)delta * 10000u) / window);
    if (share > 10000u) share = 10000u;
    entry->ulRunTimeCounter = share;
    if (entry->pcTaskName != NULL &&
        strncmp(entry->pcTaskName, idle_task_name, configMAX_TASK_NAME_LEN) == 0)
      idle_x100 = share;
    for (j = 0u; j < UMH_TASK_STATS_MAX; ++j) {
      if (accum_handle[j] == entry->xHandle) {
        accum_sum_x100[j] += share;
        break;
      }
    }
    if (j == UMH_TASK_STATS_MAX) {
      for (j = 0u; j < UMH_TASK_STATS_MAX; ++j) {
        if (accum_handle[j] == NULL) {
          accum_handle[j] = entry->xHandle;
          accum_sum_x100[j] = share;
          break;
        }
      }
    }
  }
  for (i = 0u; i < UMH_TASK_STATS_MAX; ++i) {
    previous_handle[i] = i < count ? stats.entries[i].xHandle : NULL;
    previous_counter[i] = i < count ? raw_now[i] : 0u;
  }

  busy_x100 = idle_x100 >= 10000u ? 0u : 10000u - idle_x100;
  ++accum_windows;
  accum_busy_sum_x100 += busy_x100;
  if (busy_x100 > accum_busy_peak_x100) accum_busy_peak_x100 = busy_x100;

  stats.count = count;
  stats.window_ms = now_ms - previous_tick_ms;
  stats.busy_percent_x100 = busy_x100;
  stats.samples = accum_windows;
  stats.avg_busy_x100 = accum_windows != 0u ? accum_busy_sum_x100 / accum_windows : 0u;
  stats.peak_busy_x100 = accum_busy_peak_x100;
  previous_tick_ms = now_ms;
  previous_total = total;
  previous_valid = 1u;
}

umh_task_stats_t *umh_task_stats_get(void) { return &stats; }

uint32_t umh_task_stats_percent_x100(TaskHandle_t handle)
{
  uint32_t i;
  for (i = 0u; i < stats.count; ++i)
    if (stats.entries[i].xHandle == handle) return stats.entries[i].ulRunTimeCounter;
  return 0u;
}

uint32_t umh_task_stats_avg_x100(TaskHandle_t handle)
{
  uint32_t i;
  if (accum_windows == 0u) return 0u;
  for (i = 0u; i < UMH_TASK_STATS_MAX; ++i)
    if (accum_handle[i] == handle) return accum_sum_x100[i] / accum_windows;
  return 0u;
}
