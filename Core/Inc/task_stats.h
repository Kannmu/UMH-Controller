#ifndef TASK_STATS_H
#define TASK_STATS_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

/* Number of tasks the sampler keeps.  The image runs exactly nine (protocol,
 * render, storage, ui, disp, input, health, cal, IDLE); the entry array and the
 * per-handle bookkeeping are sized to that, because every entry is a full
 * TaskStatus_t and SRAM1 has no spare. */
#define UMH_TASK_STATS_MAX 9u

/* Sampled scheduler state, refreshed from the low-priority health task.
 *
 * Every entry except ulRunTimeCounter is what uxTaskGetSystemState() produced.
 * ulRunTimeCounter is replaced with the CPU share of the last window in
 * 1/100 %, because the raw value is an absolute microsecond timestamp: it wraps
 * after 71 minutes and a reader cannot turn it into a percentage without a
 * second sample of its own.  The GUI and a debugger therefore see the same
 * instantaneous load, and the render path no longer has to call into the
 * scheduler to draw the SYSTEM page.
 *
 * A second set of accumulators averages the windows since the last
 * reset_request, so a profile can be taken by clearing the counters, running a
 * workload and reading the sums afterwards, without sampling in the middle of
 * it (a debugger halt on this board interrupts the FPGA link).
 */
typedef struct {
  TaskStatus_t entries[UMH_TASK_STATS_MAX];
  uint32_t count;             /* valid entries in `entries` */
  uint32_t window_ms;         /* wall time covered by the shares */
  uint32_t busy_percent_x100; /* 10000 - IDLE share of the last window */
  uint32_t samples;           /* windows since reset_request */
  uint32_t avg_busy_x100;     /* mean busy_percent_x100 over `samples` */
  uint32_t peak_busy_x100;    /* largest busy_percent_x100 since reset */
  volatile uint32_t reset_request; /* write 1 to restart the accumulators */
} umh_task_stats_t;

void umh_task_stats_init(void);
/* Must be called from task context, from one task only. */
void umh_task_stats_update(void);
umh_task_stats_t *umh_task_stats_get(void);
/* CPU share of `handle` in 1/100 % for the last window, or 0. */
uint32_t umh_task_stats_percent_x100(TaskHandle_t handle);
/* Mean CPU share of `handle` in 1/100 % since the last reset, or 0. */
uint32_t umh_task_stats_avg_x100(TaskHandle_t handle);

#endif
