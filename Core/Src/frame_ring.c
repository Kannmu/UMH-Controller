#include "frame_ring.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* Index advances are the hot part of this module (one per submitted frame plus
 * one per released frame).  A runtime modulo of a 16-bit value pulls in
 * __aeabi_uidivmod, a library call; the slot count is 24, which is not a power
 * of two, so instead of masking, the increment is tested against the limit and
 * only the (once per 24 frames) wrap pays for the subtraction. */
#define FRAME_RING_SLOT_LIMIT ((uint16_t)UMH_DEVICE_FRAME_RING_SLOTS)

static inline uint16_t frame_ring_next_index(uint16_t index)
{
  uint16_t next = (uint16_t)(index + 1u);
  return next >= FRAME_RING_SLOT_LIMIT ? (uint16_t)(next - FRAME_RING_SLOT_LIMIT) : next;
}

void frame_ring_init(umh_frame_ring_t *ring)
{
  if (ring == NULL) return;
  memset(ring, 0, sizeof(*ring));
}

umh_output_frame_t *frame_ring_acquire_write(umh_frame_ring_t *ring)
{
  if (ring == NULL) return NULL;
  if (ring->count >= UMH_DEVICE_FRAME_RING_SLOTS) return NULL;
  return &ring->slots[ring->write_index];
}

int frame_ring_commit_write(umh_frame_ring_t *ring)
{
  if (ring == NULL || ring->count >= UMH_DEVICE_FRAME_RING_SLOTS) {
    if (ring != NULL) ++ring->dropped;
    return -1;
  }
  ring->write_index = frame_ring_next_index(ring->write_index);
  ++ring->count;
  return 0;
}

umh_output_frame_t *frame_ring_peek_read(umh_frame_ring_t *ring)
{
  if (ring == NULL) return NULL;
  if (ring->count == 0u) return NULL;
  return &ring->slots[ring->read_index];
}

int frame_ring_release_read(umh_frame_ring_t *ring)
{
  if (ring == NULL || ring->count == 0u) return -1;
  ring->read_index = frame_ring_next_index(ring->read_index);
  --ring->count;
  return 0;
}

uint16_t frame_ring_count(const umh_frame_ring_t *ring)
{
  return ring != NULL ? ring->count : 0u;
}

uint16_t frame_ring_free(const umh_frame_ring_t *ring)
{
  return ring != NULL ? (uint16_t)(UMH_DEVICE_FRAME_RING_SLOTS - ring->count) : 0u;
}

/* Freeze the current circular contents as an in-place loop.  The queue can
 * start at any slot: restore_loop walks the same circular order and only
 * shifts deadlines/sequences.  Callers invoke this before playback starts,
 * so the producer is idle and the frame pool cannot be overwritten. */
int frame_ring_snapshot(umh_frame_ring_t *ring, uint64_t duration)
{
  uint16_t last_index;
  uint64_t first_deadline;
  uint64_t last_deadline;
  if (ring == NULL || ring->count == 0u) return -1;
  taskENTER_CRITICAL();
  if (ring->count > UMH_DEVICE_FRAME_RING_SLOTS) {
    taskEXIT_CRITICAL();
    return -1;
  }
  ring->loop_count = ring->count;
  ring->loop_start_index = ring->read_index;
  first_deadline = ring->slots[ring->read_index].deadline;
  last_index = (uint16_t)((ring->read_index + ring->count - 1u) % UMH_DEVICE_FRAME_RING_SLOTS);
  last_deadline = ring->slots[last_index].deadline;
  ring->loop_origin_deadline = first_deadline;
  if (duration == 0u) {
    duration = last_deadline > first_deadline ? last_deadline - first_deadline : 1u;
  }
  ring->loop_duration = duration;
  ring->loop_iteration_last = 0u;
  ring->loop_valid = 1u;
  taskEXIT_CRITICAL();
  return 0;
}

int frame_ring_restore_loop(umh_frame_ring_t *ring, uint32_t iteration)
{
  uint16_t i;
  uint16_t index;
  uint32_t step;
  uint64_t shift;
  if (ring == NULL || ring->loop_valid == 0u || ring->loop_count == 0u) return -1;
  if (iteration <= ring->loop_iteration_last) return -2;
  step = iteration - ring->loop_iteration_last;
  if (ring->loop_duration != 0u && step > UINT64_MAX / ring->loop_duration) return -3;
  shift = (uint64_t)step * ring->loop_duration;
  taskENTER_CRITICAL();
  /* The index walks with the same wrapping increment the producers use instead
   * of recomputing a runtime modulo for every frame in the loop. */
  index = ring->loop_start_index;
  for (i = 0u; i < ring->loop_count; ++i) {
    if (ring->slots[index].deadline > UINT64_MAX - shift) {
      taskEXIT_CRITICAL();
      return -4;
    }
    ring->slots[index].deadline += shift;
    ring->slots[index].sequence += (uint32_t)(step * (uint32_t)ring->loop_count);
    index = frame_ring_next_index(index);
  }
  ring->read_index = ring->loop_start_index;
  ring->write_index = index;
  ring->count = ring->loop_count;
  ring->loop_iteration_last = iteration;
  taskEXIT_CRITICAL();
  return 0;
}

uint8_t frame_ring_loop_valid(const umh_frame_ring_t *ring)
{
  return ring != NULL ? ring->loop_valid : 0u;
}
