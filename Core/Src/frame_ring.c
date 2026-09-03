#include "frame_ring.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

void frame_ring_init(umh_frame_ring_t *ring)
{
  if (ring == NULL) return;
  memset(ring, 0, sizeof(*ring));
}

umh_output_frame_t *frame_ring_acquire_write(umh_frame_ring_t *ring)
{
  if (ring == NULL) return NULL;
  taskENTER_CRITICAL();
  if (ring->count >= UMH_DEVICE_FRAME_RING_SLOTS) {
    taskEXIT_CRITICAL();
    return NULL;
  }
  taskEXIT_CRITICAL();
  return &ring->slots[ring->write_index];
}

int frame_ring_commit_write(umh_frame_ring_t *ring)
{
  int result = 0;
  if (ring == NULL || ring->count >= UMH_DEVICE_FRAME_RING_SLOTS) {
    if (ring != NULL) ++ring->dropped;
    return -1;
  }
  taskENTER_CRITICAL();
  if (ring->count >= UMH_DEVICE_FRAME_RING_SLOTS) {
    ++ring->dropped;
    result = -1;
  } else {
    ring->write_index = (uint16_t)((ring->write_index + 1u) % UMH_DEVICE_FRAME_RING_SLOTS);
    ++ring->count;
  }
  taskEXIT_CRITICAL();
  return result;
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
  taskENTER_CRITICAL();
  if (ring->count == 0u) {
    taskEXIT_CRITICAL();
    return -1;
  }
  ring->read_index = (uint16_t)((ring->read_index + 1u) % UMH_DEVICE_FRAME_RING_SLOTS);
  --ring->count;
  taskEXIT_CRITICAL();
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

int frame_ring_snapshot(umh_frame_ring_t *ring, uint64_t duration)
{
  uint16_t i;
  if (ring == NULL || ring->count == 0u) return -1;
  taskENTER_CRITICAL();
  if (ring->count > UMH_DEVICE_FRAME_RING_SLOTS) {
    taskEXIT_CRITICAL();
    return -1;
  }
  ring->loop_count = ring->count;
  ring->loop_origin_deadline = ring->slots[ring->read_index].deadline;
  for (i = 0u; i < ring->loop_count; ++i) {
    uint16_t index = (uint16_t)((ring->read_index + i) % UMH_DEVICE_FRAME_RING_SLOTS);
    ring->loop_slots[i] = ring->slots[index];
  }
  if (duration == 0u) {
    uint16_t last = (uint16_t)(ring->loop_count - 1u);
    duration = ring->loop_slots[last].deadline - ring->loop_origin_deadline;
    if (duration == 0u) duration = 1u;
  }
  ring->loop_duration = duration;
  ring->loop_valid = 1u;
  taskEXIT_CRITICAL();
  return 0;
}

int frame_ring_restore_loop(umh_frame_ring_t *ring, uint32_t iteration)
{
  uint16_t i;
  uint64_t shift;
  if (ring == NULL || ring->loop_valid == 0u || ring->loop_count == 0u) return -1;
  if (ring->count != 0u) return -2;
  if (ring->loop_duration != 0u && iteration > UINT64_MAX / ring->loop_duration) return -3;
  shift = (uint64_t)iteration * ring->loop_duration;
  taskENTER_CRITICAL();
  for (i = 0u; i < ring->loop_count; ++i) {
    if (ring->loop_slots[i].deadline > UINT64_MAX - shift) {
      taskEXIT_CRITICAL();
      return -4;
    }
    ring->slots[i] = ring->loop_slots[i];
    ring->slots[i].deadline = ring->loop_slots[i].deadline + shift;
    ring->slots[i].sequence = ring->loop_slots[i].sequence +
                              (uint32_t)(iteration * (uint32_t)ring->loop_count);
  }
  ring->read_index = 0u;
  ring->write_index = (uint16_t)(ring->loop_count % UMH_DEVICE_FRAME_RING_SLOTS);
  ring->count = ring->loop_count;
  taskEXIT_CRITICAL();
  return 0;
}

uint8_t frame_ring_loop_valid(const umh_frame_ring_t *ring)
{
  return ring != NULL ? ring->loop_valid : 0u;
}

uint64_t frame_ring_loop_duration(const umh_frame_ring_t *ring)
{
  return ring != NULL ? ring->loop_duration : 0u;
}
