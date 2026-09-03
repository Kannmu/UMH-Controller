#ifndef FRAME_RING_H
#define FRAME_RING_H

#include <stdint.h>
#include <stddef.h>
#include "device_profile.h"

#define UMH_FRAME_FLAG_ULTRASOUND (1u << 0)
#define UMH_FRAME_FLAG_RGB        (1u << 1)
#define UMH_FRAME_FLAG_DIGITAL    (1u << 2)
#define UMH_FRAME_FLAG_EXTENDED   (1u << 3)

typedef struct __attribute__((packed)) {
  uint16_t phase;
  uint8_t level;
  uint8_t enabled;
} umh_output_channel_t;

typedef struct __attribute__((packed)) {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
} umh_rgb_value_t;

typedef struct __attribute__((packed)) {
  uint32_t sequence;
  uint64_t deadline;
  uint16_t update_flags;
  uint16_t reserved;
  uint8_t digital_state;
  uint8_t digital_mask;
  uint16_t extension_length;
  umh_output_channel_t channels[UMH_DEVICE_CHANNEL_COUNT];
  umh_rgb_value_t rgb[UMH_DEVICE_RGB_COUNT];
  uint8_t extension[32];
} umh_output_frame_t;

_Static_assert(sizeof(umh_output_frame_t) == 400u, "frame slot wire layout");

typedef struct {
  umh_output_frame_t slots[UMH_DEVICE_FRAME_RING_SLOTS];
  /* A second fixed pool makes LOOP_RAM independent of the producer after start. */
  umh_output_frame_t loop_slots[UMH_DEVICE_FRAME_RING_SLOTS];
  volatile uint16_t read_index;
  volatile uint16_t write_index;
  volatile uint16_t count;
  volatile uint32_t dropped;
  uint16_t loop_count;
  uint8_t loop_valid;
  uint8_t reserved;
  uint64_t loop_origin_deadline;
  uint64_t loop_duration;
} umh_frame_ring_t;

void frame_ring_init(umh_frame_ring_t *ring);
umh_output_frame_t *frame_ring_acquire_write(umh_frame_ring_t *ring);
int frame_ring_commit_write(umh_frame_ring_t *ring);
umh_output_frame_t *frame_ring_peek_read(umh_frame_ring_t *ring);
int frame_ring_release_read(umh_frame_ring_t *ring);
uint16_t frame_ring_count(const umh_frame_ring_t *ring);
uint16_t frame_ring_free(const umh_frame_ring_t *ring);
int frame_ring_snapshot(umh_frame_ring_t *ring, uint64_t duration);
int frame_ring_restore_loop(umh_frame_ring_t *ring, uint32_t iteration);
uint8_t frame_ring_loop_valid(const umh_frame_ring_t *ring);
uint64_t frame_ring_loop_duration(const umh_frame_ring_t *ring);

#endif
