#ifndef DEVICE_PROFILE_H
#define DEVICE_PROFILE_H

#include <stdint.h>
#include <stddef.h>

#define UMH_DEVICE_CHANNEL_COUNT 84u
#define UMH_DEVICE_RGB_COUNT 4u
#define UMH_DEVICE_MIC_COUNT 4u
#define UMH_DEVICE_CHANNEL_BITMAP_BYTES 11u
#define UMH_DEVICE_TIMEBASE_HZ 1000000u
#define UMH_DEVICE_FPGA_REF_CLOCK_HZ 42500000u
#define UMH_DEVICE_FRAME_RING_SLOTS 24u
#define UMH_PROFILE_CAP_CHANNEL_STATE (1u << 0)
#define UMH_PROFILE_CAP_SPATIAL_POINT (1u << 1)
#define UMH_PROFILE_CAP_RGB           (1u << 2)
#define UMH_PROFILE_CAP_DIGITAL      (1u << 3)
#define UMH_PROFILE_CAP_EXTENDED     (1u << 4)
#define UMH_PROFILE_CAP_LOOP_RAM     (1u << 5)
#define UMH_PROFILE_CAP_LOOP_STREAM  (1u << 6)
#define UMH_PROFILE_CAP_GEOMETRY_VALID (1u << 7)

_Static_assert(UMH_DEVICE_CHANNEL_COUNT == 84u, "device channel contract");
_Static_assert(UMH_DEVICE_RGB_COUNT == 4u, "device RGB contract");
_Static_assert(UMH_DEVICE_MIC_COUNT == 4u, "device microphone contract");
_Static_assert(UMH_DEVICE_CHANNEL_BITMAP_BYTES == 11u, "channel bitmap contract");

typedef struct __attribute__((packed)) {
  int32_t x_um;
  int32_t y_um;
  int32_t z_um;
} umh_element_coordinate_t;

typedef struct __attribute__((packed)) {
  uint8_t phase;
  uint8_t gain;
  uint8_t enabled;
} umh_channel_calibration_t;

typedef struct __attribute__((packed)) {
  char model[16];
  char firmware[16];
  char protocol[8];
  char serial[24];
  uint16_t channel_count;
  uint16_t rgb_count;
  uint16_t microphone_count;
  uint16_t phase_bits;
  uint16_t intensity_bits;
  uint16_t max_frame_rate;
  uint32_t timebase_hz;
  uint32_t carrier_hz;
  uint32_t sound_speed_um_per_s;
  uint32_t fpga_ref_clock_hz;
  uint32_t ram_bytes;
  uint32_t fpga_fifo_frames;
  uint16_t calibration_version;
  uint32_t calibration_generation;
  uint32_t capability_flags;
  umh_element_coordinate_t coordinates[UMH_DEVICE_CHANNEL_COUNT];
} umh_device_profile_t;

_Static_assert(sizeof(umh_device_profile_t) == 1118u, "device profile wire size");

void device_profile_init(umh_device_profile_t *profile);
const umh_device_profile_t *device_profile_get(void);
void device_profile_set_serial(const char *serial);
void device_profile_set_calibration_generation(umh_device_profile_t *profile,
                                               uint16_t version,
                                               uint32_t generation);

#endif
