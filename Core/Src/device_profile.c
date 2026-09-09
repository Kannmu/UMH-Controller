#include "device_profile.h"
#include <string.h>
#include "../../Reference/UMH 7 Element Layout/umh7_element_map.h"

_Static_assert((sizeof(UMH7_ELEMENTS) / sizeof(UMH7_ELEMENTS[0])) == UMH_DEVICE_CHANNEL_COUNT,
               "UMH7 geometry table size");

static umh_device_profile_t profile;
static umh_device_profile_t *active_profile;

void device_profile_init(umh_device_profile_t *target)
{
  uint16_t i;
  if (target == NULL) return;
  memset(target, 0, sizeof(*target));
  memcpy(target->model, "UMH-84", 6u);
  memcpy(target->firmware, "v7.0.0", 6u);
  memcpy(target->protocol, "UMH7", 4u);
  target->channel_count = UMH_DEVICE_CHANNEL_COUNT;
  target->rgb_count = UMH_DEVICE_RGB_COUNT;
  target->microphone_count = UMH_DEVICE_MIC_COUNT;
  /* The 2000HC implementation exposes one phase bit and a two-bit level
   * envelope in the parallel output bank.  Input frames remain 16/8-bit and
   * are quantized at the FPGA boundary. */
  target->phase_bits = 16u;
  target->intensity_bits = 8u;
  target->max_frame_rate = 10000u;
  target->timebase_hz = UMH_DEVICE_TIMEBASE_HZ;
  target->carrier_hz = 40000u;
  target->sound_speed_um_per_s = 343000000u;
  target->fpga_ref_clock_hz = UMH_DEVICE_FPGA_REF_CLOCK_HZ;
  target->ram_bytes = 112u * 1024u;
  target->fpga_fifo_frames = 64u;
  target->capability_flags = UMH_PROFILE_CAP_CHANNEL_STATE |
                             UMH_PROFILE_CAP_SPATIAL_POINT |
                             UMH_PROFILE_CAP_RGB |
                             UMH_PROFILE_CAP_DIGITAL |
                             UMH_PROFILE_CAP_EXTENDED |
                             UMH_PROFILE_CAP_LOOP_RAM |
                             UMH_PROFILE_CAP_LOOP_STREAM |
                             UMH_PROFILE_CAP_GEOMETRY_VALID;
  /* The Reference table is indexed by stable element ID (E01..E84), which is
   * also the renderer's firmware channel order.  Keep z at the transducer
   * plane so spatial blocks can use a single documented user coordinate frame. */
  for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) {
    target->coordinates[i].x_um = UMH7_ELEMENTS[i].x_um;
    target->coordinates[i].y_um = UMH7_ELEMENTS[i].y_um;
    target->coordinates[i].z_um = 0;
  }
  active_profile = target;
}

const umh_device_profile_t *device_profile_get(void)
{
  if (active_profile == NULL) {
    device_profile_init(&profile);
  }
  return active_profile;
}

void device_profile_set_serial(const char *serial)
{
  umh_device_profile_t *target = active_profile;
  if (serial == NULL) return;
  if (target == NULL) {
    device_profile_init(&profile);
    target = &profile;
  }
  strncpy(target->serial, serial, sizeof(target->serial) - 1u);
  target->serial[sizeof(target->serial) - 1u] = '\0';
}

void device_profile_set_calibration_generation(umh_device_profile_t *target,
                                               uint16_t version,
                                               uint32_t generation)
{
  if (target == NULL) return;
  target->calibration_version = version;
  target->calibration_generation = generation;
}
