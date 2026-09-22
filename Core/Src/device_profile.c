#include "device_profile.h"
#include <string.h>
#include "../../Reference/UMH 7 Element Layout/umh7_element_map.h"

_Static_assert((sizeof(UMH7_ELEMENTS) / sizeof(UMH7_ELEMENTS[0])) == UMH_DEVICE_CHANNEL_COUNT,
               "UMH7 geometry table size");

static umh_device_profile_t profile;
static umh_device_profile_t *active_profile;

const uint8_t umh_device_logical_to_physical[UMH_DEVICE_CHANNEL_COUNT] = {
  21, 53, 34, 37, 20, 57, 43, 48, 47, 23, 29, 30,
  11,  5, 13,  4, 69, 58, 61, 56, 45, 42, 44, 33,
  24, 25, 41, 10, 17,  8, 14, 16, 66, 77, 82, 72,
  79, 62,  3, 49, 46, 54, 28, 36, 22, 31, 27,  6,
  18,  1,  0, 15,  7, 67, 65, 71, 63, 64, 68, 83,
  52, 60, 51, 59, 50, 55, 38, 26, 39, 32, 40, 19,
   9,  2, 12, 35, 80, 75, 74, 73, 70, 76, 81, 78,
};

static const struct { float x_mm, y_mm; } measured_element_position[UMH_DEVICE_CHANNEL_COUNT] = {
  {  -26.0f,  -35.0f}, {  -43.3f,  -15.0f}, {    8.6f,  -45.0f}, {   -0.0f,  -30.0f}, {  -26.0f,  -25.0f}, {   -0.0f,  -10.0f},
  {  -26.0f,   -5.0f}, {   -0.0f,  -40.0f}, {  -17.4f,  -30.0f}, {  -17.4f,  -20.0f}, {  -17.3f,  -10.0f}, {  -43.3f,   -5.0f},
  {   -8.7f,  -35.0f}, {  -34.7f,  -30.0f}, {  -34.7f,  -20.0f}, {   -8.7f,  -25.0f}, {   -8.7f,  -15.0f}, {  -26.0f,  -15.0f},
  {  -34.7f,  -10.0f}, {  -34.7f,    0.0f}, {   -8.7f,   -5.0f}, {    0.0f,   10.0f}, {  -17.3f,    0.0f}, {    0.0f,   30.0f},
  {   -8.7f,   25.0f}, {   -8.7f,    5.0f}, {  -34.7f,   10.0f}, {  -26.0f,   25.0f}, {   -8.7f,   45.0f}, {  -17.3f,   40.0f},
  {  -26.0f,    5.0f}, {  -43.3f,   15.0f}, {  -26.0f,   35.0f}, {   -8.7f,   35.0f}, {   -8.7f,   15.0f}, {  -34.7f,   20.0f},
  {  -17.3f,   30.0f}, {  -17.3f,   20.0f}, {  -26.0f,   15.0f}, {  -17.3f,   10.0f}, {  -43.3f,    5.0f}, {  -34.6f,   30.0f},
  {    0.0f,   20.0f}, {   43.3f,   15.0f}, {    0.0f,   40.0f}, {   17.4f,   40.0f}, {   26.0f,   35.0f}, {    8.7f,   45.0f},
  {    8.7f,   25.0f}, {   17.3f,   30.0f}, {    8.7f,    5.0f}, {   34.7f,   -0.0f}, {   34.7f,   10.0f}, {    8.7f,   35.0f},
  {   17.3f,   20.0f}, {    8.7f,   15.0f}, {   17.3f,   10.0f}, {   17.4f,   -0.0f}, {   34.7f,   30.0f}, {   26.0f,   25.0f},
  {   34.7f,   20.0f}, {   26.0f,   15.0f}, {   26.0f,    5.0f}, {    8.6f,  -35.0f}, {   26.0f,  -25.0f}, {   -0.0f,  -20.0f},
  {   26.0f,  -15.0f}, {   17.3f,  -20.0f}, {   43.3f,  -15.0f}, {   26.0f,   -5.0f}, {   -8.7f,  -45.0f}, {   26.0f,  -35.0f},
  {   43.3f,    5.0f}, {  -17.4f,  -40.0f}, {   17.3f,  -30.0f}, {   17.3f,  -40.0f}, {   34.7f,  -30.0f}, {   34.7f,  -20.0f},
  {   17.4f,  -10.0f}, {   43.3f,   -5.0f}, {    8.6f,  -25.0f}, {    8.6f,  -15.0f}, {   34.7f,  -10.0f}, {    8.7f,   -5.0f},
};

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
  /* The FPGA wire format uses one byte for phase and one byte for level.
   * EEPROM calibration keeps its legacy 16-bit phase storage and is reduced
   * to the wire resolution by the renderer. */
  target->phase_bits = 8u;
  target->intensity_bits = 8u;
  target->max_frame_rate = 20000u;
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
                             UMH_PROFILE_CAP_GEOMETRY_VALID |
                             UMH_PROFILE_CAP_FOCUSED_AM;
  /* Channel order is the FPGA us_tx bit / SPI serialization order.  The x/y
   * values are the measured transducer positions recovered from the 4-mic
   * short-burst time-of-flight mapping (2026-09 bench).  The theoretical
   * hex-lattice E## table in Reference/UMH 7 Element Layout is kept as a
   * sanity reference, but the physical board is rotated relative to that
   * table and the netlist us_tx order is not E01..E84, so the measured table
   * is authoritative for focusing.  GU1008C-40TR piezo ceramic sits 7.0 mm
   * above the PCB microphone-port plane, hence z=+7000 um. */
  for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) {
    target->coordinates[i].x_um = (int32_t)(measured_element_position[i].x_mm * 1000.0f + (measured_element_position[i].x_mm >= 0.0f ? 0.5f : -0.5f));
    target->coordinates[i].y_um = (int32_t)(measured_element_position[i].y_mm * 1000.0f + (measured_element_position[i].y_mm >= 0.0f ? 0.5f : -0.5f));
    target->coordinates[i].z_um = 7000;
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
