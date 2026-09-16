#ifndef US_CALIBRATION_H
#define US_CALIBRATION_H

#include <stdint.h>
#include "fpga_link.h"
#include "device_profile.h"
#define CAL_Q_ECHO        (1u << 0)
#define CAL_Q_MIC_RATIO   (1u << 1)
#define CAL_Q_RESIDUAL    (1u << 2)
#define CAL_Q_CONSISTENCY (1u << 3)
#define CAL_Q_PRERMS      (1u << 4)
#define CAL_Q_TILT        (1u << 5)
#define CAL_Q_VERIFY      (1u << 6)
#define CAL_Q_MEASURE     (1u << 7)

/* State numbers intentionally mirror device_gui_cal_state_t. */
typedef enum {
  US_CAL_IDLE = 0u,
  US_CAL_WAIT,
  US_CAL_MEASURE,
  US_CAL_SOLVE,
  US_CAL_VERIFY,
  US_CAL_OK,
  US_CAL_FAIL
} us_cal_state_t;

typedef struct {
  uint8_t phase_byte[UMH_DEVICE_CHANNEL_COUNT]; /* correction to add to the channel command */
  uint8_t progress;
  uint8_t good_mics;
  uint16_t used_gate_start;   /* 10 us units */
  uint8_t used_gate_count;
  uint8_t used_gate_width;    /* 10 us units */
  uint8_t fault;              /* UMH_FAULT_* code on failure */
  uint8_t quality_flags;      /* CAL_Q_* bits for the failed quality gate */
  uint8_t reserved;
  float rms_before_deg;
  float rms_after_deg;
  float mic_consistency_deg;
  float residual;
  float distance_m;
  float tilt_x_deg;
  float tilt_y_deg;
  float echo_ratio;
  float mic_ratio;
  float verify_gain_db;
  uint16_t final_block_count;
} umh_calibration_result_t;

typedef void (*us_cal_progress_cb_t)(uint8_t state, uint8_t progress, void *context);

int us_calibration_run(fpga_link_t *link, const umh_device_profile_t *profile,
                       us_cal_progress_cb_t progress, void *context,
                       umh_calibration_result_t *result);

/* Raw final-scan I/Q dump for offline diagnosis (int16 little-endian,
 * layout [gate][pattern][mic][i,q]). */
uint32_t us_calibration_raw_size(void);
int us_calibration_raw_read(uint32_t offset, uint8_t *out, uint16_t length);

#endif
