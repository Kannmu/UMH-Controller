#ifndef US_CALIBRATION_H
#define US_CALIBRATION_H

#include <stdint.h>
#include "fpga_link.h"
#include "device_profile.h"

/* Quality flags reported by the built-in near-field coupling calibration.
 * Bit assignments are preserved from the previous echo calibration; the
 * meanings are updated to the v2 flow. */
#define CAL_Q_COUPLING    (1u << 0)
#define CAL_Q_MICS        (1u << 1)
#define CAL_Q_FIT         (1u << 2)
#define CAL_Q_CONSISTENCY (1u << 3)
#define CAL_Q_LEVEL       (1u << 4)
#define CAL_Q_GEOM        (1u << 5)
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
  uint8_t phase_byte[UMH_DEVICE_CHANNEL_COUNT]; /* correction added to command phase */
  uint8_t progress;
  uint8_t good_mics;
  uint8_t fault;              /* UMH_FAULT_* code on failure */
  uint8_t quality_flags;      /* CAL_Q_* bits that failed the quality gates */
  uint8_t used_gate_count;
  uint8_t used_gate_width;    /* 40 kHz samples (25 us) */
  uint8_t level_used;         /* selected drive level (ladder 128..8) */
  uint8_t geom_hypothesis;    /* 0 = fixed coordinate table */
  uint8_t sign_hypothesis;    /* 0..3 candidate index chosen by run-time gain */
  uint8_t reserved0;
  uint16_t used_gate_start;   /* 40 kHz samples after the FPGA pattern swap */
  uint16_t patterns_used;
  uint16_t reserved1;
  float fit_rms_deg;          /* circular phase residual of the direct-path fit */
  float mic_consistency_deg;  /* cross-microphone consistency */
  float residual;             /* relative complex residual in the fit domain */
  float drift_deg;            /* first/second half residual drift */
  float band_trend_deg;       /* max-min mean residual over 3 distance bands */
  float verify_gain_db;       /* measured focusing gain of chosen candidate */
  float coupling_db;          /* microphone coupling spread */
  float rms_before_deg;       /* static phase spread before correction */
  float rms_after_deg;        /* quantized residual after correction */
} umh_calibration_result_t;

typedef void (*us_cal_progress_cb_t)(uint8_t state, uint8_t progress, void *context);

typedef struct {
  uint8_t progress;
  uint8_t fault;
  uint8_t pass;
  uint8_t good_mics;
  uint8_t used_level;
  uint8_t used_gate_width;
  uint16_t used_gate_start;
  uint16_t channels_measured;
  float focus_gain_db;             /* measured focus power / random-phase control */
  float coherence;                 /* direct H_i + production focus phase coherence */
  float predicted_gain_db;         /* coherent gain predicted from single-channel H_i */
  float actual_vs_predicted_db;    /* measured focus power / predicted coherent power */
  float per_mic_gain_db[UMH_DEVICE_MIC_COUNT];
  float per_mic_coherence[UMH_DEVICE_MIC_COUNT];
  float per_mic_predicted_gain_db[UMH_DEVICE_MIC_COUNT];
  float per_mic_actual_vs_predicted_db[UMH_DEVICE_MIC_COUNT];
} umh_cal_self_test_result_t;

/* Bench-only streaming callback for raw per-pattern microphone I/Q capture.
 * data is int16 little-endian [pattern][mic][I,Q], 16 bytes per pattern. */
typedef int (*us_cal_raw_tx_cb_t)(uint32_t first_pattern_index,
                                  const uint8_t *data, uint16_t length,
                                  void *context);

int us_calibration_run(fpga_link_t *link, const umh_device_profile_t *profile,
                       us_cal_progress_cb_t progress, void *context,
                       umh_calibration_result_t *result);

int us_calibration_capture_raw(fpga_link_t *link, uint8_t level,
                               uint16_t gate_start, uint8_t gate_width,
                               uint32_t burst_us, uint16_t patterns,
                               us_cal_raw_tx_cb_t tx, void *context);

int us_calibration_self_test(fpga_link_t *link, const umh_device_profile_t *profile,
                             const umh_channel_calibration_t *calibration,
                             uint8_t level, uint16_t gate_start, uint8_t gate_width,
                             uint32_t burst_us,
                             us_cal_progress_cb_t progress, void *context,
                             umh_cal_self_test_result_t *result);

/* Bench instrumentation: measure the 4x84 differential H matrix only, with
 * configurable drive level/gate and coherent repeat averaging.  The result is
 * left in the internal cal_z_re/cal_z_im arrays and can be read over SWD. */
int us_calibration_measure_h(fpga_link_t *link, const umh_device_profile_t *profile,
                             uint8_t level, uint16_t gate_start, uint8_t gate_width,
                             uint32_t burst_us, uint8_t repeats,
                             us_cal_progress_cb_t progress, void *context);

/* Bench instrumentation: short-burst arrival profile for one logical channel.
 * channel >= UMH_DEVICE_CHANNEL_COUNT drives nothing (noise floor).
 * The averaged per-gate I/Q is left in cal_profile_i/cal_profile_q, indexed
 * [gate][microphone], and can be read over SWD. */
int us_calibration_measure_profile(fpga_link_t *link, uint8_t channel,
                                   uint8_t phase, uint8_t level,
                                   uint8_t gate_count, uint16_t gate_start,
                                   uint16_t gate_step, uint8_t gate_width,
                                   uint32_t burst_us, uint8_t repeats);

typedef struct __attribute__((packed)) {
  uint8_t early_gate;
  uint8_t late_gate;
  int16_t late_i;
  int16_t late_q;
} us_cal_profile_peak_t;

/* Bench instrumentation: run the short-burst arrival profile for every
 * logical channel and reduce it to early/late peak per microphone.  The
 * caller supplies a UMH_DEVICE_CHANNEL_COUNT x UMH_DEVICE_MIC_COUNT array. */
/* Bench instrumentation: drive a complete arbitrary 84-channel frame and
 * return the four microphone gate phasors. */
int us_calibration_measure_pattern(fpga_link_t *link,
                                   const uint8_t *phase, const uint8_t *level,
                                   uint16_t gate_start, uint8_t gate_width,
                                   uint32_t burst_us, uint8_t repeats,
                                   float out_i[UMH_DEVICE_MIC_COUNT],
                                   float out_q[UMH_DEVICE_MIC_COUNT]);

int us_calibration_measure_profile_all(fpga_link_t *link,
                                       uint8_t level, uint32_t burst_us,
                                       uint8_t gate_count, uint16_t gate_start,
                                       uint16_t gate_step, uint8_t gate_width,
                                       uint8_t repeats, uint8_t min_gate,
                                       us_cal_profile_peak_t *peaks);

/* Raw diagnostic dump sections.  See Docs/UMH_v7_Protocol.md. */
uint32_t us_calibration_dump_size(uint8_t section);
int us_calibration_dump_read(uint8_t section, uint32_t offset, uint8_t *out, uint16_t length);

#endif
