/* v7 application tasks and static runtime objects. */
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "device_profile.h"
#include "umh_protocol.h"
#include "spatiotemporal_block.h"
#include "block_parser.h"
#include "frame_ring.h"
#include "spatial_renderer.h"
#include "playback_plan.h"
#include "fpga_link.h"
#include "flash_nor.h"
#include "flash_store.h"
#include "eeprom_profile.h"
#include "oled_ssd1315.h"
#include "input_events.h"
#include "system_status.h"
#include "device_gui.h"
#include "us_calibration.h"
#include "demo_engine.h"
#include "motion_engine.h"
#include "audio_engine.h"
#include "spi.h"
#include "i2c.h"
#include "i2c_bus.h"
#include <string.h>
#include <math.h>

osThreadId_t umh_protocol_task_handle;
umh_rx_ring_t umh_usb_rx_ring;

static umh_device_profile_t device_profile;
static umh_spatial_renderer_t renderer;
static umh_frame_ring_t frame_ring;
static umh_block_parser_t block_parser;
static umh_playback_plan_t playback_plan;
static umh_audio_engine_t audio_engine;
static umh_motion_engine_t motion_engine;
static fpga_link_t fpga_link;
static flash_store_t flash_store;
static eeprom_profile_t eeprom_profile;
static oled_ssd1315_t oled;
static umh_protocol_parser_t protocol_parser;
static device_gui_t device_gui;
static umh_calibration_result_t calibration_result;
static umh_cal_self_test_result_t calibration_self_test_result;
static osThreadId_t calibration_task_handle;
static volatile uint8_t calibration_raw_mode;
static volatile uint8_t calibration_selftest_mode;
static uint8_t calibration_raw_level;
static uint16_t calibration_raw_gate_start;
static uint8_t calibration_raw_gate_width;
static uint16_t calibration_raw_burst_us;
static uint16_t calibration_raw_patterns;

/* --------------------------------------------------------------------------
 * Bench mailbox.
 *
 * The ST-Link/OpenOCD bench session cannot use USB CDC on every setup.  These
 * words are deliberately plain globals so a debugger can write a command and
 * poll the status/side effects while the calibration task services it.
 * They have no effect in production until a debugger writes umh_bench_cmd.
 * -------------------------------------------------------------------------- */
enum {
  UMH_BENCH_CMD_NONE = 0u,
  UMH_BENCH_CMD_SELFTEST = 1u,
  UMH_BENCH_CMD_CALIBRATE = 2u,
  UMH_BENCH_CMD_MEASURE_H = 3u,
  UMH_BENCH_CMD_APPLY_PHASE = 4u,
  UMH_BENCH_CMD_COMMIT = 5u,
  UMH_BENCH_CMD_START_DEMO = 6u,
  UMH_BENCH_CMD_PROFILE = 7u,
  UMH_BENCH_CMD_PROFILE_ALL = 8u,
  UMH_BENCH_CMD_PATTERN = 9u
};
volatile uint32_t umh_bench_cmd;
volatile uint32_t umh_bench_status;   /* 0 idle, 1 busy, 2 done, 3 error */
volatile uint32_t umh_bench_result;   /* last command return code        */
volatile uint32_t umh_bench_arg[16];
volatile uint8_t  umh_bench_phase[UMH_DEVICE_CHANNEL_COUNT];
volatile uint8_t  umh_bench_level[UMH_DEVICE_CHANNEL_COUNT];
volatile float    umh_bench_frame_i[UMH_DEVICE_MIC_COUNT];
volatile float    umh_bench_frame_q[UMH_DEVICE_MIC_COUNT];
us_cal_profile_peak_t umh_bench_peaks[UMH_DEVICE_CHANNEL_COUNT][UMH_DEVICE_MIC_COUNT];

typedef struct {
  umh_protocol_frame_t frame;
} storage_request_t;

static osMessageQueueId_t storage_queue;
static StaticQueue_t storage_queue_cb;
static uint32_t storage_queue_memory[(sizeof(storage_request_t) * 2u + sizeof(uint32_t) - 1u) / sizeof(uint32_t)];
static storage_request_t storage_request_buffer;
static uint8_t storage_response[UMH_PROTOCOL_MAX_PAYLOAD];
static uint8_t storage_data[UMH_PROTOCOL_MAX_PAYLOAD];
static flash_store_record_t storage_records[FLASH_STORE_MAX_OBJECTS];
static eeprom_profile_record_t eeprom_pending;
static uint8_t eeprom_pending_valid;
typedef struct __attribute__((packed)) {
  uint8_t phase[UMH_DEVICE_CHANNEL_COUNT];
  uint8_t progress;
  uint8_t good_mics;
  uint8_t fault;
  uint8_t quality_flags;
  uint8_t reserved;
  uint8_t used_gate_count;
  uint8_t used_gate_width;
  uint16_t used_gate_start;
  uint16_t block_count;
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
} umh_cal_result_wire_t;

_Static_assert(sizeof(umh_cal_result_wire_t) == 135u, "cal result wire size");

typedef struct __attribute__((packed)) {
  uint8_t valid;
  uint8_t pass;
  uint8_t good_mics;
  uint8_t fault;
  uint16_t channels_measured;
  uint16_t used_gate_start;
  uint8_t used_level;
  uint8_t used_gate_width;
  uint8_t reserved[2];
  int16_t focus_gain_db_x10;
  uint16_t coherence_x1000;
  int16_t predicted_gain_db_x10;
  int16_t actual_vs_predicted_db_x10;
  int16_t per_mic_gain_db_x10[UMH_DEVICE_MIC_COUNT];
  uint16_t per_mic_coherence_x1000[UMH_DEVICE_MIC_COUNT];
} umh_cal_self_test_wire_t;

_Static_assert(sizeof(umh_cal_self_test_wire_t) == 36u, "cal self-test wire size");

static uint8_t calibration_dump_buffer[512];
static uint8_t block_stream_active;
static uint32_t block_stream_expected_sequence;
static volatile uint8_t demo_building;
static TaskHandle_t render_task_handle;
static TIM_HandleTypeDef render_timer;
static volatile uint8_t render_timer_ready;

static umh_fault_code_t fault_code_for_status(umh_status_t status, uint8_t message_type)
{
  if (status == UMH_STATUS_IO) {
    if (message_type == UMH_MSG_EEPROM_COMMIT || message_type == UMH_MSG_EEPROM_WRITE)
      return UMH_FAULT_EEPROM_IO;
    if (message_type >= UMH_MSG_FLASH_LIST && message_type <= UMH_MSG_FLASH_DELETE)
      return UMH_FAULT_FLASH_IO;
    return UMH_FAULT_FPGA_OUTPUT;
  }
  if (message_type == UMH_MSG_SET_PLAN || message_type == UMH_MSG_START_PLAN ||
      message_type == UMH_MSG_STOP_PLAN || message_type == UMH_MSG_CLEAR_PLAN)
    return UMH_FAULT_PLAN_START;
  if (message_type == UMH_MSG_BLOCK_BEGIN || message_type == UMH_MSG_BLOCK_DATA ||
      message_type == UMH_MSG_BLOCK_END || message_type == UMH_MSG_BLOCK_CANCEL)
    return UMH_FAULT_BLOCK_PARSE;
  return UMH_FAULT_PROTOCOL_NACK;
}

static StaticTask_t protocol_task_cb, render_task_cb, storage_task_cb, ui_task_cb, health_task_cb;
static StaticTask_t input_task_cb;
static StackType_t input_task_stack[128];
static const osThreadAttr_t input_task_attributes = {
  .name = "input", .cb_mem = &input_task_cb, .cb_size = sizeof(input_task_cb),
  .stack_mem = input_task_stack, .stack_size = sizeof(input_task_stack),
  .priority = osPriorityNormal
};
/* The demo renderer uses floating point temporaries through a deep call chain.
 * Keep those temporaries out of the small UI stack so selecting a demo cannot
 * corrupt the scheduler state. */
static StackType_t protocol_task_stack[1024], render_task_stack[1024], storage_task_stack[512], ui_task_stack[1024], health_task_stack[256];
static const osThreadAttr_t protocol_task_attributes = {
  .name = "protocol", .cb_mem = &protocol_task_cb, .cb_size = sizeof(protocol_task_cb),
  .stack_mem = protocol_task_stack, .stack_size = sizeof(protocol_task_stack),
  .priority = osPriorityHigh
};
static const osThreadAttr_t render_task_attributes = {
  .name = "render", .cb_mem = &render_task_cb, .cb_size = sizeof(render_task_cb),
  .stack_mem = render_task_stack, .stack_size = sizeof(render_task_stack),
  /* Focused-AM needs a deterministic 50 us service cadence.  Keep the render
   * task above the USB protocol task; ring/queue decoupling means the
   * protocol task can still drain USB bursts when render sleeps. */
  .priority = osPriorityRealtime
};
static const osThreadAttr_t storage_task_attributes = {
  .name = "storage", .cb_mem = &storage_task_cb, .cb_size = sizeof(storage_task_cb),
  .stack_mem = storage_task_stack, .stack_size = sizeof(storage_task_stack),
  .priority = osPriorityLow
};
static const osThreadAttr_t ui_task_attributes = {
  .name = "ui", .cb_mem = &ui_task_cb, .cb_size = sizeof(ui_task_cb),
  .stack_mem = ui_task_stack, .stack_size = sizeof(ui_task_stack),
  .priority = osPriorityBelowNormal
};
static StaticTask_t calibration_task_cb;
static StackType_t calibration_task_stack[1024];
static const osThreadAttr_t calibration_task_attributes = {
  .name = "cal", .cb_mem = &calibration_task_cb, .cb_size = sizeof(calibration_task_cb),
  .stack_mem = calibration_task_stack, .stack_size = sizeof(calibration_task_stack),
  .priority = osPriorityNormal
};
static const osThreadAttr_t health_task_attributes = {
  .name = "health", .cb_mem = &health_task_cb, .cb_size = sizeof(health_task_cb),
  .stack_mem = health_task_stack, .stack_size = sizeof(health_task_stack),
  .priority = osPriorityLow
};

static void send_response(const umh_protocol_frame_t *request, uint8_t type,
                          const void *payload, uint16_t length);
static void application_init(void);
static int gui_demo(void *context);

static uint32_t read_u32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void apply_calibration_from_eeprom(void)
{
  umh_channel_calibration_t calibration[UMH_DEVICE_CHANNEL_COUNT];
  const eeprom_profile_record_t *record = eeprom_profile_current(&eeprom_profile);
  uint16_t i;
  uint16_t enabled_count = 0u;
  if (record == NULL) return;
  for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) {
    calibration[i].phase = (uint8_t)(record->phase[i] >> 8);
    calibration[i].gain = record->gain[i];
    calibration[i].enabled = (uint8_t)((record->enabled[i / 8u] >> (i % 8u)) & 1u);
    if (calibration[i].enabled != 0u) ++enabled_count;
  }
  if (enabled_count == 0u) {
    for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) calibration[i].enabled = 1u;
  }
  spatial_renderer_set_calibration(&renderer, calibration, UMH_DEVICE_CHANNEL_COUNT);
  spatial_renderer_set_rgb_calibration(&renderer, record->rgb_gain);
  device_profile_set_calibration_generation(&device_profile, record->version,
                                            record->generation);
  if (record->cal_meta_valid != 0u) {
    umh_system_status_t *s = system_status_get();
    s->cal_rms_deg_x10 = record->cal_rms_deg_x10;
    s->cal_tilt_x_x10 = record->cal_tilt_x_x10;
    s->cal_tilt_y_x10 = record->cal_tilt_y_x10;
    s->cal_state = DEVICE_GUI_CAL_OK;
    s->cal_last_ms = HAL_GetTick();
  }
  system_status_set(UMH_SYSTEM_CALIBRATION_VALID);
}

static int gui_calibration(void *context)
{
  (void)context;
  if (calibration_task_handle == NULL) return -1;
  if (audio_engine_owns_output(&audio_engine) != 0u) audio_engine_abort(&audio_engine);
  if (motion_engine_owns_output(&motion_engine) != 0u) motion_engine_abort(&motion_engine, &fpga_link);
  device_gui_calibration_begin(&device_gui);
  (void)xTaskNotifyGive((TaskHandle_t)calibration_task_handle);
  return 0;
}

static int gui_selftest(void *context)
{
  (void)context;
  if (calibration_task_handle == NULL) return -1;
  if (audio_engine_owns_output(&audio_engine) != 0u) audio_engine_abort(&audio_engine);
  if (motion_engine_owns_output(&motion_engine) != 0u) motion_engine_abort(&motion_engine, &fpga_link);
  device_gui_calibration_begin(&device_gui);
  calibration_selftest_mode = 1u;
  calibration_raw_mode = 0u;
  (void)xTaskNotifyGive((TaskHandle_t)calibration_task_handle);
  return 0;
}

static void calibration_progress(uint8_t state, uint8_t progress, void *context)
{
  (void)context;
  device_gui_calibration_state(&device_gui, state, progress);
  system_status_get()->cal_state = state;
  system_status_get()->cal_last_ms = HAL_GetTick();
}

static void calibration_fail(umh_fault_code_t code, uint32_t argument)
{
  calibration_progress(DEVICE_GUI_CAL_FAIL, 100u, NULL);
  system_status_fault(code, argument, UMH_FAULT_WARNING);
}

static int calibration_raw_tx(uint32_t first_pattern, const uint8_t *data,
                              uint16_t length, void *context)
{
  uint8_t encoded[UMH_PROTOCOL_HEADER_SIZE + UMH_PROTOCOL_MAX_PAYLOAD];
  uint16_t encoded_length;
  (void)context;
  encoded_length = umh_protocol_encode(UMH_MSG_CAL_RAW, UMH_FLAG_RESPONSE,
                                       0x00524157u, first_pattern,
                                       data, length, encoded, sizeof(encoded));
  if (encoded_length == 0u) return -1;
  while (umh_usb_tx_enqueue(encoded, encoded_length) == 0u) {
    umh_usb_tx_service();
    osDelay(1u);
  }
  umh_usb_tx_service();
  return 0;
}

static void calibration_raw_session(void)
{
  int rc;
  if (audio_engine_owns_output(&audio_engine) != 0u) audio_engine_abort(&audio_engine);
  if (motion_engine_owns_output(&motion_engine) != 0u) motion_engine_abort(&motion_engine, &fpga_link);
  device_gui_calibration_begin(&device_gui);
  playback_plan_stop(&playback_plan);
  (void)fpga_link_safe_stop(&fpga_link);
  block_parser_cancel(&block_parser);
  frame_ring_init(&frame_ring);
  block_stream_active = 0u;
  demo_building = 1u;
  calibration_progress(DEVICE_GUI_CAL_WAIT, 0u, NULL);
  osDelay(1000u);
  rc = us_calibration_capture_raw(&fpga_link, calibration_raw_level,
                                  calibration_raw_gate_start,
                                  calibration_raw_gate_width,
                                  calibration_raw_burst_us,
                                  calibration_raw_patterns,
                                  calibration_raw_tx, NULL);
  if (rc != 0) {
    calibration_fail(rc == -2 ? UMH_FAULT_CAL_MIC_SILENT : UMH_FAULT_CAL_SOLVER,
                     (uint32_t)(-rc));
  } else {
    calibration_progress(DEVICE_GUI_CAL_OK, 100u, NULL);
  }
  (void)fpga_link_safe_stop(&fpga_link);
  demo_building = 0u;
}

static void calibration_selftest_session(void)
{
  int rc;
  const eeprom_profile_record_t *record = eeprom_profile_current(&eeprom_profile);
  uint8_t level = (record != NULL && record->cal_level != 0u) ? (uint8_t)record->cal_level : 8u;
  uint16_t gate_start = 0u;
  uint8_t gate_width = 0u;
  if (record != NULL && record->cal_meta_valid != 0u) {
    /* Re-use the same steady-state gate that produced the stored record.  The
     * acoustic self-test must measure the real calibrated array, not a
     * different operating point chosen by the transient detector. */
    memcpy(&gate_start, &record->cal_reserved[2], sizeof(gate_start));
    gate_width = record->cal_reserved[4];
  }
  if (audio_engine_owns_output(&audio_engine) != 0u) audio_engine_abort(&audio_engine);
  if (motion_engine_owns_output(&motion_engine) != 0u) motion_engine_abort(&motion_engine, &fpga_link);
  device_gui_calibration_begin(&device_gui);
  playback_plan_stop(&playback_plan);
  (void)fpga_link_safe_stop(&fpga_link);
  block_parser_cancel(&block_parser);
  frame_ring_init(&frame_ring);
  block_stream_active = 0u;
  demo_building = 1u;
  calibration_progress(DEVICE_GUI_CAL_WAIT, 0u, NULL);
  osDelay(300u);
  rc = us_calibration_self_test(&fpga_link, device_profile_get(), renderer.calibration,
                                level, gate_start, gate_width, 0u,
                                calibration_progress, NULL,
                                &calibration_self_test_result);
  if (rc == 0) {
    umh_system_status_t *st = system_status_get();
    float coh = calibration_self_test_result.coherence * 1000.0f;
    st->self_test_valid = 1u;
    st->self_test_pass = calibration_self_test_result.pass;
    st->self_test_good_mics = calibration_self_test_result.good_mics;
    st->self_test_gain_x10 = (int16_t)lroundf(calibration_self_test_result.focus_gain_db * 10.0f);
    if (coh > 65535.0f) coh = 65535.0f;
    if (coh < 0.0f) coh = 0.0f;
    st->self_test_coherence_x1000 = (uint16_t)lroundf(coh);
    if (calibration_self_test_result.pass != 0u)
      calibration_progress(DEVICE_GUI_CAL_OK, 100u, NULL);
    else
      calibration_fail(UMH_FAULT_CAL_QUALITY, 4u);
  } else {
    calibration_fail((umh_fault_code_t)calibration_self_test_result.fault, (uint32_t)(-rc));
  }
  (void)fpga_link_safe_stop(&fpga_link);
  demo_building = 0u;
}

static void calibration_session(void)
{
  int rc;
  uint16_t i;
  const umh_device_profile_t *profile = device_profile_get();
  if (audio_engine_owns_output(&audio_engine) != 0u) audio_engine_abort(&audio_engine);
  if (motion_engine_owns_output(&motion_engine) != 0u) motion_engine_abort(&motion_engine, &fpga_link);
  device_gui_calibration_begin(&device_gui);
  playback_plan_stop(&playback_plan);
  (void)fpga_link_safe_stop(&fpga_link);
  block_parser_cancel(&block_parser);
  frame_ring_init(&frame_ring);
  block_stream_active = 0u;
  demo_building = 1u;
  calibration_progress(DEVICE_GUI_CAL_WAIT, 0u, NULL);
  osDelay(1000u);
  rc = us_calibration_run(&fpga_link, profile, calibration_progress, NULL,
                          &calibration_result);
  if (rc == 0) {
    eeprom_profile_record_t record = eeprom_profile.record;
    record.version = EEPROM_PROFILE_VERSION;
    for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) {
      record.phase[i] = (uint16_t)((uint16_t)calibration_result.phase_byte[i] << 8);
      record.enabled[i / 8u] |= (uint8_t)(1u << (i % 8u));
    }
    if (UMH_DEVICE_CHANNEL_COUNT % 8u != 0u) {
      uint8_t mask = (uint8_t)((1u << (UMH_DEVICE_CHANNEL_COUNT % 8u)) - 1u);
      record.enabled[UMH_DEVICE_CHANNEL_BITMAP_BYTES - 1u] &= mask;
    }
    record.cal_meta_valid = 1u;
    {
      float rms10 = calibration_result.fit_rms_deg * 10.0f + 0.5f;
      if (rms10 > 255.0f) rms10 = 255.0f;
      record.cal_rms_deg_x10 = (uint8_t)rms10;
    }
    {
      float gx10 = calibration_result.verify_gain_db * 10.0f;
      float cx10 = calibration_result.mic_consistency_deg * 10.0f;
      if (gx10 > 127.0f) gx10 = 127.0f;
      if (gx10 < -128.0f) gx10 = -128.0f;
      if (cx10 > 127.0f) cx10 = 127.0f;
      if (cx10 < -128.0f) cx10 = -128.0f;
      record.cal_tilt_x_x10 = (int8_t)lroundf(gx10);
      record.cal_tilt_y_x10 = (int8_t)lroundf(cx10);
    }
    record.cal_carrier_hz10 = (uint16_t)(device_profile.carrier_hz / 10u);
    record.cal_level = calibration_result.level_used;
    record.cal_time_ms = (uint32_t)(system_time_us() / 1000u);
    memset(record.cal_reserved, 0, sizeof(record.cal_reserved));
    {
      uint16_t patterns = calibration_result.patterns_used;
      uint16_t gate_start = calibration_result.used_gate_start;
      int16_t trend10 = (int16_t)lroundf(calibration_result.band_trend_deg * 10.0f);
      memcpy(&record.cal_reserved[0], &patterns, sizeof(patterns));
      memcpy(&record.cal_reserved[2], &gate_start, sizeof(gate_start));
      record.cal_reserved[4] = calibration_result.used_gate_width;
      record.cal_reserved[5] = calibration_result.level_used;
      record.cal_reserved[6] = calibration_result.geom_hypothesis;
      record.cal_reserved[7] = calibration_result.sign_hypothesis;
      memcpy(&record.cal_reserved[8], &trend10, sizeof(trend10));
      record.reserved[0] = calibration_result.used_gate_count;
      record.reserved[1] = 0u;
      record.reserved[2] = 0u;
    }
    if (eeprom_profile_commit(&eeprom_profile, &record) != 0) {
      calibration_fail(UMH_FAULT_EEPROM_IO, 3u);
    } else {
      apply_calibration_from_eeprom();
      system_status_get()->cal_rms_deg_x10 = record.cal_rms_deg_x10;
      system_status_get()->cal_tilt_x_x10 = record.cal_tilt_x_x10;
      system_status_get()->cal_tilt_y_x10 = record.cal_tilt_y_x10;
      system_status_get()->cal_good_mics = calibration_result.good_mics;
      system_status_get()->cal_last_ms = HAL_GetTick();
    }
  } else {
    umh_fault_code_t code = (umh_fault_code_t)calibration_result.fault;
    if (code == UMH_FAULT_NONE) code = UMH_FAULT_CAL_SOLVER;
    calibration_fail(code, (uint32_t)rc);
  }
  (void)fpga_link_safe_stop(&fpga_link);
  demo_building = 0u;
}


static void bench_service(void)
{
  uint32_t cmd = umh_bench_cmd;
  int rc = 0;
  uint16_t i;
  if (cmd == UMH_BENCH_CMD_NONE) return;
  if (audio_engine_owns_output(&audio_engine) != 0u) audio_engine_abort(&audio_engine);
  if (motion_engine_owns_output(&motion_engine) != 0u) motion_engine_abort(&motion_engine, &fpga_link);
  umh_bench_cmd = UMH_BENCH_CMD_NONE;
  umh_bench_status = 1u;
  umh_bench_result = 0u;
  switch (cmd) {
    case UMH_BENCH_CMD_SELFTEST:
      calibration_selftest_session();
      break;
    case UMH_BENCH_CMD_CALIBRATE:
      calibration_session();
      break;
    case UMH_BENCH_CMD_MEASURE_H:
      playback_plan_stop(&playback_plan);
      (void)fpga_link_safe_stop(&fpga_link);
      block_parser_cancel(&block_parser);
      frame_ring_init(&frame_ring);
      block_stream_active = 0u;
      demo_building = 1u;
      rc = us_calibration_measure_h(&fpga_link, device_profile_get(),
                                    (uint8_t)umh_bench_arg[0],
                                    (uint16_t)umh_bench_arg[1],
                                    (uint8_t)umh_bench_arg[2],
                                    umh_bench_arg[3],
                                    (uint8_t)umh_bench_arg[4],
                                    NULL, NULL);
      (void)fpga_link_safe_stop(&fpga_link);
      demo_building = 0u;
      break;
    case UMH_BENCH_CMD_PROFILE:
      playback_plan_stop(&playback_plan);
      (void)fpga_link_safe_stop(&fpga_link);
      block_parser_cancel(&block_parser);
      frame_ring_init(&frame_ring);
      block_stream_active = 0u;
      demo_building = 1u;
      rc = us_calibration_measure_profile(&fpga_link,
                                          (uint8_t)umh_bench_arg[0],
                                          (uint8_t)umh_bench_arg[1],
                                          (uint8_t)umh_bench_arg[2],
                                          (uint8_t)umh_bench_arg[3],
                                          (uint16_t)umh_bench_arg[4],
                                          (uint16_t)umh_bench_arg[5],
                                          (uint8_t)umh_bench_arg[6],
                                          umh_bench_arg[7],
                                          (uint8_t)umh_bench_arg[8]);
      (void)fpga_link_safe_stop(&fpga_link);
      demo_building = 0u;
      break;
    case UMH_BENCH_CMD_PATTERN:
      playback_plan_stop(&playback_plan);
      (void)fpga_link_safe_stop(&fpga_link);
      block_parser_cancel(&block_parser);
      frame_ring_init(&frame_ring);
      block_stream_active = 0u;
      demo_building = 1u;
      rc = us_calibration_measure_pattern(&fpga_link,
                                          (const uint8_t *)umh_bench_phase,
                                          (const uint8_t *)umh_bench_level,
                                          (uint16_t)umh_bench_arg[0],
                                          (uint8_t)umh_bench_arg[1],
                                          umh_bench_arg[2],
                                          (uint8_t)umh_bench_arg[3],
                                          (float *)umh_bench_frame_i,
                                          (float *)umh_bench_frame_q);
      (void)fpga_link_safe_stop(&fpga_link);
      demo_building = 0u;
      break;
    case UMH_BENCH_CMD_PROFILE_ALL:
      playback_plan_stop(&playback_plan);
      (void)fpga_link_safe_stop(&fpga_link);
      block_parser_cancel(&block_parser);
      frame_ring_init(&frame_ring);
      block_stream_active = 0u;
      demo_building = 1u;
      rc = us_calibration_measure_profile_all(&fpga_link,
                                              (uint8_t)umh_bench_arg[0],
                                              umh_bench_arg[1],
                                              (uint8_t)umh_bench_arg[2],
                                              (uint16_t)umh_bench_arg[3],
                                              (uint16_t)umh_bench_arg[4],
                                              (uint8_t)umh_bench_arg[5],
                                              (uint8_t)umh_bench_arg[6],
                                              (uint8_t)umh_bench_arg[7],
                                              &umh_bench_peaks[0][0]);
      (void)fpga_link_safe_stop(&fpga_link);
      demo_building = 0u;
      break;
    case UMH_BENCH_CMD_APPLY_PHASE:
      for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i)
        renderer.calibration[i].phase = umh_bench_phase[i];
      break;
    case UMH_BENCH_CMD_START_DEMO:
      device_gui.selected_demo = (uint8_t)umh_bench_arg[0];
      rc = gui_demo(NULL);
      break;
    case UMH_BENCH_CMD_COMMIT:
      {
        eeprom_profile_record_t record = eeprom_profile.record;
        record.version = EEPROM_PROFILE_VERSION;
        for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) {
          record.phase[i] = (uint16_t)((uint16_t)renderer.calibration[i].phase << 8);
          record.enabled[i / 8u] |= (uint8_t)(1u << (i % 8u));
          renderer.calibration[i].enabled = 1u;
        }
        record.cal_meta_valid = 1u;
        record.cal_level = (uint16_t)umh_bench_arg[0];
        record.cal_time_ms = (uint32_t)(system_time_us() / 1000u);
        if (eeprom_profile_commit(&eeprom_profile, &record) == 0)
          apply_calibration_from_eeprom();
        else
          rc = -1;
      }
      break;
    default:
      rc = -2;
      break;
  }
  umh_bench_result = (uint32_t)rc;
  umh_bench_status = (rc == 0) ? 2u : 3u;
}

static void calibration_task(void *argument)
{
  (void)argument;
  for (;;) {
    /* Only an explicit GUI/bench notification may start an acoustic run.
     * A timeout merely services the debugger mailbox.  Treating the timeout
     * as a calibration request made every boot start an unattended sweep,
     * which emitted ultrasound without any user action and starved the
     * lower-priority UI task (OLED stayed blank/keys were ignored). */
    BaseType_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
    if (umh_bench_cmd != UMH_BENCH_CMD_NONE) {
      bench_service();
      if (notified == 0) continue;
    }
    if (notified == 0) continue;
    if (calibration_selftest_mode != 0u) {
      calibration_selftest_mode = 0u;
      calibration_selftest_session();
    } else if (calibration_raw_mode != 0u) {
      calibration_raw_mode = 0u;
      calibration_raw_session();
    } else {
      calibration_session();
    }
  }
}

static int gui_demo(void *context)
{
  const umh_demo_descriptor_t *descriptor;
  int result;
  (void)context;
  descriptor = demo_engine_descriptor(device_gui.selected_demo);
  if (descriptor == NULL) return -1;
  demo_building = 1u;
  if (audio_engine_owns_output(&audio_engine) != 0u) audio_engine_abort(&audio_engine);
  if (motion_engine_owns_output(&motion_engine) != 0u) motion_engine_abort(&motion_engine, &fpga_link);
  playback_plan_stop(&playback_plan);
  (void)fpga_link_safe_stop(&fpga_link);
  block_parser_cancel(&block_parser);
  block_stream_active = 0u;
  result = demo_engine_build(device_gui.selected_demo, &renderer, &frame_ring,
                             system_time_us(), &playback_plan);
  demo_building = 0u;
  return result == 0 ? 0 : -1;
}

static int gui_ws2812_set(void *context)
{
  uint8_t r = 0u, g = 0u, b = 0u;
  int result;
  (void)context;

  switch (device_gui.ws2812_mode) {
    case 0u: /* OFF */
      r = 0u; g = 0u; b = 0u;
      break;
    case 1u: /* RED */
      r = 255u; g = 0u; b = 0u;
      break;
    case 2u: /* GREEN */
      r = 0u; g = 255u; b = 0u;
      break;
    case 3u: /* BLUE */
      r = 0u; g = 0u; b = 255u;
      break;
    case 4u: /* WHITE */
      r = 255u; g = 255u; b = 255u;
      break;
    default:
      r = 0u; g = 0u; b = 0u;
      break;
  }

  result = fpga_link_set_ws2812(&fpga_link, r, g, b);

  /* Log error details to system status for debugging */
  if (result != 0) {
    system_status_fault(UMH_FAULT_FPGA_OUTPUT, (uint32_t)(-result), UMH_FAULT_WARNING);
  }

  return result;
}

static void send_result(const umh_protocol_frame_t *request, umh_status_t status,
                        const void *payload, uint16_t length)
{
  uint8_t status_payload[4];
  status_payload[0] = (uint8_t)status;
  status_payload[1] = (uint8_t)frame_ring_count(&frame_ring);
  status_payload[2] = (uint8_t)frame_ring_free(&frame_ring);
  status_payload[3] = 0u;
  if (status == UMH_STATUS_OK && payload != NULL && length != 0u) {
    send_response(request, UMH_MSG_ACK, payload, length);
  } else {
    send_response(request, status == UMH_STATUS_OK ? UMH_MSG_ACK : UMH_MSG_NACK,
                  status_payload, sizeof(status_payload));
  }
  if (status != UMH_STATUS_OK) {
    system_status_get()->protocol_errors++;
    system_status_fault(fault_code_for_status(status,
                                               request != NULL ? request->header.message_type : 0u),
                         status,
                         status == UMH_STATUS_IO ? UMH_FAULT_CRITICAL : UMH_FAULT_WARNING);
  }
}

static void send_response(const umh_protocol_frame_t *request, uint8_t type,
                          const void *payload, uint16_t length)
{
  uint8_t output[UMH_PROTOCOL_HEADER_SIZE + UMH_PROTOCOL_MAX_PAYLOAD];
  uint16_t encoded;
  uint8_t flags = UMH_FLAG_RESPONSE;
  if (type == UMH_MSG_NACK) flags |= UMH_FLAG_ERROR;
  encoded = umh_protocol_encode(type, flags,
                                 request != NULL ? request->header.transaction_id : 0u,
                                 request != NULL ? request->header.stream_sequence : 0u,
                                  (const uint8_t *)payload, length, output, sizeof(output));
  if (encoded == 0u || umh_usb_tx_enqueue(output, encoded) == 0u) {
    system_status_get()->usb_dropped++;
    system_status_fault(UMH_FAULT_USB_TX_DROP, system_status_get()->usb_dropped, UMH_FAULT_WARNING);
  }
}

static int queue_storage_request(const umh_protocol_frame_t *frame)
{
  storage_request_t request;
  if (storage_queue == NULL || frame == NULL) return -1;
  memcpy(&request.frame, frame, sizeof(request.frame));
  return osMessageQueuePut(storage_queue, &request, 0u, 0u) == osOK ? 0 : -1;
}

static void storage_process(const storage_request_t *request)
{
  const umh_protocol_frame_t *frame = &request->frame;
  uint32_t id;
  uint32_t length;
  uint16_t count;
  umh_status_t status = UMH_STATUS_OK;
  if (frame->header.message_type == UMH_MSG_FLASH_LIST) {
    if (frame->payload_size != 0u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
    count = flash_store_list(&flash_store, storage_records, FLASH_STORE_MAX_OBJECTS);
    if ((uint32_t)count * sizeof(storage_records[0]) > sizeof(storage_response)) { send_result(frame, UMH_STATUS_NO_MEMORY, NULL, 0u); return; }
    memcpy(storage_response, storage_records, (uint32_t)count * sizeof(storage_records[0]));
    send_response(frame, UMH_MSG_FLASH_LIST, storage_response, (uint16_t)(count * sizeof(storage_records[0])));
    return;
  }
  if (frame->payload_size < 4u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
  id = read_u32(frame->payload);
  if (frame->header.message_type == UMH_MSG_FLASH_READ) {
    uint32_t offset;
    uint32_t requested;
    uint32_t total_length;
    if (frame->payload_size != 10u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
    offset = read_u32(&frame->payload[4]);
    requested = (uint32_t)frame->payload[8] | ((uint32_t)frame->payload[9] << 8);
    if (requested == 0u || requested > sizeof(storage_data) - 12u) requested = sizeof(storage_data) - 12u;
    if (flash_store_read_range(&flash_store, id, offset, storage_data, requested,
                               &length, &total_length) != 0 || length > sizeof(storage_response) - 12u) {
      send_result(frame, UMH_STATUS_IO, NULL, 0u); return;
    }
    storage_response[0] = (uint8_t)id; storage_response[1] = (uint8_t)(id >> 8); storage_response[2] = (uint8_t)(id >> 16); storage_response[3] = (uint8_t)(id >> 24);
    storage_response[4] = (uint8_t)total_length; storage_response[5] = (uint8_t)(total_length >> 8); storage_response[6] = (uint8_t)(total_length >> 16); storage_response[7] = (uint8_t)(total_length >> 24);
    storage_response[8] = (uint8_t)offset; storage_response[9] = (uint8_t)(offset >> 8); storage_response[10] = (uint8_t)(offset >> 16); storage_response[11] = (uint8_t)(offset >> 24);
    memcpy(&storage_response[12], storage_data, length);
    send_response(frame, UMH_MSG_FLASH_READ, storage_response, (uint16_t)(length + 12u));
    return;
  }
  if (frame->header.message_type == UMH_MSG_FLASH_WRITE) {
    if (frame->payload_size < 9u) status = UMH_STATUS_BAD_LENGTH;
    else if (flash_store_write(&flash_store, id,
                               (uint16_t)frame->payload[4] | ((uint16_t)frame->payload[5] << 8),
                               (uint16_t)frame->payload[6] | ((uint16_t)frame->payload[7] << 8),
                               &frame->payload[8], frame->payload_size - 8u) != 0) status = UMH_STATUS_IO;
  } else if (frame->header.message_type == UMH_MSG_FLASH_DELETE) {
    if (frame->payload_size != 4u || flash_store_delete(&flash_store, id) != 0) status = UMH_STATUS_IO;
  } else status = UMH_STATUS_UNSUPPORTED;
  send_result(frame, status, NULL, 0u);
}

static void protocol_frame_received(const umh_protocol_frame_t *frame, void *context)
{
  umh_status_t status = UMH_STATUS_OK;
  uint8_t status_payload[8];
  uint32_t value;
  uint8_t audio_owns;
  uint8_t motion_owns;
  (void)context;
  if (frame == NULL) return;
  audio_owns = audio_engine_owns_output(&audio_engine);
  motion_owns = motion_engine_owns_output(&motion_engine);
  switch (frame->header.message_type) {
    case UMH_MSG_GET_PROFILE:
      if (frame->payload_size != 0u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
      send_response(frame, UMH_MSG_PROFILE, &device_profile, (uint16_t)sizeof(device_profile));
      return;
    case UMH_MSG_GET_STATUS:
      if (frame->payload_size != 0u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
      value = system_status_get()->flags;
      memcpy(status_payload, &value, sizeof(value));
      value = ((uint32_t)frame_ring_count(&frame_ring) << 16) | frame_ring_free(&frame_ring);
      memcpy(&status_payload[4], &value, sizeof(value));
      send_response(frame, UMH_MSG_STATUS, status_payload, sizeof(status_payload));
      return;
    case UMH_MSG_BLOCK_BEGIN:
      if (audio_owns != 0u || motion_owns != 0u) { status = UMH_STATUS_BUSY; break; }
      if (playback_plan.running != 0u || block_stream_active != 0u ||
          frame_ring_count(&frame_ring) != 0u ||
          block_parser_begin(&block_parser, frame->payload, frame->payload_size) != 0) {
        block_parser_cancel(&block_parser);
        frame_ring_init(&frame_ring);
        status = playback_plan.running != 0u ? UMH_STATUS_BUSY : UMH_STATUS_INVALID_STATE;
      } else {
        block_stream_active = 1u;
        block_stream_expected_sequence = frame->header.stream_sequence + 1u;
      }
      break;
    case UMH_MSG_BLOCK_DATA:
      if (block_stream_active == 0u ||
          frame->header.stream_sequence != block_stream_expected_sequence ||
          block_parser_data(&block_parser, frame->payload, frame->payload_size) != 0) {
        block_parser_cancel(&block_parser);
        frame_ring_init(&frame_ring);
        block_stream_active = 0u;
        status = frame->header.stream_sequence != block_stream_expected_sequence ?
                 UMH_STATUS_BAD_SEQUENCE : UMH_STATUS_BAD_LENGTH;
      } else {
        ++block_stream_expected_sequence;
      }
      break;
    case UMH_MSG_BLOCK_END:
      if (block_stream_active == 0u ||
          frame->header.stream_sequence != block_stream_expected_sequence ||
          block_parser_end(&block_parser) != 0) {
        block_parser_cancel(&block_parser);
        frame_ring_init(&frame_ring);
        block_stream_active = 0u;
        status = frame->header.stream_sequence != block_stream_expected_sequence ?
                 UMH_STATUS_BAD_SEQUENCE : UMH_STATUS_INVALID_STATE;
      } else {
        ++block_stream_expected_sequence;
        block_stream_active = 0u;
      }
      break;
    case UMH_MSG_BLOCK_CANCEL:
      if (frame->payload_size != 0u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
      block_parser_cancel(&block_parser);
      frame_ring_init(&frame_ring);
      block_stream_active = 0u;
      break;
    case UMH_MSG_SET_PLAN:
      if (audio_owns != 0u || motion_owns != 0u) { status = UMH_STATUS_BUSY; break; }
      if (playback_plan.running != 0u ||
          frame->payload_size != sizeof(umh_playback_plan_wire_t) ||
          playback_plan_set(&playback_plan, (const umh_playback_plan_wire_t *)frame->payload) != 0) status = UMH_STATUS_INVALID_STATE;
      break;
    case UMH_MSG_SET_DEMO:
      if (audio_owns != 0u || motion_owns != 0u) { status = UMH_STATUS_BUSY; break; }
      if (frame->payload_size != 1u || frame->payload[0] >= demo_engine_count()) {
        status = UMH_STATUS_BAD_LENGTH;
      } else {
        playback_plan_stop(&playback_plan);
        (void)fpga_link_safe_stop(&fpga_link);
        block_parser_cancel(&block_parser);
        block_stream_active = 0u;
        if (demo_engine_build(frame->payload[0], &renderer, &frame_ring,
                              system_time_us(), &playback_plan) != 0) status = UMH_STATUS_INVALID_STATE;
        else {
          const umh_demo_descriptor_t *descriptor = demo_engine_descriptor(frame->payload[0]);
          send_result(frame, UMH_STATUS_OK, descriptor->name,
                      (uint16_t)strlen(descriptor->name));
          return;
        }
      }
      break;
    case UMH_MSG_MOTION_UPLOAD:
      if (audio_owns != 0u) { status = UMH_STATUS_BUSY; break; }
      if (motion_engine_upload(&motion_engine, frame->payload, frame->payload_size) != 0)
        status = UMH_STATUS_BAD_LENGTH;
      break;
    case UMH_MSG_MOTION_CONFIG:
      if (audio_owns != 0u) { status = UMH_STATUS_BUSY; break; }
      if (motion_engine_configure(&motion_engine, frame->payload, frame->payload_size) != 0)
        status = UMH_STATUS_BAD_LENGTH;
      break;
    case UMH_MSG_MOTION_START:
      if (frame->payload_size != 0u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
      if (audio_owns != 0u || device_gui_calibration_busy(&device_gui) != 0u ||
          demo_building != 0u) { status = UMH_STATUS_BUSY; break; }
      motion_engine_abort(&motion_engine, NULL);
      playback_plan_stop(&playback_plan);
      block_parser_cancel(&block_parser);
      block_stream_active = 0u;
      frame_ring_init(&frame_ring);
      if (fpga_link_safe_stop(&fpga_link) != 0) status = UMH_STATUS_IO;
      else if (motion_engine_start(&motion_engine) != 0) status = UMH_STATUS_INVALID_STATE;
      break;
    case UMH_MSG_MOTION_TARGET:
      if (audio_owns != 0u) { status = UMH_STATUS_BUSY; break; }
      if (motion_engine_target(&motion_engine, frame->payload, frame->payload_size) != 0)
        status = UMH_STATUS_BAD_LENGTH;
      break;
    case UMH_MSG_MOTION_STOP:
      if (frame->payload_size != 0u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
      motion_engine_request_stop(&motion_engine);
      break;
    case UMH_MSG_MOTION_STATUS:
      if (frame->payload_size != 0u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
      {
        umh_motion_status_wire_t motion_status;
        motion_engine_get_status(&motion_engine, &motion_status);
        send_response(frame, UMH_MSG_MOTION_STATUS, &motion_status,
                      (uint16_t)sizeof(motion_status));
        return;
      }
    case UMH_MSG_START_PLAN:
      if (frame->payload_size != 0u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
      if (audio_owns != 0u || motion_owns != 0u) { status = UMH_STATUS_BUSY; break; }
      if (playback_plan.configured == 0u ||
          (block_parser.block.active == 0u && frame_ring_count(&frame_ring) == 0u) ||
          playback_plan.wire.block_id != block_parser.block.header.block_id ||
          (block_parser.block.active != 0u && playback_plan.wire.repeat_mode != UMH_PLAN_LOOP_STREAM) ||
          (playback_plan.wire.repeat_mode != UMH_PLAN_LOOP_STREAM && frame_ring_count(&frame_ring) == 0u)) {
        status = UMH_STATUS_INVALID_STATE;
        break;
      }
      if (playback_plan.wire.repeat_mode == UMH_PLAN_LOOP_RAM &&
          frame_ring_snapshot(&frame_ring, block_parser_duration(&block_parser)) != 0) {
        status = UMH_STATUS_INVALID_STATE;
        break;
      }
      if (playback_plan_start(&playback_plan, system_time_us(),
                              frame_ring_count(&frame_ring),
                              block_parser.block.header.start_time) != 0) status = UMH_STATUS_INVALID_STATE;
      break;
    case UMH_MSG_STOP_PLAN:
      if (frame->payload_size != 0u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
      if (audio_owns != 0u) audio_engine_abort(&audio_engine);
      if (motion_owns != 0u) motion_engine_request_stop(&motion_engine);
      playback_plan_stop(&playback_plan);
      block_parser_cancel(&block_parser);
      block_stream_active = 0u;
      frame_ring_init(&frame_ring);
      if (motion_owns == 0u) (void)fpga_link_safe_stop(&fpga_link);
      break;
    case UMH_MSG_CLEAR_PLAN:
      if (frame->payload_size != 0u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
      if (audio_owns != 0u) audio_engine_abort(&audio_engine);
      if (motion_owns != 0u) motion_engine_abort(&motion_engine, &fpga_link);
      playback_plan_stop(&playback_plan);
      (void)fpga_link_safe_stop(&fpga_link);
      playback_plan_clear(&playback_plan);
      block_parser_cancel(&block_parser);
      block_stream_active = 0u;
      frame_ring_init(&frame_ring);
      break;
    case UMH_MSG_FPGA_STATUS:
      if (frame->payload_size != 0u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
      send_response(frame, UMH_MSG_FPGA_STATUS, fpga_link_status(&fpga_link), sizeof(fpga_status_wire_t));
      return;
    case UMH_MSG_EEPROM_READ:
      if (frame->payload_size != 0u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
      send_response(frame, UMH_MSG_EEPROM_READ, eeprom_profile_current(&eeprom_profile), sizeof(eeprom_profile_record_t));
      return;
    case UMH_MSG_EEPROM_WRITE:
      if (frame->payload_size != sizeof(eeprom_profile_record_t)) status = UMH_STATUS_BAD_LENGTH;
      else {
        memcpy(&eeprom_pending, frame->payload, sizeof(eeprom_pending));
        eeprom_pending_valid = 1u;
      }
      break;
    case UMH_MSG_EEPROM_COMMIT:
      if (frame->payload_size != 0u || eeprom_pending_valid == 0u ||
          eeprom_profile_commit(&eeprom_profile, &eeprom_pending) != 0) status = UMH_STATUS_IO;
      else {
        eeprom_pending_valid = 0u;
        /* Make a host-written calibration record effective immediately; the
         * same hook is used by the on-device calibration task after it
         * commits a successful run. */
        apply_calibration_from_eeprom();
      }
      break;
    case UMH_MSG_CAL_RESULT:
      if (frame->payload_size != 0u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
      {
        umh_cal_result_wire_t wire;
        memset(&wire, 0, sizeof(wire));
        memcpy(wire.phase, calibration_result.phase_byte, UMH_DEVICE_CHANNEL_COUNT);
        wire.progress = calibration_result.progress;
        wire.good_mics = calibration_result.good_mics;
        wire.fault = calibration_result.fault;
        wire.quality_flags = calibration_result.quality_flags;
        wire.used_gate_count = calibration_result.used_gate_count;
        wire.used_gate_width = calibration_result.used_gate_width;
        wire.used_gate_start = calibration_result.used_gate_start;
        wire.block_count = calibration_result.patterns_used;
        wire.rms_before_deg = calibration_result.fit_rms_deg;
        wire.rms_after_deg = calibration_result.rms_after_deg;
        wire.mic_consistency_deg = calibration_result.mic_consistency_deg;
        wire.residual = calibration_result.residual;
        wire.distance_m = (float)calibration_result.patterns_used;
        wire.tilt_x_deg = (float)calibration_result.level_used;
        wire.tilt_y_deg = calibration_result.coupling_db;
        wire.echo_ratio = calibration_result.band_trend_deg;
        wire.mic_ratio = calibration_result.drift_deg;
        wire.verify_gain_db = calibration_result.verify_gain_db;
        send_response(frame, UMH_MSG_CAL_RESULT, &wire, (uint16_t)sizeof(wire));
        return;
      }
    case UMH_MSG_CAL_DUMP:
      if (frame->payload_size != 6u && frame->payload_size != 7u) {
        send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return;
      }
      {
        uint32_t offset = read_u32(&frame->payload[0]);
        uint16_t length = (uint16_t)((uint16_t)frame->payload[4] | ((uint16_t)frame->payload[5] << 8));
        uint8_t section = (frame->payload_size == 7u) ? frame->payload[6] : 0u;
        int copied;
        if (length > sizeof(calibration_dump_buffer)) length = sizeof(calibration_dump_buffer);
        copied = us_calibration_dump_read(section, offset, calibration_dump_buffer, length);
        if (copied < 0) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
        send_response(frame, UMH_MSG_CAL_DUMP, calibration_dump_buffer, (uint16_t)copied);
        return;
      }
    case UMH_MSG_CAL_RAW:
      if (frame->payload_size != 8u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
      if (audio_owns != 0u || motion_owns != 0u) { status = UMH_STATUS_BUSY; break; }
      if (calibration_raw_mode != 0u || device_gui_calibration_busy(&device_gui) != 0u) {
        status = UMH_STATUS_BUSY;
      } else if (frame->payload[0] == 0u || frame->payload[3] == 0u ||
                 frame->payload[3] > 64u) {
        status = UMH_STATUS_BAD_LENGTH;
      } else {
        calibration_raw_level = frame->payload[0];
        calibration_raw_gate_start = (uint16_t)((uint16_t)frame->payload[1] |
                                                ((uint16_t)frame->payload[2] << 8));
        calibration_raw_gate_width = frame->payload[3];
        calibration_raw_burst_us = (uint16_t)((uint16_t)frame->payload[4] |
                                              ((uint16_t)frame->payload[5] << 8));
        calibration_raw_patterns = (uint16_t)((uint16_t)frame->payload[6] |
                                              ((uint16_t)frame->payload[7] << 8));
        if (calibration_raw_patterns == 0u || calibration_raw_burst_us == 0u) {
          status = UMH_STATUS_BAD_LENGTH;
        } else {
          calibration_raw_mode = 1u;
          if (calibration_task_handle == NULL) status = UMH_STATUS_INVALID_STATE;
          else {
            (void)xTaskNotifyGive((TaskHandle_t)calibration_task_handle);
            status = UMH_STATUS_OK;
          }
        }
      }
      break;
    case UMH_MSG_CAL_START:
      if (frame->payload_size != 0u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
      if (audio_owns != 0u || motion_owns != 0u) { status = UMH_STATUS_BUSY; break; }
      if (calibration_task_handle == NULL) status = UMH_STATUS_INVALID_STATE;
      else if (calibration_raw_mode != 0u ||
               device_gui_calibration_busy(&device_gui) != 0u) status = UMH_STATUS_BUSY;
      else {
        device_gui_calibration_begin(&device_gui);
        (void)xTaskNotifyGive((TaskHandle_t)calibration_task_handle);
        status = UMH_STATUS_OK;
      }
      break;
    case UMH_MSG_CAL_SELFTEST:
      if (frame->payload_size != 0u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
      if (audio_owns != 0u || motion_owns != 0u) { status = UMH_STATUS_BUSY; break; }
      if (calibration_task_handle == NULL) status = UMH_STATUS_INVALID_STATE;
      else if (calibration_raw_mode != 0u || calibration_selftest_mode != 0u ||
               device_gui_calibration_busy(&device_gui) != 0u) status = UMH_STATUS_BUSY;
      else {
        device_gui_calibration_begin(&device_gui);
        calibration_selftest_mode = 1u;
        (void)xTaskNotifyGive((TaskHandle_t)calibration_task_handle);
        status = UMH_STATUS_OK;
      }
      break;
    case UMH_MSG_CAL_SELFTEST_RESULT:
      if (frame->payload_size != 0u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
      {
        umh_cal_self_test_wire_t wire;
        float coh = calibration_self_test_result.coherence * 1000.0f;
        if (coh > 65535.0f) coh = 65535.0f;
        if (coh < 0.0f) coh = 0.0f;
        memset(&wire, 0, sizeof(wire));
        wire.valid = (calibration_self_test_result.progress == 100u &&
                      calibration_self_test_result.channels_measured != 0u) ? 1u : 0u;
        wire.pass = calibration_self_test_result.pass;
        wire.good_mics = calibration_self_test_result.good_mics;
        wire.fault = calibration_self_test_result.fault;
        wire.channels_measured = calibration_self_test_result.channels_measured;
        wire.used_gate_start = calibration_self_test_result.used_gate_start;
        wire.used_level = calibration_self_test_result.used_level;
        wire.used_gate_width = calibration_self_test_result.used_gate_width;
        wire.focus_gain_db_x10 = (int16_t)lroundf(calibration_self_test_result.focus_gain_db * 10.0f);
        wire.coherence_x1000 = (uint16_t)lroundf(coh);
        wire.predicted_gain_db_x10 = (int16_t)lroundf(calibration_self_test_result.predicted_gain_db * 10.0f);
        wire.actual_vs_predicted_db_x10 = (int16_t)lroundf(calibration_self_test_result.actual_vs_predicted_db * 10.0f);
        for (uint8_t m = 0u; m < UMH_DEVICE_MIC_COUNT; ++m) {
          float mc = calibration_self_test_result.per_mic_coherence[m] * 1000.0f;
          if (mc > 65535.0f) mc = 65535.0f;
          if (mc < 0.0f) mc = 0.0f;
          wire.per_mic_gain_db_x10[m] = (int16_t)lroundf(calibration_self_test_result.per_mic_gain_db[m] * 10.0f);
          wire.per_mic_coherence_x1000[m] = (uint16_t)lroundf(mc);
        }
        send_response(frame, UMH_MSG_CAL_SELFTEST_RESULT, &wire, (uint16_t)sizeof(wire));
        return;
      }
    case UMH_MSG_ERROR_COUNTERS:
      if (frame->payload_size != 0u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
      {
        uint32_t counters[5];
        counters[0] = system_status_get()->protocol_errors;
        counters[1] = system_status_get()->parser_errors;
        counters[2] = system_status_get()->fpga_errors;
        counters[3] = system_status_get()->underruns;
        counters[4] = system_status_get()->usb_dropped;
        send_response(frame, UMH_MSG_ERROR_COUNTERS, counters, sizeof(counters));
        return;
      }
    case UMH_MSG_FLASH_READ:
    case UMH_MSG_FLASH_WRITE:
    case UMH_MSG_FLASH_DELETE:
    case UMH_MSG_FLASH_LIST:
      if (queue_storage_request(frame) == 0) return;
      status = UMH_STATUS_BUSY;
      break;
    case UMH_MSG_AUDIO_CONFIGURE:
      if (frame->payload_size < sizeof(umh_audio_config_wire_t) ||
          frame->payload_size > (uint16_t)(sizeof(umh_audio_config_wire_t) + 1u +
                                           UMH_AUDIO_MAX_EXTRA_POINTS * sizeof(umh_audio_point_wire_t))) {
        status = UMH_STATUS_BAD_LENGTH;
        break;
      }
      if (device_gui_calibration_busy(&device_gui) != 0u || demo_building != 0u ||
          motion_owns != 0u) {
        status = UMH_STATUS_BUSY;
        break;
      }
      if (playback_plan.running != 0u || block_stream_active != 0u ||
          block_parser.block.active != 0u || frame_ring_count(&frame_ring) != 0u) {
        status = UMH_STATUS_BUSY;
        break;
      }
      if (audio_engine_owns_output(&audio_engine) != 0u)
        audio_engine_abort(&audio_engine);
      if (audio_engine_configure(&audio_engine, &renderer, &fpga_link,
                                 frame->payload, frame->payload_size) != 0)
        status = UMH_STATUS_INVALID_STATE;
      break;
    case UMH_MSG_AUDIO_START:
      if (frame->payload_size != 0u) { status = UMH_STATUS_BAD_LENGTH; break; }
      if (audio_engine_start(&audio_engine) != 0) status = UMH_STATUS_INVALID_STATE;
      break;
    case UMH_MSG_AUDIO_DATA:
      if (frame->payload_size == 0u) return;
      (void)audio_engine_feed(&audio_engine, frame->payload, frame->payload_size,
                              frame->header.stream_sequence);
      return;
    case UMH_MSG_AUDIO_STOP:
      if (frame->payload_size != 0u) { status = UMH_STATUS_BAD_LENGTH; break; }
      audio_engine_request_stop(&audio_engine);
      system_status_clear(UMH_SYSTEM_UNDERRUN);
      break;
    case UMH_MSG_AUDIO_STATUS:
      if (frame->payload_size != 0u) { status = UMH_STATUS_BAD_LENGTH; break; }
      {
        umh_audio_status_wire_t audio_status;
        audio_engine_get_status(&audio_engine, &audio_status);
        send_response(frame, UMH_MSG_AUDIO_STATUS, &audio_status,
                      (uint16_t)sizeof(audio_status));
        return;
      }
    default:
      status = UMH_STATUS_UNSUPPORTED;
      break;
  }
  send_result(frame, status, NULL, 0u);
}

static void protocol_task(void *argument)
{
  uint8_t data[256];
  uint32_t count;
  uint32_t previous_parser_errors = 0u;
  uint32_t previous_rx_dropped = 0u;
  uint32_t previous_frame_dropped = 0u;
  (void)argument;
  /* Peripheral startup uses mutexes, DMA interrupts and the HAL tick. */
  application_init();
  MX_USB_Device_Init();
  system_status_set(UMH_SYSTEM_CONNECTED);
  for (;;) {
    count = umh_rx_ring_read(&umh_usb_rx_ring, data, sizeof(data));
    if (count != 0u) umh_protocol_parser_feed(&protocol_parser, data, count);
    else (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10u));
    system_status_get()->parser_errors = umh_protocol_parser_error_count(&protocol_parser);
    system_status_get()->rx_dropped = umh_usb_rx_ring.dropped_bytes;
    system_status_get()->frame_dropped = frame_ring.dropped;
    if (system_status_get()->frame_dropped != previous_frame_dropped) {
      previous_frame_dropped = system_status_get()->frame_dropped;
      system_status_fault(UMH_FAULT_FRAME_RING, previous_frame_dropped, UMH_FAULT_CRITICAL);
    }
    if (system_status_get()->rx_dropped != previous_rx_dropped) {
      previous_rx_dropped = system_status_get()->rx_dropped;
      system_status_fault(UMH_FAULT_USB_RX_DROP, previous_rx_dropped, UMH_FAULT_WARNING);
    }
    if (system_status_get()->parser_errors != previous_parser_errors) {
      previous_parser_errors = system_status_get()->parser_errors;
      system_status_fault(UMH_FAULT_PROTOCOL_PARSE, previous_parser_errors, UMH_FAULT_WARNING);
    }
    umh_usb_tx_service();
  }
}

#define RENDER_TIMER_CLOCK_HZ 1000000u
#define RENDER_TIMER_MAX_WAIT_US 1500u

static void render_timer_init(void)
{
  uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();
  /* On STM32G4 an APB1 prescaler > 1 doubles the timer kernel clock. */
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0u) timer_clock *= 2u;
  if (timer_clock < RENDER_TIMER_CLOCK_HZ) {
    system_status_fault(UMH_FAULT_HAL_INIT, 2u, UMH_FAULT_CRITICAL);
    return;
  }
  __HAL_RCC_TIM7_CLK_ENABLE();
  render_timer.Instance = TIM7;
  render_timer.Init.Prescaler = (uint16_t)(timer_clock / RENDER_TIMER_CLOCK_HZ - 1u);
  render_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
  render_timer.Init.Period = 0xFFFFu;
  render_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  render_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&render_timer) != HAL_OK) {
    system_status_fault(UMH_FAULT_HAL_INIT, 3u, UMH_FAULT_CRITICAL);
    return;
  }
  render_timer.Instance->CR1 |= TIM_CR1_OPM | TIM_CR1_URS;
  HAL_NVIC_SetPriority(TIM7_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0u);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
  render_timer_ready = 1u;
}

/* Arm a one-shot microsecond delay.  Keep the service routine minimal: it only
 * wakes the render task, which is then responsible for time re-evaluation. */
static void render_timer_arm_us(uint32_t delay_us)
{
  if (render_timer_ready == 0u || delay_us == 0u) return;
  if (delay_us > 0xFFFFu) delay_us = 0xFFFFu;
  TIM7->CR1 &= ~TIM_CR1_CEN;
  TIM7->DIER &= ~TIM_DIER_UIE;
  TIM7->CNT = 0u;
  TIM7->ARR = delay_us - 1u;
  TIM7->EGR = TIM_EGR_UG;
  TIM7->SR = 0u;
  TIM7->DIER |= TIM_DIER_UIE;
  TIM7->CR1 |= TIM_CR1_CEN;
}

static void render_wait_until_us(uint64_t target_us)
{
  for (;;) {
    uint64_t now = system_time_us();
    uint64_t delta;
    if (target_us <= now) return;
    delta = target_us - now;
    /* vTaskDelay has 1 ms granularity.  Use the hardware timer for the final
     * sub-millisecond span so a 200 Hz trajectory is not quantised to the
     * FreeRTOS tick. */
    if (render_timer_ready == 0u || delta > RENDER_TIMER_MAX_WAIT_US) {
      /* Long inter-frame gaps are legitimate (HOLD/ONCE and slow host plans),
       * but the FPGA must also see periodic link traffic.  Poll status every
       * millisecond while waiting; if the MCU dies the FPGA watchdog will
       * still clear the last frame. */
      if (delta > 5000u) (void)fpga_link_poll_status(&fpga_link);
      osDelay(1u);
      continue;
    }
    render_timer_arm_us((uint32_t)delta);
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2u));
  }
}

void TIM7_IRQHandler(void)
{
  if ((TIM7->SR & TIM_SR_UIF) != 0u) {
    TIM7->SR = 0u;
    if (render_task_handle != NULL) {
      BaseType_t higher_priority_task_woken = pdFALSE;
      vTaskNotifyGiveFromISR(render_task_handle, &higher_priority_task_woken);
      portYIELD_FROM_ISR(higher_priority_task_woken);
    }
  }
}

static void render_task(void *argument)
{
  umh_output_frame_t *frame;
  uint64_t last_time_us = system_time_us();
  uint64_t now_us;
  uint64_t elapsed_us;
  uint16_t previous_fpga_flags = 0u;
  uint8_t render_burst = 0u;
  (void)argument;
  render_task_handle = xTaskGetCurrentTaskHandle();
  for (;;) {
    if (demo_building != 0u) {
      render_burst = 0u;
      osDelay(1u);
      continue;
    }
    if (audio_engine_owns_output(&audio_engine) != 0u) {
      uint64_t audio_deadline_us = 0u;
      uint8_t have_audio_deadline = audio_engine_next_deadline(&audio_engine, &audio_deadline_us);
      now_us = system_time_us();
      if (have_audio_deadline != 0u && now_us < audio_deadline_us) {
        render_wait_until_us(audio_deadline_us);
        continue;
      }
      audio_engine_service(&audio_engine, now_us);
      {
        umh_audio_status_wire_t audio_status;
        audio_engine_get_status(&audio_engine, &audio_status);
        if (audio_status.state == (uint8_t)UMH_AUDIO_RUNNING)
          system_status_set(UMH_SYSTEM_PLAYING);
        else
          system_status_clear(UMH_SYSTEM_PLAYING);
        if ((audio_status.flags & UMH_AUDIO_STATUS_UNDERRUN) != 0u)
          system_status_set(UMH_SYSTEM_UNDERRUN);
        else
          system_status_clear(UMH_SYSTEM_UNDERRUN);
        system_status_get()->device_time = (uint32_t)now_us;
        system_status_get()->frame_count = 0u;
        system_status_get()->frame_free = 0u;
      }
      if (have_audio_deadline == 0u) osDelay(1u);
      last_time_us = now_us;
      render_burst = 0u;
      continue;
    }
    if (motion_engine_owns_output(&motion_engine) != 0u) {
      uint32_t motion_wait_us = 1000u;
      now_us = system_time_us();
      (void)motion_engine_service(&motion_engine, &renderer, &fpga_link, now_us,
                                  &motion_wait_us);
      if (motion_engine_owns_output(&motion_engine) != 0u) {
        system_status_set(UMH_SYSTEM_PLAYING);
        system_status_clear(UMH_SYSTEM_UNDERRUN);
      } else {
        system_status_clear(UMH_SYSTEM_PLAYING);
      }
      system_status_get()->device_time = (uint32_t)now_us;
      system_status_get()->frame_count = 0u;
      system_status_get()->frame_free = 0u;
      system_status_get()->fpga_credit = fpga_link_status(&fpga_link)->fifo_credit;
      system_status_get()->fpga_depth = fpga_link_status(&fpga_link)->fifo_depth;
      if (motion_wait_us == 0u) motion_wait_us = 250u;
      if (motion_wait_us > 50000u) motion_wait_us = 50000u;
      render_wait_until_us(system_time_us() + (uint64_t)motion_wait_us);
      last_time_us = system_time_us();
      render_burst = 0u;
      continue;
    }
    now_us = system_time_us();
    elapsed_us = now_us - last_time_us;
    if (elapsed_us > UINT32_MAX) elapsed_us = UINT32_MAX;
    playback_plan_tick(&playback_plan, now_us, (uint32_t)elapsed_us,
                       frame_ring_count(&frame_ring));
    last_time_us = now_us;
    frame = frame_ring_peek_read(&frame_ring);
    if (frame != NULL) {
      uint8_t due = playback_plan_frame_due(&playback_plan, frame->deadline, now_us);
      if (due != 0u) {
        if (fpga_link_status(&fpga_link)->fifo_credit != 0u) {
          int submit_result = fpga_link_submit(&fpga_link, frame);
          if (submit_result == 0) {
            (void)frame_ring_release_read(&frame_ring);
            playback_plan_frame_submitted(&playback_plan);
            /* A valid plan that is already behind can make several frames due
             * at once.  Bound the catch-up burst so a lower-priority I2C/UI
             * transfer is never starved for its full HAL timeout. */
            if (++render_burst >= 16u) {
              render_burst = 0u;
              osDelay(1u);
            }
            system_status_get()->frame_count = frame_ring_count(&frame_ring);
            system_status_get()->frame_free = frame_ring_free(&frame_ring);
            if (frame_ring_count(&frame_ring) == 0u) {
              int loop_result = playback_plan_prepare_loop(&playback_plan, &frame_ring, now_us);
              if (loop_result == 0 && playback_plan.running != 0u &&
                  playback_plan.wire.repeat_mode != UMH_PLAN_LOOP_STREAM) {
                playback_plan_stop(&playback_plan);
                /* ONCE and HOLD_LAST intentionally leave the last committed
                 * device state active. Only an explicit STOP plan disables output. */
                if (playback_plan.wire.repeat_mode == UMH_PLAN_STOP)
                  (void)fpga_link_safe_stop(&fpga_link);
              }
            }
          } else {
            ++system_status_get()->fpga_errors;
            system_status_fault(UMH_FAULT_FPGA_OUTPUT, (uint32_t)(-submit_result), UMH_FAULT_CRITICAL);
            osDelay(1u);
          }
        } else {
          /* link->status.fifo_credit is sampled from MISO at the start of the
           * previous SPI transaction, so it is one transaction stale.  Poll
           * the real credit and retry without dropping into a full 1 ms RTOS
           * tick: at a 200 Hz trajectory each extra tick would destroy the
           * frame cadence. */
          (void)fpga_link_poll_status(&fpga_link);
          continue;
        }
      } else {
        uint64_t due_time_us = 0u;
        uint8_t have_due_time = playback_plan_frame_output_time(&playback_plan,
                                                                frame->deadline,
                                                                &due_time_us);
        if (have_due_time != 0u && due_time_us > now_us) {
          render_wait_until_us(due_time_us);
          render_burst = 0u;
          continue;
        }
        (void)fpga_link_poll_status(&fpga_link);
        render_burst = 0u;
        osDelay(1u);
      }
    } else {
      render_burst = 0u;
      (void)fpga_link_poll_status(&fpga_link);
      osDelay(1u);
    }
    if (playback_plan.running != 0u) system_status_set(UMH_SYSTEM_PLAYING);
    else system_status_clear(UMH_SYSTEM_PLAYING);
    if (playback_plan.underrun_reported != 0u) {
      system_status_set(UMH_SYSTEM_UNDERRUN);
      system_status_get()->underruns++;
      system_status_fault(UMH_FAULT_FPGA_UNDERRUN, system_status_get()->underruns, UMH_FAULT_WARNING);
      playback_plan.underrun_reported = 0u;
      if (playback_plan.running == 0u && playback_plan.wire.underrun_policy == UMH_UNDERRUN_DISABLE)
        (void)fpga_link_safe_stop(&fpga_link);
    }
    {
      uint16_t fpga_flags = fpga_link_status(&fpga_link)->status_flags;
      /* Bit 4 is the normal RUNNING indication.  Only bits 0..3 are
       * fault conditions; treating any nonzero word as an error made a
       * healthy running FPGA appear as "FPGA UNDERRU, ARG=16". */
      uint16_t fpga_fault_flags = (uint16_t)(fpga_flags &
                                             (FPGA_STATUS_UNDERRUN |
                                              FPGA_STATUS_OVERFLOW |
                                              FPGA_STATUS_INVALID_FRAME |
                                              FPGA_STATUS_OUTPUT_FAULT));
      if (fpga_fault_flags != 0u) system_status_set(UMH_SYSTEM_ERROR);
      if (fpga_fault_flags != 0u && fpga_fault_flags != previous_fpga_flags) {
        umh_fault_code_t code = (fpga_fault_flags & FPGA_STATUS_OUTPUT_FAULT) != 0u ? UMH_FAULT_FPGA_OUTPUT :
                                 (fpga_fault_flags & FPGA_STATUS_INVALID_FRAME) != 0u ? UMH_FAULT_FPGA_INVALID_FRAME :
                                 (fpga_fault_flags & FPGA_STATUS_OVERFLOW) != 0u ? UMH_FAULT_FPGA_OVERFLOW :
                                 UMH_FAULT_FPGA_UNDERRUN;
        system_status_fault(code, fpga_fault_flags, UMH_FAULT_CRITICAL);
      }
      previous_fpga_flags = fpga_fault_flags;
    }
    system_status_get()->fpga_credit = fpga_link_status(&fpga_link)->fifo_credit;
    system_status_get()->fpga_depth = fpga_link_status(&fpga_link)->fifo_depth;
    system_status_get()->frame_count = frame_ring_count(&frame_ring);
    system_status_get()->frame_free = frame_ring_free(&frame_ring);
    system_status_get()->device_time = (uint32_t)now_us;
    system_status_get()->frame_dropped = frame_ring.dropped;
  }
}

static void storage_task(void *argument)
{
  (void)argument;
  for (;;) {
    if (osMessageQueueGet(storage_queue, &storage_request_buffer, NULL, osWaitForever) == osOK) storage_process(&storage_request_buffer);
  }
}

static void input_task(void *argument)
{
  TickType_t last_wake = xTaskGetTickCount();
  (void)argument;
  for (;;) {
    input_events_sample();
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10u));
  }
}

static void ui_task(void *argument)
{
  input_event_t event;
  uint32_t last_oled_attempt = HAL_GetTick();
  (void)argument;
  for (;;) {
    while (input_events_poll(&event) != 0u) {
      device_gui_handle_event(&device_gui, &event);
    }
    if (oled.initialized == 0u && (HAL_GetTick() - last_oled_attempt) >= 1000u) {
      last_oled_attempt = HAL_GetTick();
      oled_ssd1315_init(&oled, &hi2c1);
    }
    device_gui_render(&device_gui, HAL_GetTick());
    if (oled.initialized != 0u && oled_ssd1315_refresh(&oled) != 0) {
      oled.initialized = 0u;
      last_oled_attempt = HAL_GetTick();
      /* A display/I2C fault must not disable the FPGA output path.  Keep it
       * visible in diagnostics while allowing the render task to continue. */
      system_status_fault(UMH_FAULT_HAL_INIT, 1u, UMH_FAULT_WARNING);
    }
    /* Keep the display and cursor animation responsive while input scanning
     * remains independent at 10 ms. */
    osDelay(30u);
  }
}

static void health_task(void *argument)
{
  (void)argument;
  for (;;) {
    HAL_GPIO_TogglePin(HEART_GPIO_Port, HEART_Pin);
    system_status_get()->heartbeat ^= 1u;
    system_status_get()->uptime_ms = HAL_GetTick();
    osDelay(250u);
  }
}

void MX_FREERTOS_Init(void)
{
  umh_protocol_task_handle = osThreadNew(protocol_task, NULL, &protocol_task_attributes);
}

static void application_init(void)
{
  umh_rx_ring_init(&umh_usb_rx_ring);
  device_profile_init(&device_profile);
  system_status_init();
  i2c_bus_init();
  frame_ring_init(&frame_ring);
  spatial_renderer_init(&renderer, &device_profile);
  block_parser_init(&block_parser, &renderer, &frame_ring);
  audio_engine_init(&audio_engine);
  motion_engine_init(&motion_engine);
  playback_plan_clear(&playback_plan);
  fpga_link_init(&fpga_link, &hspi1);
  /* A reboot is a new command epoch.  Always clear a possible output left
   * behind by the previous MCU life before accepting UI/demo commands. */
  (void)fpga_link_safe_stop(&fpga_link);
  /* Establish the FPGA link before accepting UI/demo commands.  Without an
   * initial status transaction the link retains its optimistic default credit
   * and a failed SPI slave is only discovered after the first output frame. */
  if (fpga_link_poll_status(&fpga_link) != 0) {
    system_status_fault(UMH_FAULT_FPGA_SPI_TIMEOUT, 1u, UMH_FAULT_CRITICAL);
  }
  flash_nor_init(&hspi3);
  flash_store_init(&flash_store);
  {
    uint8_t flash_id[3];
    if (flash_nor_read_jedec(flash_id) == FLASH_NOR_OK &&
        flash_nor_is_present() != 0u && flash_store_mount(&flash_store) == 0)
      system_status_set(UMH_SYSTEM_FLASH_READY);
    else
      system_status_fault(UMH_FAULT_FLASH_IO, 1u, UMH_FAULT_WARNING);
  }
  eeprom_profile_init(&eeprom_profile, &hi2c1);
  {
    int load_result = eeprom_profile_load(&eeprom_profile);
    if (load_result == 0) {
      apply_calibration_from_eeprom();
    } else if (load_result > 0 && eeprom_profile.present != 0u) {
      /* Blank but reachable EEPROM: defaults are not an I/O fault. */
      apply_calibration_from_eeprom();
      system_status_clear(UMH_SYSTEM_CALIBRATION_VALID);
    } else {
      system_status_fault(UMH_FAULT_EEPROM_IO, 1u, UMH_FAULT_WARNING);
    }
  }
  oled_ssd1315_init(&oled, &hi2c1);
  if (oled.initialized == 0u) system_status_fault(UMH_FAULT_HAL_INIT, 1u, UMH_FAULT_WARNING);
  input_events_init();
  device_gui_init(&device_gui, &oled, &device_profile, system_status_get(),
                  &playback_plan, &fpga_link, &flash_store, &eeprom_profile,
                  gui_calibration, gui_selftest, gui_demo, gui_ws2812_set,
                  demo_engine_count(), NULL);
  {
    const osMessageQueueAttr_t storage_queue_attributes = {
      .name = "storage", .cb_mem = &storage_queue_cb, .cb_size = sizeof(storage_queue_cb),
      .mq_mem = storage_queue_memory, .mq_size = sizeof(storage_queue_memory)
    };
    storage_queue = osMessageQueueNew(2u, sizeof(storage_request_t), &storage_queue_attributes);
  }
  umh_protocol_parser_init(&protocol_parser, protocol_frame_received, NULL);
  render_timer_init();
  (void)osThreadNew(render_task, NULL, &render_task_attributes);
  (void)osThreadNew(storage_task, NULL, &storage_task_attributes);
  (void)osThreadNew(ui_task, NULL, &ui_task_attributes);
  (void)osThreadNew(input_task, NULL, &input_task_attributes);
  (void)osThreadNew(health_task, NULL, &health_task_attributes);
  calibration_task_handle = osThreadNew(calibration_task, NULL, &calibration_task_attributes);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == fpga_link.spi) fpga_link_spi_txrx_complete(&fpga_link);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == fpga_link.spi) {
    fpga_link_spi_error(&fpga_link);
    system_status_get()->fpga_errors++;
    system_status_fault(UMH_FAULT_FPGA_SPI_DMA, HAL_SPI_GetError(hspi), UMH_FAULT_CRITICAL);
  }
}
