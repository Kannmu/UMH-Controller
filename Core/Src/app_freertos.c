/* v7 application tasks and static runtime objects. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
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
#include "flash_library.h"
#include "eeprom_profile.h"
#include "oled_ssd1315.h"
#include "input_events.h"
#include "system_status.h"
#include "task_stats.h"
#include "device_gui.h"
#include "us_calibration.h"
#include "demo_engine.h"
#include "motion_engine.h"
#include "aperture_mod.h"
#include "hologram_engine.h"
#include "gun_trigger.h"
#include "gun_voice_dsp.h"
#include "usb_audio.h"
#include "usb_audio_dsp.h"
#include "spi.h"
#include "i2c.h"
#include "i2c_bus.h"
#include "umh_utils.h"
#include <string.h>
#include <math.h>

osThreadId_t umh_protocol_task_handle;
static void abort_aperture_output(void);
umh_rx_ring_t umh_usb_rx_ring;

static umh_device_profile_t device_profile;
static umh_spatial_renderer_t renderer;
static umh_frame_ring_t frame_ring;
static umh_block_parser_t block_parser;
static umh_playback_plan_t playback_plan;
static umh_aperture_mod_t aperture_mod;
static usb_audio_dsp_t usb_audio_dsp;
static volatile uint8_t usb_audio_owns, usb_audio_inhibited;
static uint32_t usb_audio_epoch, usb_audio_blocked_epoch, usb_audio_sequence;
/* UAC envelope feed accumulator.  Feeding aperture_mod 20 samples every
 * millisecond put a 1 kHz component on the common level; the microphone
 * comparison showed the UAC-specific 1.2..1.3 kHz sideband (1 kHz + 261.6 Hz)
 * that the CDC path (256-sample frames at 78 Hz) does not have.  Accumulate
 * to 32 samples so the host feed rate is ~625 Hz instead of 1 kHz. */
static uint8_t usb_audio_env_block[256];
#define UAC_SERVICE_BLOCK 48u
static uint16_t usb_audio_env_count;
static umh_motion_engine_t motion_engine;
static umh_hologram_engine_t hologram_engine;
static umh_gun_trigger_t gun_trigger;
static fpga_link_t fpga_link;
static flash_store_t flash_store;
static flash_library_t flash_library;
static flash_library_reader_t flash_reader;
static flash_library_writer_t flash_writer;
static flash_library_ring_sink_t flash_ring_sink;
static volatile uint8_t flash_play_active;
static volatile uint8_t flash_play_stop_request;
static TaskHandle_t storage_task_handle;
static uint8_t flash_upload_active;
static uint32_t flash_upload_expected_sequence;
static uint32_t flash_upload_object_id;
static uint32_t flash_play_object_id;
static uint16_t flash_last_result;
#define UMH_FLASH_PLAY_PREBUFFER 8u
#define UMH_FLASH_PLAY_HIGH_WATER 16u
static eeprom_profile_t eeprom_profile;
static oled_ssd1315_t oled;
static umh_protocol_parser_t protocol_parser;
static device_gui_t device_gui;
static umh_calibration_result_t calibration_result;
static umh_cal_self_test_result_t calibration_self_test_result;
static osThreadId_t calibration_task_handle;
static volatile uint8_t calibration_raw_mode;
static volatile uint8_t calibration_probe_mode;
static volatile uint8_t calibration_selftest_mode;
static uint8_t calibration_raw_level;
static uint16_t calibration_raw_gate_start;
static uint8_t calibration_raw_gate_width;
static uint16_t calibration_raw_burst_us;
static uint16_t calibration_raw_patterns;
/* CAL_PROBE bench request (see us_calibration_probe()).  Filled by the
 * protocol task, consumed by the calibration task. */
typedef struct __attribute__((packed)) {
  uint8_t gate_count;
  uint8_t gate_width;
  uint16_t gate_start;
  uint16_t gate_step;
  uint16_t burst_us;
  uint16_t settle_us;
  uint8_t repeats;
  uint8_t flags;
  uint8_t reserved[4];
  uint8_t phase[UMH_DEVICE_CHANNEL_COUNT];
  uint8_t level[UMH_DEVICE_CHANNEL_COUNT];
} umh_cal_probe_request_t;
_Static_assert(sizeof(umh_cal_probe_request_t) == 184u, "cal probe request size");
static umh_cal_probe_request_t calibration_probe_request;
static uint32_t calibration_probe_tid;
static volatile uint8_t gun_voice_state;
static uint16_t gun_voice_feed_index;
static volatile uint16_t gun_voice_sample_count; /* CCM buffer follows; keep 0x20018000 alias free in RAM */
static uint8_t gun_voice_buffer[UMH_GUN_VOICE_MAX_SAMPLES]
    __attribute__((section(".ccmram"), aligned(4)));
static uint32_t gun_voice_feed_sequence;
/* Capture keeps one fixed companded scale; playback processing borrows the
 * inactive aperture ring and never allocates a second recording. */
static int32_t gun_voice_dc_q8;
static int16_t gun_voice_fir_hist[2u * UMH_GUN_VOICE_FIR_HISTORY];
static uint8_t gun_voice_fir_head;
static uint8_t gun_voice_decim_phase;
static uint32_t gun_voice_prime_count;
static gun_voice_processor_t gun_voice_processor;
static uint32_t gun_voice_capture_max_cycles;
static uint32_t gun_voice_process_max_cycles;
static uint32_t gun_voice_raw_invalid;
_Static_assert(UMH_GUN_VOICE_MAX_SAMPLES <= CCMSRAM_SIZE,
               "voice buffer must fit in STM32G491 CCMSRAM");
_Static_assert(GUN_VOICE_WORK_BYTES <= UMH_MOD_RING_SIZE,
               "voice processing must fit the inactive aperture ring");
_Static_assert(offsetof(umh_aperture_mod_t, envelope_ring) % 4u == 0u,
               "voice FFT workspace must be word aligned");

enum {
  GUN_VOICE_IDLE = 0u,
  GUN_VOICE_RECORDING = 1u,
  GUN_VOICE_FULL = 2u,
  GUN_VOICE_PLAYBACK = 3u,
  GUN_VOICE_DRAINING = 4u,
  GUN_VOICE_READY = 5u
};
#define GUN_VOICE_PRIME_SAMPLES 32u

static void gun_voice_chain_reset(void)
{
  gun_voice_dc_q8 = 0;
  gun_voice_decim_phase = 0u;
  gun_voice_prime_count = GUN_VOICE_PRIME_SAMPLES;
  gun_voice_fir_head = 0u;
  gun_voice_capture_max_cycles = 0u;
  gun_voice_process_max_cycles = 0u;
  gun_voice_raw_invalid = 0u;
  memset(&gun_voice_processor, 0, sizeof(gun_voice_processor));
  memset(gun_voice_fir_hist, 0, sizeof(gun_voice_fir_hist));
}

/* 16 kHz input -> DC removal -> M4 dual-MAC anti-alias FIR -> 8 kHz.
 * Fixed mu-law companding preserves quiet speech without a local AGC that
 * would normalize the hiss in pauses.  Detection/denoising uses decoded PCM
 * before one common gain is chosen for the whole recording. */
static void gun_voice_store_sample(int16_t pcm)
{
  int32_t centred;
  int32_t filtered;
  uint8_t i;
  /* FPGA's density count is scaled by 64: nonzero low bits diagnose SPI
   * corruption rather than microphone noise.  Readable over running SWD. */
  if (((uint16_t)pcm & 63u) != 0u) ++gun_voice_raw_invalid;
  if (gun_voice_prime_count == GUN_VOICE_PRIME_SAMPLES &&
      gun_voice_decim_phase == 0u)
    gun_voice_dc_q8 = (int32_t)pcm * 256;
  gun_voice_dc_q8 += (((int32_t)pcm * 256) - gun_voice_dc_q8) >> 5;
  centred = (int32_t)pcm - (gun_voice_dc_q8 >> 8);
  gun_voice_fir_head = (uint8_t)((gun_voice_fir_head - 1u) &
                               (UMH_GUN_VOICE_FIR_HISTORY - 1u));
  gun_voice_fir_hist[gun_voice_fir_head] = (int16_t)__SSAT(centred, 16);
  gun_voice_fir_hist[gun_voice_fir_head + UMH_GUN_VOICE_FIR_HISTORY] =
      gun_voice_fir_hist[gun_voice_fir_head];
  if (++gun_voice_decim_phase < 2u) return;
  gun_voice_decim_phase = 0u;
  {
    const int16_t *history = &gun_voice_fir_hist[gun_voice_fir_head];
    filtered = history[UMH_GUN_VOICE_FIR_TAPS / 2u] *
               (int32_t)umh_gun_voice_fir[UMH_GUN_VOICE_FIR_TAPS / 2u];
    for (i = 0u; i < UMH_GUN_VOICE_FIR_TAPS / 2u; ++i) {
      uint32_t pair = (uint16_t)history[i] |
          ((uint32_t)(uint16_t)history[UMH_GUN_VOICE_FIR_TAPS - 1u - i] << 16);
      uint32_t coefficient = (uint16_t)umh_gun_voice_fir[i];
      coefficient |= coefficient << 16;
      filtered = __SMLAD(pair, coefficient, filtered);
    }
  }
  if (gun_voice_prime_count != 0u) { --gun_voice_prime_count; return; }
  if (gun_voice_sample_count < UMH_GUN_VOICE_MAX_SAMPLES) {
    gun_voice_buffer[gun_voice_sample_count++] =
        gun_voice_encode(__SSAT((filtered + 16384) >> 15, 16));
    if (gun_voice_state == GUN_VOICE_RECORDING &&
        gun_voice_sample_count >= UMH_GUN_VOICE_MAX_SAMPLES)
      gun_voice_state = GUN_VOICE_FULL;
  }
}

static void gun_voice_capture_service(void)
{
  int16_t pcm[FPGA_MIC_AUDIO_BATCH];
  uint8_t count;
  uint8_t i;
  uint8_t batch;
  uint32_t start = DWT->CYCCNT;
  uint32_t elapsed;
  /* The 32-sample FPGA FIFO covers only 2 ms.  Drain a second full batch
   * after task jitter, but bound the work so the render task always yields. */
  for (batch = 0u; batch < 2u && gun_voice_state == GUN_VOICE_RECORDING; ++batch) {
    count = 0u;
    if (fpga_link_mic_audio_read(&fpga_link, pcm, FPGA_MIC_AUDIO_BATCH, &count) != 0)
      break;
    for (i = 0u; i < count && gun_voice_state == GUN_VOICE_RECORDING; ++i)
      gun_voice_store_sample(pcm[i]);
    if (count < FPGA_MIC_AUDIO_BATCH) break;
  }
  elapsed = DWT->CYCCNT - start;
  if (elapsed > gun_voice_capture_max_cycles) gun_voice_capture_max_cycles = elapsed;
}

/* Release hands the buffer to the render task.  Processing is bounded per
 * iteration while the array stays stopped; the ordinary modulation path is
 * configured only after the entire envelope is ready. */
static int gun_voice_prepare_service(void)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t elapsed;
  int result;
  /* The ring is scratch only while the voice session owns the stopped array. */
  if (aperture_mod_owns_output(&aperture_mod) != 0u ||
      gun_trigger_owns_output(&gun_trigger) == 0u ||
      gun_trigger_mode(&gun_trigger) != (uint8_t)UMH_GUN_MODE_VOICE)
    result = -1;
  else {
    if (gun_voice_processor.samples == NULL) {
      gun_voice_process_begin(&gun_voice_processor, gun_voice_buffer,
                              gun_voice_sample_count, aperture_mod.envelope_ring);
    }
    result = gun_voice_process_service(&gun_voice_processor);
  }
  elapsed = DWT->CYCCNT - start;
  if (elapsed > gun_voice_process_max_cycles) gun_voice_process_max_cycles = elapsed;
  if (result < 0) {
    gun_voice_state = GUN_VOICE_IDLE;
    (void)fpga_link_safe_stop(&fpga_link);
    gun_trigger_drop_output(&gun_trigger);
    system_status_clear(UMH_SYSTEM_PLAYING | UMH_SYSTEM_RECORDING);
  }
  return result;
}

/* STOP may arrive during the bounded release-time passes, before the AM
 * module is configured.  Cancel that pending playback as well as AM output.
 * The render task has higher priority, so protocol commands run between passes. */
static void gun_voice_cancel_pending(void)
{
  if (gun_voice_state != GUN_VOICE_READY) return;
  gun_voice_state = GUN_VOICE_IDLE;
  gun_trigger_drop_output(&gun_trigger);
  system_status_clear(UMH_SYSTEM_PLAYING | UMH_SYSTEM_RECORDING);
}

static int gun_voice_start_playback(void)
{
  umh_mod_config_wire_t config;
  uint8_t initial[UMH_GUN_VOICE_PREBUFFER];
  uint16_t first;
  int rc;
  system_status_clear(UMH_SYSTEM_RECORDING);
  if (gun_voice_sample_count < 80u) {
    gun_voice_state = GUN_VOICE_IDLE;
    (void)fpga_link_safe_stop(&fpga_link);
    gun_trigger_drop_output(&gun_trigger);
    system_status_clear(UMH_SYSTEM_PLAYING | UMH_SYSTEM_RECORDING);
    return 0;
  }
  memset(&config, 0, sizeof(config));
  /* 255 selects the module's full ceiling, UMH_MOD_MAX_LEVEL = 128 slots, which
   * is the 50 % duty that actually radiates.  The envelope underneath runs from
   * 0 to 128, so this is the complete modulation-depth range the FPGA's common
   * level path accepts. */
  config.level = 255u;
  config.envelope_rate_hz = UMH_GUN_VOICE_OUTPUT_RATE;
  config.prebuffer_samples = UMH_GUN_VOICE_PREBUFFER;
  config.flags = UMH_MOD_CONFIG_PLANE_WAVE;
  rc = aperture_mod_configure(&aperture_mod, &renderer, &fpga_link,
                              (const uint8_t *)&config, sizeof(config));
  if (rc != 0) goto fail;
  first = gun_voice_sample_count > sizeof(initial) ? (uint16_t)sizeof(initial)
                                                   : gun_voice_sample_count;
  memcpy(initial, gun_voice_buffer, first);
  if (aperture_mod_feed(&aperture_mod, initial, first, gun_voice_feed_sequence++) != 0 ||
      aperture_mod_start(&aperture_mod) != 0) goto fail;
  gun_voice_feed_index = first;
  gun_voice_state = GUN_VOICE_PLAYBACK;
  system_status_set(UMH_SYSTEM_PLAYING);
  return 0;
fail:
  gun_voice_state = GUN_VOICE_IDLE;
  abort_aperture_output();
  gun_trigger_drop_output(&gun_trigger);
  system_status_clear(UMH_SYSTEM_PLAYING | UMH_SYSTEM_RECORDING);
  return -1;
}

#define GUN_VOICE_FEED_BATCH 128u
#define GUN_VOICE_RING_TARGET 256u
#define GUN_VOICE_TAIL_SAMPLES 64u
_Static_assert(GUN_VOICE_FEED_BATCH >= GUN_VOICE_TAIL_SAMPLES &&
               GUN_VOICE_RING_TARGET + GUN_VOICE_TAIL_SAMPLES <= UMH_MOD_RING_SIZE,
               "voice refill and fade tail must fit their buffers");
_Static_assert(UMH_GUN_VOICE_PREBUFFER <= UMH_MOD_RING_SIZE,
               "voice prebuffer must fit the envelope ring");
_Static_assert(UMH_GUN_VOICE_INPUT_RATE == 2u * UMH_GUN_VOICE_SAMPLE_RATE,
               "voice decimator must be 2:1");
_Static_assert(UMH_GUN_VOICE_BIAS == 64u && UMH_GUN_VOICE_SWING == 64u &&
               UMH_MOD_MAX_LEVEL == 128u, "voice DSP envelope range");

/* One render iteration of playback: top the envelope ring up from the recording.
 * The FPGA pops the FIFO on its own 20 kHz clock, so this only has to stay ahead
 * of it; the aperture module holds the last sample and fades out on a real gap. */
static void gun_voice_playback_service(void)
{
  umh_mod_status_wire_t status;
  uint8_t block[GUN_VOICE_FEED_BATCH];
  uint16_t count;
  uint16_t i;
  if (gun_voice_state != GUN_VOICE_PLAYBACK && gun_voice_state != GUN_VOICE_DRAINING)
    return;
  aperture_mod_get_status(&aperture_mod, &status);
  if (status.state == UMH_MOD_OFF) {
    gun_voice_state = GUN_VOICE_IDLE;
    gun_trigger_drop_output(&gun_trigger);
    system_status_clear(UMH_SYSTEM_PLAYING | UMH_SYSTEM_UNDERRUN | UMH_SYSTEM_RECORDING);
    return;
  }
  if (status.state == UMH_MOD_FAULT) {
    aperture_mod_request_stop(&aperture_mod);
    gun_voice_state = GUN_VOICE_DRAINING;
    return;
  }
  if (status.state == UMH_MOD_STOPPING) {
    gun_voice_state = GUN_VOICE_DRAINING;
    return;
  }
  if (gun_voice_state == GUN_VOICE_DRAINING) {
    /* A zero-valued tail already follows the last voice sample.  Waiting for
     * the ring to empty preserves every queued sample; STOP still uses the
     * module's ordinary fade and drains the FPGA FIFO. */
    if (status.ring_fill == 0u) aperture_mod_request_stop(&aperture_mod);
    return;
  }
  if (status.ring_fill >= GUN_VOICE_RING_TARGET) return;
  count = (uint16_t)(gun_voice_sample_count - gun_voice_feed_index);
  if (count > GUN_VOICE_RING_TARGET - status.ring_fill)
    count = (uint16_t)(GUN_VOICE_RING_TARGET - status.ring_fill);
  if (count > sizeof(block)) count = (uint16_t)sizeof(block);
  memcpy(block, &gun_voice_buffer[gun_voice_feed_index], count);
  if (count != 0u && aperture_mod_feed(&aperture_mod, block, count,
                                     gun_voice_feed_sequence++) == 0)
    gun_voice_feed_index = (uint16_t)(gun_voice_feed_index + count);
  if (gun_voice_feed_index >= gun_voice_sample_count) {
    /* Fade modulation to silence in the source domain before requesting STOP.
     * This tail is short, bounded and only appended once. */
    uint8_t last = count != 0u ? block[count - 1u] : (uint8_t)UMH_GUN_VOICE_BIAS;
    for (i = 0u; i < GUN_VOICE_TAIL_SAMPLES; ++i)
      block[i] = (uint8_t)((uint32_t)last * (GUN_VOICE_TAIL_SAMPLES - 1u - i) /
                          GUN_VOICE_TAIL_SAMPLES);
    if (aperture_mod_feed(&aperture_mod, block, GUN_VOICE_TAIL_SAMPLES,
                         gun_voice_feed_sequence++) == 0)
      gun_voice_state = GUN_VOICE_DRAINING;
  }
}

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

static SemaphoreHandle_t storage_free_sem;
static SemaphoreHandle_t storage_ready_sem;
static SemaphoreHandle_t storage_wake_sem;
static StaticSemaphore_t storage_free_mem;
static StaticSemaphore_t storage_ready_mem;
static StaticSemaphore_t storage_wake_mem;
static umh_protocol_frame_t storage_request_buffer;
/* Consolidate the 2048 B storage buffers into one union.  response and data
 * are used by different FLASH message types (LIST replies straight into
 * response, READ fills data and frames it in response, WRITE reads its payload
 * from the incoming frame), so a second array for the record list would only
 * duplicate a buffer FLASH_LIST never needs at the same time. */
static union {
  uint8_t response[UMH_PROTOCOL_MAX_PAYLOAD];
  uint8_t data[UMH_PROTOCOL_MAX_PAYLOAD];
} storage_scratch;
#define storage_response (storage_scratch.response)
#define storage_data (storage_scratch.data)
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
  /* v2 extension (axial table-echo calibration), appended after the legacy
   * 135-byte prefix so older hosts can still parse the first part. */
  float correction_rms_deg;
  float amp_p10;
  float amp_p90;
  uint16_t pairs_used;
  uint8_t channels_covered;
  uint8_t dead_count;
  uint8_t verify_wins;
  uint8_t verify_sets;
  uint8_t committed;          /* 1 = written to EEPROM and applied */
  uint8_t method;             /* 2 = axial table echo */
  uint8_t amplitude[UMH_DEVICE_CHANNEL_COUNT];
  uint8_t coverage[UMH_DEVICE_CHANNEL_COUNT];
} umh_cal_result_wire_t;

_Static_assert(sizeof(umh_cal_result_wire_t) == 135u + 20u + 2u * UMH_DEVICE_CHANNEL_COUNT,
               "cal result wire size");
static uint8_t calibration_committed;

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
static volatile uint8_t render_waiting_for_work;

static void render_wake(void)
{
  if (render_waiting_for_work != 0u && render_task_handle != NULL)
    (void)xTaskNotifyGive(render_task_handle);
}

static TIM_HandleTypeDef render_timer;
static volatile uint8_t render_timer_ready;

static umh_fault_code_t fault_code_for_status(umh_status_t status, uint8_t message_type)
{
  if (status == UMH_STATUS_IO) {
    if (message_type == UMH_MSG_EEPROM_COMMIT || message_type == UMH_MSG_EEPROM_WRITE)
      return UMH_FAULT_EEPROM_IO;
    if ((message_type >= UMH_MSG_FLASH_LIST && message_type <= UMH_MSG_FLASH_DELETE) ||
        (message_type >= UMH_MSG_LIB_BEGIN && message_type <= UMH_MSG_LIB_STOP))
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
static StaticTask_t input_task_cb, display_task_cb;
static StackType_t input_task_stack[128];
static const osThreadAttr_t input_task_attributes = {
  .name = "input", .cb_mem = &input_task_cb, .cb_size = sizeof(input_task_cb),
  .stack_mem = input_task_stack, .stack_size = sizeof(input_task_stack),
  .priority = osPriorityNormal
};
/* Task stacks.
 *
 * The compiler emits one -fstack-usage frame per function and a call-graph
 * file, so the worst-case chain of every task entry point is known exactly.
 * Each grant below is that chain plus headroom for the 8-word Cortex-M4
 * exception frame, the ISRs that can pre-empt the task and FreeRTOS
 * bookkeeping, and was then checked against the measured stack watermark
 * (minimum free across the bench loads, bytes) instead of a round kilobyte.
 *
 *   task       worst chain   grant   free at worst case
 *   render          1176 B   2560    1448 B   frame-critical, kept generous
 *   protocol        1240 B   2560     896 B   worst chain is the EEPROM/I2C
 *                                            bring-up in application_init; the
 *                                            PA9 focused-shot path
 *                                            (gun_trigger_emit -> fpga_link_submit)
 *                                            is 928 B, and its accumulation
 *                                            arrays live in the gun_trigger_t,
 *                                            not here
 *   storage          376 B    960     796 B   the measured 164 B use never runs
 *                                            the flash/EEPROM chain, so the
 *                                            static chain is what sizes it
 *   ui               n/a     3072    2684 B   GUI callbacks arrive through
 *                                            device_gui function pointers, which
 *                                            static analysis cannot follow
 *   input            132 B    512     356 B
 *   health           124 B    512     220 B
 *   cal             3708 B   5120    4936 B   (deepest chain in the image)
 */
static StackType_t protocol_task_stack[640], render_task_stack[640], storage_task_stack[320], ui_task_stack[768], health_task_stack[128];
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
  .priority = osPriorityAboveNormal
};
static const osThreadAttr_t ui_task_attributes = {
  .name = "ui", .cb_mem = &ui_task_cb, .cb_size = sizeof(ui_task_cb),
  .stack_mem = ui_task_stack, .stack_size = sizeof(ui_task_stack),
  .priority = osPriorityBelowNormal
};
/* The panel task is deliberately at the same priority as the UI: it only has to
 * keep the 30 ms refresh cadence, and a display fault must never outrank input
 * handling.  Its stack is the GUI render chain (104 B) plus the HAL I2C call;
 * 1536 B leaves 1276 B free at the measured watermark. */
static StackType_t display_task_stack[384];
static const osThreadAttr_t display_task_attributes = {
  .name = "disp", .cb_mem = &display_task_cb, .cb_size = sizeof(display_task_cb),
  .stack_mem = display_task_stack, .stack_size = sizeof(display_task_stack),
  .priority = osPriorityBelowNormal
};
static StaticTask_t calibration_task_cb;
/* Deepest chain in the image: 3708 B measured from -fstack-usage plus the
 * call-graph file (calibration_task -> bench_service ->
 * calibration_selftest_session -> us_calibration_self_test ->
 * cal_self_test_eval -> cal_measure_frame_diff -> ... -> mic_command), plus the
 * 8-word Cortex-M4 exception frame and room for a nested USB/SPI ISR frame.
 * The v4 high-water figure of 4680 B that the old 4 KiB stack overflowed on
 * belonged to a code base that allocated a whole spatial_renderer_t inside
 * cal_self_test_eval; that frame is 1880 B now.  5120 B leaves ~1.3 KiB of
 * margin over the measured worst case and 1 KiB less than the old grant. */
static StackType_t calibration_task_stack[1280];
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
static void discard_block_stream(void);
static void flash_library_storage_process(const umh_protocol_frame_t *frame);
static void flash_upload_abort(void);
static void application_init(void);
static void usb_audio_cancel(void);
static void abort_aperture_output(void);
static int gui_demo(void *context);
static int gui_gun_select(void *context);
static int gui_gun_exit(void *context);

/* Use unified serialization helper from umh_utils.h */
#define read_u32(p) umh_read_u32_le(p)

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
  /* The calibration sessions need undisturbed echo windows, so they never
   * start over a held shot (the shot itself is deferred the same way). */
  if (gun_trigger_owns_output(&gun_trigger) != 0u) return -1;
  if (aperture_mod_owns_output(&aperture_mod) != 0u) abort_aperture_output();
  if (motion_engine_owns_output(&motion_engine) != 0u) motion_engine_abort(&motion_engine, &fpga_link);
  device_gui_calibration_begin(&device_gui);
  (void)xTaskNotifyGive((TaskHandle_t)calibration_task_handle);
  return 0;
}

static int gui_selftest(void *context)
{
  (void)context;
  if (calibration_task_handle == NULL) return -1;
  if (gun_trigger_owns_output(&gun_trigger) != 0u) return -1;
  if (aperture_mod_owns_output(&aperture_mod) != 0u) abort_aperture_output();
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
  calibration_result.progress = progress;   /* live progress for CAL_RESULT polling */
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
  uint8_t *encoded;
  uint16_t encoded_length;
  (void)context;
  encoded = umh_usb_tx_acquire();
  if (encoded == NULL) return -1;
  encoded_length = umh_protocol_encode(UMH_MSG_CAL_RAW, UMH_FLAG_RESPONSE,
                                       0x00524157u, first_pattern,
                                       data, length, encoded, UMH_USB_TX_SLOT_SIZE);
  if (encoded_length == 0u || umh_usb_tx_commit(encoded, encoded_length) == 0u) return -1;
  umh_usb_tx_service();
  return 0;
}

static void calibration_raw_session(void)
{
  int rc;
  uint32_t saved_cr1;
  if (aperture_mod_owns_output(&aperture_mod) != 0u) abort_aperture_output();
  if (motion_engine_owns_output(&motion_engine) != 0u) motion_engine_abort(&motion_engine, &fpga_link);
  device_gui_calibration_begin(&device_gui);
  playback_plan_stop(&playback_plan);
  (void)fpga_link_safe_stop(&fpga_link);
  discard_block_stream();
  demo_building = 1u;
  calibration_progress(DEVICE_GUI_CAL_WAIT, 0u, NULL);
  osDelay(1000u);
  saved_cr1 = fpga_link_calibration_link_begin(&fpga_link);
  rc = us_calibration_capture_raw(&fpga_link, calibration_raw_level,
                                  calibration_raw_gate_start,
                                  calibration_raw_gate_width,
                                  calibration_raw_burst_us,
                                  calibration_raw_patterns,
                                  calibration_raw_tx, NULL);
  fpga_link_calibration_link_end(&fpga_link, saved_cr1);
  if (rc != 0) {
    calibration_fail(rc == -2 ? UMH_FAULT_CAL_MIC_SILENT : UMH_FAULT_CAL_SOLVER,
                     (uint32_t)(-rc));
  } else {
    calibration_progress(DEVICE_GUI_CAL_OK, 100u, NULL);
  }
  (void)fpga_link_safe_stop(&fpga_link);
  demo_building = 0u;
}

/* One CAL_PROBE request.  The FPGA output is taken over exactly like the
 * other calibration sessions, but without the OLED calibration screen or
 * the 1 s pre-roll, so a host can sweep all channels quickly. */
static void calibration_probe_session(void)
{
  int rc;
  uint32_t saved_cr1;
  uint32_t scratch_bytes = 0u;
  uint16_t saturated = 0u;
  uint16_t length;
  uint8_t *encoded;
  uint16_t encoded_length;
  float *out = us_calibration_scratch(&scratch_bytes);
  const umh_cal_probe_request_t *rq = &calibration_probe_request;
  if (aperture_mod_owns_output(&aperture_mod) != 0u) abort_aperture_output();
  if (motion_engine_owns_output(&motion_engine) != 0u) motion_engine_abort(&motion_engine, &fpga_link);
  playback_plan_stop(&playback_plan);
  (void)fpga_link_safe_stop(&fpga_link);
  discard_block_stream();
  demo_building = 1u;
  saved_cr1 = fpga_link_calibration_link_begin(&fpga_link);
  rc = us_calibration_probe(&fpga_link, rq->phase, rq->level, rq->gate_count,
                            rq->gate_start, rq->gate_step, rq->gate_width,
                            rq->burst_us, rq->settle_us, rq->repeats, rq->flags,
                            out, &saturated);
  fpga_link_calibration_link_end(&fpga_link, saved_cr1);
  (void)fpga_link_safe_stop(&fpga_link);
  demo_building = 0u;
  length = (rc == 0) ? (uint16_t)((uint32_t)rq->gate_count * 8u * sizeof(float)) : 0u;
  if ((uint32_t)length > scratch_bytes || length > UMH_PROTOCOL_MAX_PAYLOAD) length = 0u;
  encoded = umh_usb_tx_acquire();
  if (encoded == NULL) return;
  /* stream_sequence = saturated blocks << 16 | (uint16)rc. */
  encoded_length = umh_protocol_encode(UMH_MSG_CAL_PROBE,
                                       (uint8_t)(UMH_FLAG_RESPONSE | (rc != 0 ? UMH_FLAG_ERROR : 0u)),
                                       calibration_probe_tid,
                                       ((uint32_t)saturated << 16) | ((uint32_t)rc & 0xFFFFu),
                                       (const uint8_t *)out, length, encoded, UMH_USB_TX_SLOT_SIZE);
  if (encoded_length != 0u) (void)umh_usb_tx_commit(encoded, encoded_length);
  umh_usb_tx_service();
}

static void calibration_selftest_session(void)
{
  int rc;
  uint32_t saved_cr1;
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
  if (aperture_mod_owns_output(&aperture_mod) != 0u) abort_aperture_output();
  if (motion_engine_owns_output(&motion_engine) != 0u) motion_engine_abort(&motion_engine, &fpga_link);
  device_gui_calibration_begin(&device_gui);
  playback_plan_stop(&playback_plan);
  (void)fpga_link_safe_stop(&fpga_link);
  discard_block_stream();
  demo_building = 1u;
  calibration_progress(DEVICE_GUI_CAL_WAIT, 0u, NULL);
  osDelay(300u);
  saved_cr1 = fpga_link_calibration_link_begin(&fpga_link);
  rc = us_calibration_self_test(&fpga_link, device_profile_get(), renderer.calibration,
                                level, gate_start, gate_width, 0u,
                                calibration_progress, NULL,
                                &calibration_self_test_result);
  fpga_link_calibration_link_end(&fpga_link, saved_cr1);
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
  uint32_t saved_cr1;
  if (aperture_mod_owns_output(&aperture_mod) != 0u) abort_aperture_output();
  if (motion_engine_owns_output(&motion_engine) != 0u) motion_engine_abort(&motion_engine, &fpga_link);
  device_gui_calibration_begin(&device_gui);
  playback_plan_stop(&playback_plan);
  (void)fpga_link_safe_stop(&fpga_link);
  discard_block_stream();
  demo_building = 1u;
  calibration_committed = 0u;
  calibration_progress(DEVICE_GUI_CAL_WAIT, 0u, NULL);
  osDelay(1000u);
  saved_cr1 = fpga_link_calibration_link_begin(&fpga_link);
  rc = us_calibration_run(&fpga_link, profile, calibration_progress, NULL,
                          &calibration_result);
  fpga_link_calibration_link_end(&fpga_link, saved_cr1);
  if (rc == 0) {
    /* us_calibration_run() only returns 0 after the candidate beat (or
     * matched) zero correction in a real production-renderer focus test on
     * the table echo, so it replaces the stored phase map.  Gain and RGB are
     * kept; channels without a measurable echo are disabled. */
    eeprom_profile_record_t record = eeprom_profile.record;
    record.version = EEPROM_PROFILE_VERSION;
    memset(record.enabled, 0, sizeof(record.enabled));
    for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) {
      record.phase[i] = (uint16_t)((uint16_t)calibration_result.phase_byte[i] << 8);
      if (calibration_result.coverage[i] != 0xFFu)
        record.enabled[i / 8u] |= (uint8_t)(1u << (i % 8u));
    }
    record.cal_meta_valid = 1u;
    {
      float rms10 = calibration_result.fit_rms_deg * 10.0f + 0.5f;
      if (rms10 > 255.0f) rms10 = 255.0f;
      record.cal_rms_deg_x10 = (uint8_t)rms10;
    }
    {
      float gx10 = calibration_result.verify_gain_db * 10.0f;
      float cx10 = calibration_result.correction_rms_deg * 10.0f;
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
      /* [0..1] cone pairs, [2..4] steady near-field gate for SELFTEST
       * (legacy 500 us / 400 us), [5] level, [6] method 2 = table echo,
       * [7] verify wins, [8..9] table distance 0.1 mm, [10] dead channels,
       * [11] echo-plan gate start. */
      uint16_t pairs = calibration_result.pairs_used;
      uint16_t self_gate = 20u;
      uint16_t table10 = (uint16_t)lroundf(calibration_result.table_mm * 10.0f);
      memcpy(&record.cal_reserved[0], &pairs, sizeof(pairs));
      memcpy(&record.cal_reserved[2], &self_gate, sizeof(self_gate));
      record.cal_reserved[4] = 16u;
      record.cal_reserved[5] = calibration_result.level_used;
      record.cal_reserved[6] = calibration_result.geom_hypothesis;
      record.cal_reserved[7] = calibration_result.sign_hypothesis;
      memcpy(&record.cal_reserved[8], &table10, sizeof(table10));
      record.cal_reserved[10] = calibration_result.dead_count;
      record.cal_reserved[11] = (uint8_t)calibration_result.used_gate_start;
      record.reserved[0] = calibration_result.used_gate_count;
      record.reserved[1] = 0u;
      record.reserved[2] = 0u;
    }
    if (eeprom_profile_commit(&eeprom_profile, &record) != 0) {
      calibration_fail(UMH_FAULT_EEPROM_IO, 3u);
    } else {
      calibration_committed = 1u;
      apply_calibration_from_eeprom();
      system_status_get()->cal_rms_deg_x10 = record.cal_rms_deg_x10;
      system_status_get()->cal_tilt_x_x10 = record.cal_tilt_x_x10;
      system_status_get()->cal_tilt_y_x10 = record.cal_tilt_y_x10;
      system_status_get()->cal_good_mics = calibration_result.good_mics;
      system_status_get()->cal_state = DEVICE_GUI_CAL_OK;
      system_status_get()->cal_last_ms = HAL_GetTick();
    }
  } else {
    umh_fault_code_t code = (umh_fault_code_t)calibration_result.fault;
    if (code == UMH_FAULT_NONE) code = UMH_FAULT_CAL_SOLVER;
    calibration_fail(code, (uint32_t)(-rc));
  }
  (void)fpga_link_safe_stop(&fpga_link);
  demo_building = 0u;
}


static void bench_service(void)
{
  uint32_t cmd = umh_bench_cmd;
  uint32_t saved_cr1;
  int rc = 0;
  uint16_t i;
  if (cmd == UMH_BENCH_CMD_NONE) return;
  if (aperture_mod_owns_output(&aperture_mod) != 0u) abort_aperture_output();
  if (motion_engine_owns_output(&motion_engine) != 0u) motion_engine_abort(&motion_engine, &fpga_link);
  umh_bench_cmd = UMH_BENCH_CMD_NONE;
  umh_bench_status = 1u;
  umh_bench_result = 0u;
  saved_cr1 = fpga_link_calibration_link_begin(&fpga_link);
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
      discard_block_stream();
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
    case UMH_BENCH_CMD_PATTERN:
      playback_plan_stop(&playback_plan);
      (void)fpga_link_safe_stop(&fpga_link);
      discard_block_stream();
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
    case UMH_BENCH_CMD_APPLY_PHASE:
      for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i)
        renderer.calibration[i].phase = umh_bench_phase[i];
      /* The renderer keeps a derived copy of the phase offsets, gains and the
       * calibrated-channel list, so a direct write into calibration[] has to
       * be published before the next render. */
      spatial_renderer_refresh_calibration(&renderer, UMH_DEVICE_CHANNEL_COUNT);
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
        else {
          /* The EEPROM write failed but calibration[] was already edited, so
           * publish it instead of leaving the renderer on stale derived
           * state. */
          spatial_renderer_refresh_calibration(&renderer, UMH_DEVICE_CHANNEL_COUNT);
          rc = -1;
        }
      }
      break;
    default:
      rc = -2;
      break;
  }
  fpga_link_calibration_link_end(&fpga_link, saved_cr1);
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
    if (calibration_probe_mode != 0u) {
      calibration_probe_session();
      calibration_probe_mode = 0u;
    } else if (calibration_selftest_mode != 0u) {
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

/* Abandon any block being streamed or built.
 *
 * Nineteen call sites used to open-code the same three statements when a
 * command, a demo or a calibration session took over the output: cancel the
 * parser, drop the stream flag and empty the frame ring.  Collecting them
 * keeps the takeover sequence identical everywhere, which is what stops a
 * half-parsed block from surviving into the next owner.  Every site used to be
 * reachable only from the protocol task, so this needs no locking of its own;
 * render_task only ever reads the ring. */
static void discard_block_stream(void)
{
  block_parser_cancel(&block_parser);
  block_stream_active = 0u;
  if (flash_play_active != 0u) {
    uint32_t wait_ms = 0u;
    flash_play_stop_request = 1u;
    if (storage_wake_sem != NULL) (void)xSemaphoreGive(storage_wake_sem);
    render_wake();
    while (flash_play_active != 0u && wait_ms < 30u &&
           xTaskGetCurrentTaskHandle() != storage_task_handle) {
      osDelay(1u);
      ++wait_ms;
    }
  }
  frame_ring_init(&frame_ring);
}

static int gui_demo(void *context)
{
  const umh_demo_descriptor_t *descriptor;
  int result;
  (void)context;
  descriptor = demo_engine_descriptor(device_gui.selected_demo);
  if (descriptor == NULL) return -1;
  /* A demo rebuilds the whole 24-frame ring; a held shot owns the output and
   * would be silently overwritten by the plan that follows. */
  if (gun_trigger_owns_output(&gun_trigger) != 0u) return -1;
  demo_building = 1u;
  if (aperture_mod_owns_output(&aperture_mod) != 0u) abort_aperture_output();
  if (motion_engine_owns_output(&motion_engine) != 0u) motion_engine_abort(&motion_engine, &fpga_link);
  playback_plan_stop(&playback_plan);
  (void)fpga_link_safe_stop(&fpga_link);
  discard_block_stream();
  result = demo_engine_build(device_gui.selected_demo, &renderer, &frame_ring,
                             system_time_us(), &playback_plan);
  demo_building = 0u;
  return result == 0 ? 0 : -1;
}


/* Gun menu selection.  The callback reads the row back out of the GUI, exactly
 * like the WS2812 action, because the action signature only carries the
 * context.  gun_trigger_select_mode() refuses while a shot is held; the GUI
 * then leaves the cursor on the mode that is actually being emitted. */
static int gui_gun_select(void *context)
{
  (void)context;
  return gun_trigger_select_mode(&gun_trigger, (uint8_t)device_gui.row);
}

/* Leaving the menu keeps the operator's selected mode active.  The trigger is
 * still initialized to focus at boot, so the default remains unchanged until a
 * menu selection is made. */
static int gui_gun_exit(void *context)
{
  (void)context;
  return 0;
}

/* --------------------------------------------------------------------------
 * PA9 shot trigger ("ultrasound gun").
 *
 * While the switch is held the device emits the gun mode the menu selected (mode
 * 1, the single focus, out of the box), no matter which producer was running
 * before.  The takeover is the same one the host MOTION_START and SET_DEMO paths
 * use -- claim the output, abort the producers, drop the block stream, STOP the
 * FPGA -- so a shot cannot leave a half-parsed block or a stale plan behind.
 * Both edges are hard: full power on the press, silence on the release.
 *
 * It runs in the protocol task, the context that already owns every other output
 * takeover.  The switch is sampled by the input task and the held image is kept
 * alive by the render task, which also advances the mode 5 bottle collapse; see
 * gun_trigger.h.
 * -------------------------------------------------------------------------- */
static int gun_shot_fire(void)
{
  /* Claim before the takeover: the render task tests this flag before it pulls a
   * frame out of the ring, so the reset below cannot race a frame in flight. */
  gun_trigger_claim_output(&gun_trigger);
  abort_aperture_output();
  motion_engine_abort(&motion_engine, NULL);
  hologram_engine_abort(&hologram_engine, NULL);
  playback_plan_stop(&playback_plan);
  discard_block_stream();
  if (gun_trigger_mode(&gun_trigger) == (uint8_t)UMH_GUN_MODE_VOICE) {
    if (fpga_link_safe_stop(&fpga_link) != 0 ||
        fpga_link_mic_audio_reset(&fpga_link) != 0) {
      gun_trigger_drop_output(&gun_trigger);
      system_status_clear(UMH_SYSTEM_PLAYING | UMH_SYSTEM_RECORDING);
      return -1;
    }
    gun_voice_sample_count = 0u;
    gun_voice_feed_index = 0u;
    gun_voice_feed_sequence = 0u;
    gun_voice_chain_reset();
    gun_voice_state = GUN_VOICE_RECORDING;
    system_status_clear(UMH_SYSTEM_PLAYING);
    system_status_set(UMH_SYSTEM_RECORDING);
    return 0;
  }
  if (fpga_link_safe_stop(&fpga_link) != 0 ||
      gun_trigger_emit(&gun_trigger, &renderer, &fpga_link) != 0) {
    gun_trigger_drop_output(&gun_trigger);
    system_status_clear(UMH_SYSTEM_PLAYING);
    return -1;
  }
  /* The LEDs come from the frame itself: gun_trigger_emit() writes the level into
   * frame.rgb, so the four WS2812s light with the emitted intensity.  A separate
   * fpga_link_set_ws2812() here would override that with a colour of its own. */
  /* The render task is the authoritative writer of PLAYING while the shot is
   * held: a press can land inside a render iteration whose normal-path tail would
   * clear a flag set here.  Setting it here only closes the gap before that next
   * render iteration. */
  system_status_set(UMH_SYSTEM_PLAYING);
  return 0;
}

static int gun_shot_release(void)
{
  if (gun_trigger_mode(&gun_trigger) == (uint8_t)UMH_GUN_MODE_VOICE &&
      (gun_voice_state == GUN_VOICE_RECORDING || gun_voice_state == GUN_VOICE_FULL)) {
    /* The render task owns the capture DSP and CCM store.  Hand off only the
     * state so release cannot normalize/feed a sample still being written. */
    gun_voice_state = GUN_VOICE_READY;
    system_status_clear(UMH_SYSTEM_RECORDING);
    return 0;
  }
  int result = gun_trigger_release(&gun_trigger, &fpga_link);
  if (gun_trigger_owns_output(&gun_trigger) == 0u)
    system_status_clear(UMH_SYSTEM_PLAYING);
  return result;
}

/* Edge handler: one call per protocol task iteration. */
static void gun_trigger_poll(void)
{
  static uint8_t previous_pressed;
  uint8_t pressed = gun_trigger_pressed(&gun_trigger);
  uint8_t owns = gun_trigger_owns_output(&gun_trigger);
  if (pressed != previous_pressed) {
    previous_pressed = pressed;
    if (pressed != 0u) system_status_set(UMH_SYSTEM_TRIGGER);
    else system_status_clear(UMH_SYSTEM_TRIGGER);
  }
  if (pressed != 0u) {
    /* A held shot needs no service here: the render task keeps it emitting. */
    if (owns != 0u) return;
    /* A calibration session measures timed echo windows and must keep the output
     * until it finishes; a demo under construction owns the frame ring for the
     * same moment.  The switch keeps being sampled, so a trigger that is still
     * held fires as soon as that work is done. */
    if (device_gui_calibration_busy(&device_gui) != 0u ||
        calibration_raw_mode != 0u || calibration_selftest_mode != 0u ||
        calibration_probe_mode != 0u || demo_building != 0u) {
      gun_trigger_note_deferred(&gun_trigger);
      return;
    }
    (void)gun_shot_fire();
  } else if (owns != 0u) {
    if (gun_voice_state == GUN_VOICE_READY || gun_voice_state == GUN_VOICE_PLAYBACK ||
        gun_voice_state == GUN_VOICE_DRAINING)
      return;
    (void)gun_shot_release();
  }
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
  uint8_t *output;
  uint16_t encoded;
  uint8_t flags = UMH_FLAG_RESPONSE;
  if (type == UMH_MSG_NACK) flags |= UMH_FLAG_ERROR;
  output = umh_usb_tx_acquire();
  if (output == NULL) {
    system_status_get()->usb_dropped++;
    system_status_fault(UMH_FAULT_USB_TX_DROP, system_status_get()->usb_dropped, UMH_FAULT_WARNING);
    return;
  }
  encoded = umh_protocol_encode(type, flags,
                                request != NULL ? request->header.transaction_id : 0u,
                                request != NULL ? request->header.stream_sequence : 0u,
                                (const uint8_t *)payload, length, output, UMH_USB_TX_SLOT_SIZE);
  if (encoded == 0u || umh_usb_tx_commit(output, encoded) == 0u) {
    system_status_get()->usb_dropped++;
    system_status_fault(UMH_FAULT_USB_TX_DROP, system_status_get()->usb_dropped, UMH_FAULT_WARNING);
  }
}

static int queue_storage_request(const umh_protocol_frame_t *frame)
{
  if (storage_free_sem == NULL || storage_ready_sem == NULL || frame == NULL)
    return -1;
  if (xSemaphoreTake(storage_free_sem, 0u) != pdTRUE) return -1;
  memcpy(&storage_request_buffer, frame, sizeof(storage_request_buffer));
  (void)xSemaphoreGive(storage_ready_sem);
  if (storage_wake_sem != NULL) (void)xSemaphoreGive(storage_wake_sem);
  return 0;
}

static void flash_upload_abort(void)
{
  if (flash_upload_active != 0u) {
    flash_library_writer_abort(&flash_writer);
    flash_upload_active = 0u;
  }
}

static umh_status_t flash_library_status_from_result(int result)
{
  switch (result) {
    case FLASH_LIBRARY_OK: return UMH_STATUS_OK;
    case FLASH_LIBRARY_PARAM: return UMH_STATUS_BAD_LENGTH;
    case FLASH_LIBRARY_FULL: return UMH_STATUS_NO_MEMORY;
    case FLASH_LIBRARY_BUSY: return UMH_STATUS_BUSY;
    case FLASH_LIBRARY_CORRUPT: return UMH_STATUS_CRC;
    case FLASH_LIBRARY_IO: return UMH_STATUS_IO;
    case FLASH_LIBRARY_NOT_FOUND: return UMH_STATUS_INVALID_STATE;
    default: return UMH_STATUS_INVALID_STATE;
  }
}

static void flash_library_status_fill(umh_lib_status_wire_t *wire)
{
  const flash_library_stats_t *stats = flash_library_stats(&flash_library);
  memset(wire, 0, sizeof(*wire));
  wire->state = flash_upload_active != 0u ? 1u :
                (flash_play_active != 0u ? 2u : 0u);
  wire->object_count = (uint8_t)flash_library_object_count(&flash_library);
  wire->active_bank = flash_library.active_bank;
  wire->mounted = flash_library.mounted;
  wire->free_bytes = flash_library_free_bytes(&flash_library);
  wire->next_generation = flash_library.next_generation;
  wire->upload_object_id = flash_upload_active != 0u ? flash_upload_object_id : 0u;
  wire->upload_received = flash_upload_active != 0u ? flash_writer.received_bytes : 0u;
  if (flash_play_active != 0u) {
    wire->play_object_id = flash_play_object_id;
    wire->play_record_count = flash_reader.record_count;
    wire->play_frame_period_us = flash_reader.frame_period_us;
    wire->play_cursor = flash_library_reader_cursor(&flash_reader);
  }
  wire->replay_frames = stats->replay_frames;
  wire->page_programs = stats->page_programs;
  wire->block_erases = stats->block_erases;
  wire->rotations = stats->rotations;
  wire->bytes_programmed = stats->bytes_programmed;
  wire->last_result = flash_last_result;
}

static umh_status_t flash_library_playback_start(const umh_lib_start_wire_t *wire)
{
  const flash_library_entry_t *entry;
  umh_playback_plan_wire_t plan_wire;
  umh_status_t status = UMH_STATUS_OK;
  uint32_t prefill;
  uint32_t i;
  uint64_t origin;
  if (wire->repeat > 1u) return UMH_STATUS_BAD_LENGTH;
  if (flash_play_active != 0u || flash_play_stop_request != 0u ||
      flash_upload_active != 0u) return UMH_STATUS_BUSY;
  if (storage_wake_sem == NULL) return UMH_STATUS_NO_MEMORY;
  if (aperture_mod_owns_output(&aperture_mod) != 0u ||
      motion_engine_owns_output(&motion_engine) != 0u ||
      hologram_engine_owns_output(&hologram_engine) != 0u ||
      gun_trigger_owns_output(&gun_trigger) != 0u ||
      device_gui_calibration_busy(&device_gui) != 0u ||
      demo_building != 0u) return UMH_STATUS_BUSY;
  if (flash_library.mounted == 0u) return UMH_STATUS_INVALID_STATE;
  entry = flash_library_object_find(&flash_library, wire->object_id);
  if (entry == NULL || entry->frame_period_us == 0u || entry->record_count == 0u)
    return UMH_STATUS_INVALID_STATE;
  playback_plan_stop(&playback_plan);
  block_parser_cancel(&block_parser);
  block_stream_active = 0u;
  frame_ring_init(&frame_ring);
  if (fpga_link_safe_stop(&fpga_link) != 0) return UMH_STATUS_IO;
  (void)fpga_link_set_ws2812(&fpga_link, 0u, 0u, 0u);
  prefill = entry->record_count < UMH_FLASH_PLAY_PREBUFFER ?
            entry->record_count : UMH_FLASH_PLAY_PREBUFFER;
  origin = system_time_us() + (uint64_t)prefill * entry->frame_period_us;
  if (flash_library_reader_open(&flash_reader, &flash_library, wire->object_id,
                                wire->repeat, origin) != FLASH_LIBRARY_OK)
    return UMH_STATUS_IO;
  for (i = 0u; i < prefill; ++i) {
    umh_output_frame_t frame;
    umh_output_frame_t *slot;
    if (flash_library_reader_read_frame(&flash_reader, i, &frame) != FLASH_LIBRARY_OK) {
      status = UMH_STATUS_IO;
      break;
    }
    slot = frame_ring_acquire_write(&frame_ring);
    if (slot == NULL) {
      status = UMH_STATUS_NO_MEMORY;
      break;
    }
    *slot = frame;
    if (frame_ring_commit_write(&frame_ring) != 0) {
      status = UMH_STATUS_NO_MEMORY;
      break;
    }
  }
  if (status == UMH_STATUS_OK &&
      flash_library_reader_seek(&flash_reader, prefill) != FLASH_LIBRARY_OK)
    status = UMH_STATUS_IO;
  if (status == UMH_STATUS_OK) {
    memset(&plan_wire, 0, sizeof(plan_wire));
    plan_wire.block_id = wire->object_id;
    plan_wire.start_mode = UMH_PLAN_START_IMMEDIATE;
    plan_wire.repeat_mode = UMH_PLAN_LOOP_STREAM;
    plan_wire.underrun_policy = UMH_UNDERRUN_REPORT;
    plan_wire.route = UMH_PLAN_ROUTE_DEFAULT;
    plan_wire.rate_numerator = 1u;
    plan_wire.rate_denominator = 1u;
    plan_wire.start_time = 0u;
    plan_wire.prebuffer_frames = prefill;
    if (playback_plan_set(&playback_plan, &plan_wire) != 0 ||
        playback_plan_start(&playback_plan, system_time_us(),
                            frame_ring_count(&frame_ring), origin) != 0)
      status = UMH_STATUS_INVALID_STATE;
  }
  if (status != UMH_STATUS_OK) {
    flash_library_reader_close(&flash_reader);
    flash_nor_dma_drain();
    frame_ring_init(&frame_ring);
    (void)fpga_link_safe_stop(&fpga_link);
    return status;
  }
  flash_play_object_id = wire->object_id;
  flash_play_stop_request = 0u;
  flash_play_active = 1u;
  (void)xSemaphoreGive(storage_wake_sem);
  render_wake();
  return UMH_STATUS_OK;
}

static void flash_library_storage_process(const umh_protocol_frame_t *frame)
{
  umh_status_t status = UMH_STATUS_OK;
  if (frame->header.message_type == UMH_MSG_LIB_BEGIN) {
    umh_lib_begin_wire_t wire;
    flash_library_object_info_t info;
    uint16_t expected;
    if (flash_play_active != 0u || flash_play_stop_request != 0u ||
        flash_upload_active != 0u) {
      status = UMH_STATUS_BUSY;
    } else if (frame->payload_size < sizeof(wire)) {
      status = UMH_STATUS_BAD_LENGTH;
    } else {
      memcpy(&wire, frame->payload, sizeof(wire));
      if (wire.level_mode > FLASH_LIBRARY_LEVEL_UNIFORM || wire.repeat > 1u ||
          wire.record_count == 0u || wire.frame_period_us == 0u ||
          wire.object_id == 0u) {
        status = UMH_STATUS_BAD_LENGTH;
      } else {
        expected = (uint16_t)(sizeof(wire) +
                   (wire.level_mode == FLASH_LIBRARY_LEVEL_UNIFORM ?
                    FLASH_LIBRARY_RECORD_PHASE_BYTES : 0u));
        if (frame->payload_size != expected) {
          status = UMH_STATUS_BAD_LENGTH;
        } else {
          memset(&info, 0, sizeof(info));
          info.object_id = wire.object_id;
          info.record_count = wire.record_count;
          info.frame_period_us = wire.frame_period_us;
          info.object_type = FLASH_LIBRARY_OBJECT_SEQUENCE;
          info.level_mode = wire.level_mode;
          info.repeat = wire.repeat;
          if (wire.level_mode == FLASH_LIBRARY_LEVEL_UNIFORM)
            memcpy(info.level_table, &frame->payload[sizeof(wire)],
                   FLASH_LIBRARY_RECORD_PHASE_BYTES);
          status = flash_library_status_from_result(
              flash_library_writer_begin(&flash_library, &flash_writer, &info));
          if (status == UMH_STATUS_OK) {
            flash_upload_active = 1u;
            flash_upload_object_id = wire.object_id;
            flash_upload_expected_sequence = frame->header.stream_sequence + 1u;
          }
        }
      }
    }
  } else if (frame->header.message_type == UMH_MSG_LIB_DATA) {
    if (flash_upload_active == 0u) {
      status = UMH_STATUS_INVALID_STATE;
    } else if (frame->header.stream_sequence != flash_upload_expected_sequence) {
      flash_upload_abort();
      status = UMH_STATUS_BAD_SEQUENCE;
    } else if (frame->payload_size == 0u ||
               frame->payload_size > UMH_PROTOCOL_MAX_PAYLOAD) {
      flash_upload_abort();
      status = UMH_STATUS_BAD_LENGTH;
    } else {
      int result = flash_library_writer_write(&flash_writer, frame->payload,
                                              frame->payload_size);
      if (result != FLASH_LIBRARY_OK) {
        flash_upload_abort();
        status = flash_library_status_from_result(result);
      } else {
        ++flash_upload_expected_sequence;
      }
    }
  } else if (frame->header.message_type == UMH_MSG_LIB_END) {
    if (flash_upload_active == 0u) {
      status = UMH_STATUS_INVALID_STATE;
    } else if (frame->payload_size != 0u) {
      flash_upload_abort();
      status = UMH_STATUS_BAD_LENGTH;
    } else if (frame->header.stream_sequence != flash_upload_expected_sequence) {
      flash_upload_abort();
      status = UMH_STATUS_BAD_SEQUENCE;
    } else {
      int result = flash_library_writer_finish(&flash_writer);
      flash_upload_active = 0u;
      status = flash_library_status_from_result(result);
    }
  } else if (frame->header.message_type == UMH_MSG_LIB_STATUS) {
    if (frame->payload_size != 0u) {
      status = UMH_STATUS_BAD_LENGTH;
    } else {
      umh_lib_status_wire_t wire;
      flash_library_status_fill(&wire);
      send_response(frame, UMH_MSG_LIB_STATUS, &wire, (uint16_t)sizeof(wire));
      return;
    }
  } else if (frame->header.message_type == UMH_MSG_LIB_START) {
    umh_lib_start_wire_t wire;
    if (frame->payload_size != sizeof(wire)) {
      status = UMH_STATUS_BAD_LENGTH;
    } else {
      memcpy(&wire, frame->payload, sizeof(wire));
      status = flash_library_playback_start(&wire);
    }
  } else if (frame->header.message_type == UMH_MSG_LIB_STOP) {
    if (frame->payload_size != 0u) {
      status = UMH_STATUS_BAD_LENGTH;
    } else {
      playback_plan_stop(&playback_plan);
      discard_block_stream();
      flash_library_reader_close(&flash_reader);
      flash_nor_dma_drain();
      flash_play_active = 0u;
      flash_play_stop_request = 0u;
      (void)fpga_link_safe_stop(&fpga_link);
    }
  } else {
    status = UMH_STATUS_UNSUPPORTED;
  }
  flash_last_result = (uint16_t)(status == UMH_STATUS_OK ? 0u : status);
  send_result(frame, status, NULL, 0u);
}

static void storage_process(const umh_protocol_frame_t *frame)
{
  uint32_t id;
  uint32_t length;
  uint16_t count;
  umh_status_t status = UMH_STATUS_OK;
  if (frame->header.message_type >= UMH_MSG_LIB_BEGIN &&
      frame->header.message_type <= UMH_MSG_LIB_STOP) {
    flash_library_storage_process(frame);
    return;
  }
  if (frame->header.message_type >= UMH_MSG_FLASH_LIST &&
      frame->header.message_type <= UMH_MSG_FLASH_DELETE &&
      (flash_play_active != 0u || flash_play_stop_request != 0u)) {
    send_result(frame, UMH_STATUS_BUSY, NULL, 0u);
    return;
  }
  if (frame->header.message_type == UMH_MSG_FLASH_LIST) {
    /* The record list is written straight into the response buffer and framed
     * in place: one 2 KiB buffer for both halves of the reply. */
    uint16_t capacity = (uint16_t)(sizeof(storage_response) / sizeof(flash_store_record_t));
    if (frame->payload_size != 0u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
    count = flash_store_list(&flash_store, (flash_store_record_t *)(void *)storage_response, capacity);
    send_response(frame, UMH_MSG_FLASH_LIST, storage_response,
                  (uint16_t)(count * sizeof(flash_store_record_t)));
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

/* UAC is a producer of the existing AM engine, never a second FPGA path.
 * All control/start/feed work runs in protocol task context.  Explicit legacy
 * takeovers inhibit this UAC stream until the host opens a new AS epoch. */
static void usb_audio_release(void)
{
  usb_audio_env_count = 0u;
  if (usb_audio_owns != 0u) {
    usb_audio_owns = 0u;
    umh_usb_audio_stats.owns_output = 0u;
    ++umh_usb_audio_stats.stops;
    aperture_mod_abort(&aperture_mod);
    system_status_clear(UMH_SYSTEM_USB_AUDIO);
  }
}

static void usb_audio_cancel(void)
{
  usb_audio_blocked_epoch = umh_usb_audio_generation();
  usb_audio_inhibited = 1u;
  umh_usb_audio_stats.inhibited = 1u;
  usb_audio_release();
}

static void abort_aperture_output(void)
{
  usb_audio_cancel();
  aperture_mod_abort(&aperture_mod);
}

static uint8_t usb_audio_output_busy(void)
{
  return device_gui_calibration_busy(&device_gui) != 0u || demo_building != 0u ||
      calibration_raw_mode != 0u || calibration_selftest_mode != 0u ||
      calibration_probe_mode != 0u || gun_trigger_owns_output(&gun_trigger) != 0u ||
      gun_voice_state != GUN_VOICE_IDLE ||
      motion_engine_owns_output(&motion_engine) != 0u ||
      hologram_engine_owns_output(&hologram_engine) != 0u ||
      playback_plan.running != 0u || block_stream_active != 0u ||
      block_parser.block.active != 0u || frame_ring_count(&frame_ring) != 0u ||
      (usb_audio_owns == 0u && aperture_mod_owns_output(&aperture_mod) != 0u);
}

static void usb_audio_service(void)
{
  int16_t pcm[UAC_SERVICE_BLOCK];
  uint8_t envelope[USB_AUDIO_DSP_OUTPUT_MAX];
  uint32_t epoch = umh_usb_audio_generation();
  uint8_t block;
  if (epoch != usb_audio_epoch) {
    usb_audio_release();
    usb_audio_dsp_reset(&usb_audio_dsp);
    usb_audio_epoch = epoch;
    usb_audio_sequence = 0u;
    if (epoch != usb_audio_blocked_epoch) usb_audio_inhibited = 0u;
  }
  if (usb_audio_owns != 0u && aperture_mod.state == UMH_MOD_OFF)
    usb_audio_cancel(); /* a hardware fault or another producer ended this session */
  if (umh_usb_audio_debug_inhibit != 0u) usb_audio_release();
  umh_usb_audio_stats.inhibited = usb_audio_inhibited;
  if (umh_usb_audio_streaming() == 0u || umh_usb_audio_mute() != 0u ||
      usb_audio_inhibited != 0u ||
      hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED ||
      (uint32_t)(HAL_GetTick() - umh_usb_audio_last_packet_ms()) >= UMH_USB_AUDIO_TIMEOUT_MS ||
      usb_audio_output_busy() != 0u) {
    usb_audio_release();
    umh_usb_audio_discard();
    usb_audio_dsp_reset(&usb_audio_dsp);
    return;
  }
  /* Four packets per pass bounds latency and leaves CDC and the trigger live.
   * The 20 kHz FIFO render task remains the highest-priority consumer. */
  for (block = 0u; block < 4u; ++block) {
    uint16_t count = umh_usb_audio_read(pcm, UAC_SERVICE_BLOCK);
    uint16_t output, i;
    uint8_t maximum = 0u, minimum = 128u;
    uint32_t cycles;
    if (count == 0u) break;
    cycles = DWT->CYCCNT;
    /* The UAC Feature Unit volume is deliberately not applied to the 8-bit
     * envelope.  The OS endpoint volume is commonly -6..-20 dB; scaling
     * 0..128 after the AGC leaves only a handful of codes and is heard as
     * the mechanical/robotic distortion of the UAC path.  UMH Speaker never
     * has this stage: the host sends the full-scale 0..128 envelope.  Keep
     * the modulator at 0 dB; the AGC already normalises whatever PCM level
     * the host delivers. */
    output = usb_audio_dsp_process(&usb_audio_dsp, pcm, count,
                                    32768u, envelope);
    cycles = DWT->CYCCNT - cycles;
    if (cycles > umh_usb_audio_stats.max_process_cycles)
      umh_usb_audio_stats.max_process_cycles = cycles;
    umh_usb_audio_stats.processed_samples += count;
    umh_usb_audio_stats.envelope_samples += output;
    for (i = 0u; i < output; ++i) {
      if (envelope[i] > maximum) maximum = envelope[i];
      if (envelope[i] < minimum) minimum = envelope[i];
    }
    umh_usb_audio_stats.level_min = minimum;
    umh_usb_audio_stats.level_max = maximum;
    if (output == 0u) continue;
    /* Recheck the USB epoch after DSP: an ISR may have reset/muted the stream
     * while the samples were processed.  Stale samples must never be fed. */
    if (umh_usb_audio_generation() != epoch || umh_usb_audio_mute() != 0u ||
        usb_audio_inhibited != 0u || umh_usb_audio_streaming() == 0u) {
      usb_audio_release();
      return;
    }
    if (usb_audio_owns == 0u) {
      if (umh_usb_audio_debug_inhibit != 0u) continue;
      const umh_mod_config_wire_t config = {
        .x_um = 0, .y_um = 0, .z_um = 1000000,
        .phase = 0u, .level = 255u, .envelope_rate_hz = 20000u,
        /* Match UMH Speaker's 1536-sample (76.8 ms) prebuffer.  The old 512
         * left the aperture clock servo target well above the actual fill,
         * which kept a permanent negative correction and made the servo hunt
         * slowly (audible as a 1..5 Hz pitch/level wobble on a steady tone).
         * The CDC reference path always runs at 1536. */
        .prebuffer_samples = 1536u, .flags = 0u
      };
      if (maximum == 0u) continue; /* no emission/ownership on digital silence */
      usb_audio_owns = 1u; /* publish before configure can yield on an SPI mutex */
      /* The CDC MOD_CONFIGURE path always clears the WS2812s first; the UAC
       * path never did.  A lit/breathing LED bank draws its PWM current from
       * the same rail as the transducer drivers and shows up as a low-rate
       * amplitude modulation of the 40 kHz carrier. */
      (void)fpga_link_set_ws2812(&fpga_link, 0u, 0u, 0u);
      if (aperture_mod_configure(&aperture_mod, &renderer, &fpga_link,
                                  (const uint8_t *)&config, sizeof(config)) != 0) {
        usb_audio_cancel();
        return;
      }
      if (usb_audio_owns == 0u || usb_audio_inhibited != 0u ||
          umh_usb_audio_debug_inhibit != 0u ||
          umh_usb_audio_generation() != epoch || umh_usb_audio_mute() != 0u) {
        usb_audio_release();
        return;
      }
      if (aperture_mod_start(&aperture_mod) != 0) {
        usb_audio_cancel();
        return;
      }
      ++umh_usb_audio_stats.starts;
      umh_usb_audio_stats.owns_output = 1u;
      system_status_set(UMH_SYSTEM_USB_AUDIO);
    }
    if (output != 0u) {
      if ((uint32_t)usb_audio_env_count + output > sizeof(usb_audio_env_block)) {
        if (aperture_mod_feed(&aperture_mod, usb_audio_env_block,
                              usb_audio_env_count, usb_audio_sequence++) != 0) {
          usb_audio_cancel();
          return;
        }
        usb_audio_env_count = 0u;
      }
      memcpy(&usb_audio_env_block[usb_audio_env_count], envelope, output);
      usb_audio_env_count = (uint16_t)(usb_audio_env_count + output);
      if (usb_audio_env_count >= sizeof(usb_audio_env_block)) {
        if (aperture_mod_feed(&aperture_mod, usb_audio_env_block,
                              usb_audio_env_count, usb_audio_sequence++) != 0) {
          usb_audio_cancel();
          return;
        }
        usb_audio_env_count = 0u;
      }
    }
  }
}

/* Single-channel burst start from silence (UMH_MSG_RAMP_BURST).
 *
 * The array is muted and one element is enabled through the FPGA common-level
 * path: all 84 phases are zero and only the selected staging level byte is a
 * non-zero enable marker, so the event builder substitutes the streamed
 * common level for that one channel and mutes every other.  The ramp itself
 * is played by the FPGA's own level FIFO, which consumes one level per two
 * carrier periods (50 us at 40 kHz), so the MCU never has to time the steps
 * and protocol or USB jitter cannot reach the output envelope.
 *
 * ramp_ticks = 0 plays a single level, i.e. the hard step the block path
 * produces anyway; ramp_ticks = 6 spreads 0 -> level over 300 us.  The last
 * level is held until STOP_PLAN / CLEAR_PLAN, which also drops the FPGA out
 * of common-level mode so the next ordinary FRAME is not rebuilt at the held
 * common amplitude. */
static int ramp_burst_start(uint8_t channel, uint8_t level, uint8_t ramp_ticks)
{
  uint8_t phases[UMH_DEVICE_CHANNEL_COUNT];
  uint8_t enables[UMH_DEVICE_CHANNEL_COUNT];
  uint8_t levels[FPGA_MOD_BLOCK_MAX];
  uint8_t count;
  uint16_t i;
  if (channel >= UMH_DEVICE_CHANNEL_COUNT || level == 0u ||
      ramp_ticks >= FPGA_MOD_BLOCK_MAX) return -1;
  memset(phases, 0, sizeof(phases));
  memset(enables, 0, sizeof(enables));
  enables[channel] = 1u;
  if (ramp_ticks == 0u) {
    levels[0] = level;
    count = 1u;
  } else {
    for (i = 0u; i <= ramp_ticks; ++i)
      levels[i] = (uint8_t)(((uint16_t)level * i + (uint16_t)ramp_ticks / 2u) /
                            (uint16_t)ramp_ticks);
    count = (uint8_t)(ramp_ticks + 1u);
  }
  if (fpga_link_mod_begin(&fpga_link, phases, enables, 0u) != 0) return -2;
  if (fpga_link_mod_feed(&fpga_link, levels, count) < 0) {
    (void)fpga_link_mod_mode(&fpga_link, 0u, 0u);
    (void)fpga_link_safe_stop(&fpga_link);
    return -3;
  }
  return 0;
}

static void protocol_frame_received(const umh_protocol_frame_t *frame, void *context)
{
  umh_status_t status = UMH_STATUS_OK;
  uint8_t status_payload[8];
  uint32_t value;
  uint8_t mod_owns;
  uint8_t motion_owns;
  uint8_t hologram_owns;
  uint8_t gun_owns;
  (void)context;
  if (frame == NULL) return;
  mod_owns = aperture_mod_owns_output(&aperture_mod);
  motion_owns = motion_engine_owns_output(&motion_engine);
  hologram_owns = hologram_engine_owns_output(&hologram_engine);
  gun_owns = gun_trigger_owns_output(&gun_trigger);
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
      if (mod_owns != 0u || motion_owns != 0u || hologram_owns != 0u ||
          gun_owns != 0u) { status = UMH_STATUS_BUSY; break; }
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
        discard_block_stream();
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
        discard_block_stream();
        status = frame->header.stream_sequence != block_stream_expected_sequence ?
                 UMH_STATUS_BAD_SEQUENCE : UMH_STATUS_INVALID_STATE;
      } else {
        ++block_stream_expected_sequence;
        block_stream_active = 0u;
      }
      break;
    case UMH_MSG_BLOCK_CANCEL:
      if (frame->payload_size != 0u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
      discard_block_stream();
      break;
    case UMH_MSG_SET_PLAN:
      if (mod_owns != 0u || motion_owns != 0u || hologram_owns != 0u ||
          gun_owns != 0u) { status = UMH_STATUS_BUSY; break; }
      if (playback_plan.running != 0u ||
          frame->payload_size != sizeof(umh_playback_plan_wire_t) ||
          playback_plan_set(&playback_plan, (const umh_playback_plan_wire_t *)frame->payload) != 0) status = UMH_STATUS_INVALID_STATE;
      break;
    case UMH_MSG_SET_DEMO:
      if (mod_owns != 0u || motion_owns != 0u || hologram_owns != 0u ||
          gun_owns != 0u) { status = UMH_STATUS_BUSY; break; }
      if (frame->payload_size != 1u || frame->payload[0] >= demo_engine_count()) {
        status = UMH_STATUS_BAD_LENGTH;
      } else {
        playback_plan_stop(&playback_plan);
        (void)fpga_link_safe_stop(&fpga_link);
        discard_block_stream();
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
      if (mod_owns != 0u) { status = UMH_STATUS_BUSY; break; }
      if (motion_engine_upload(&motion_engine, frame->payload, frame->payload_size) != 0)
        status = UMH_STATUS_BAD_LENGTH;
      break;
    case UMH_MSG_MOTION_CONFIG:
      if (mod_owns != 0u) { status = UMH_STATUS_BUSY; break; }
      if (motion_engine_configure(&motion_engine, frame->payload, frame->payload_size) != 0)
        status = UMH_STATUS_BAD_LENGTH;
      break;
    case UMH_MSG_MOTION_RATE:
      if (mod_owns != 0u) { status = UMH_STATUS_BUSY; break; }
      if (motion_engine_rate(&motion_engine, frame->payload, frame->payload_size) != 0)
        status = UMH_STATUS_BAD_LENGTH;
      break;
    case UMH_MSG_MOTION_START:
      if (frame->payload_size != 0u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
      if (mod_owns != 0u || device_gui_calibration_busy(&device_gui) != 0u ||
          demo_building != 0u || gun_owns != 0u) { status = UMH_STATUS_BUSY; break; }
      motion_engine_abort(&motion_engine, NULL);
      playback_plan_stop(&playback_plan);
      discard_block_stream();
      if (fpga_link_safe_stop(&fpga_link) != 0) status = UMH_STATUS_IO;
      else { if (motion_engine_uses_rgb(&motion_engine) == 0u) (void)fpga_link_set_ws2812(&fpga_link, 0u, 0u, 0u); if (motion_engine_start(&motion_engine) != 0) status = UMH_STATUS_INVALID_STATE; }
      break;
    case UMH_MSG_MOTION_TARGET:
      if (mod_owns != 0u) { status = UMH_STATUS_BUSY; break; }
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
      if (mod_owns != 0u || motion_owns != 0u || hologram_owns != 0u ||
          gun_owns != 0u) { status = UMH_STATUS_BUSY; break; }
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
      usb_audio_cancel();
      gun_voice_cancel_pending();
      if (mod_owns != 0u) abort_aperture_output();
      if (motion_owns != 0u) motion_engine_request_stop(&motion_engine);
      if (hologram_owns != 0u) hologram_engine_request_stop(&hologram_engine);
      playback_plan_stop(&playback_plan);
      discard_block_stream();
      if (motion_owns == 0u && hologram_owns == 0u) (void)fpga_link_safe_stop(&fpga_link);
      break;
    case UMH_MSG_CLEAR_PLAN:
      if (frame->payload_size != 0u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
      usb_audio_cancel();
      gun_voice_cancel_pending();
      if (mod_owns != 0u) abort_aperture_output();
      if (motion_owns != 0u) motion_engine_abort(&motion_engine, &fpga_link);
      if (hologram_owns != 0u) hologram_engine_abort(&hologram_engine, &fpga_link);
      playback_plan_stop(&playback_plan);
      (void)fpga_link_safe_stop(&fpga_link);
      playback_plan_clear(&playback_plan);
      discard_block_stream();
      break;
    case UMH_MSG_HOLOGRAM_UPLOAD:
      if (mod_owns != 0u || motion_owns != 0u) { status = UMH_STATUS_BUSY; break; }
      if (hologram_engine_upload(&hologram_engine, frame->payload,
                                 frame->payload_size) != 0)
        status = UMH_STATUS_BAD_LENGTH;
      break;
    case UMH_MSG_HOLOGRAM_CONFIG:
      if (mod_owns != 0u || motion_owns != 0u) { status = UMH_STATUS_BUSY; break; }
      if (hologram_engine_configure(&hologram_engine, frame->payload,
                                    frame->payload_size) != 0)
        status = UMH_STATUS_BAD_LENGTH;
      break;
    case UMH_MSG_HOLOGRAM_START:
      if (frame->payload_size != 0u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
      if (mod_owns != 0u || motion_owns != 0u || device_gui_calibration_busy(&device_gui) != 0u ||
          demo_building != 0u || gun_owns != 0u) { status = UMH_STATUS_BUSY; break; }
      /* Same hand-over protocol as MOTION_START: give up the block path, put the
       * FPGA in a safe state, then own the output. */
      playback_plan_stop(&playback_plan);
      discard_block_stream();
      if (fpga_link_safe_stop(&fpga_link) != 0) status = UMH_STATUS_IO;
      else {
        (void)fpga_link_set_ws2812(&fpga_link, 0u, 0u, 0u);
        hologram_engine_start(&hologram_engine);
        if (hologram_engine_owns_output(&hologram_engine) == 0u)
          status = UMH_STATUS_INVALID_STATE;
      }
      break;
    case UMH_MSG_HOLOGRAM_STOP:
      if (frame->payload_size != 0u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
      hologram_engine_request_stop(&hologram_engine);
      break;
    case UMH_MSG_HOLOGRAM_STATUS:
      if (frame->payload_size != 0u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
      {
        umh_hologram_status_wire_t hologram_status;
        hologram_engine_get_status(&hologram_engine, &hologram_status);
        send_response(frame, UMH_MSG_HOLOGRAM_STATUS, &hologram_status,
                      (uint16_t)sizeof(hologram_status));
        return;
      }
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
        wire.reserved = system_status_get()->cal_state;        /* device_gui_cal_state_t */
        wire.block_count = calibration_result.patterns_used;
        wire.rms_before_deg = calibration_result.rms_before_deg;
        wire.rms_after_deg = calibration_result.rms_after_deg;
        wire.mic_consistency_deg = calibration_result.mic_consistency_deg;
        wire.residual = calibration_result.drift_deg;          /* saturated blocks */
        wire.distance_m = calibration_result.table_mm * 0.001f;
        wire.tilt_x_deg = calibration_result.tilt_x_deg;
        wire.tilt_y_deg = calibration_result.tilt_y_deg;
        wire.echo_ratio = calibration_result.echo_mag;
        wire.mic_ratio = calibration_result.coupling_db;       /* p90/p10 amplitude, dB */
        wire.verify_gain_db = calibration_result.verify_gain_db;
        wire.correction_rms_deg = calibration_result.correction_rms_deg;
        wire.amp_p10 = calibration_result.amp_p10;
        wire.amp_p90 = calibration_result.amp_p90;
        wire.pairs_used = calibration_result.pairs_used;
        wire.channels_covered = calibration_result.channels_covered;
        wire.dead_count = calibration_result.dead_count;
        wire.verify_wins = calibration_result.sign_hypothesis;
        wire.verify_sets = (uint8_t)calibration_result.reserved1;
        wire.committed = calibration_committed;
        wire.method = calibration_result.geom_hypothesis;
        memcpy(wire.amplitude, calibration_result.amplitude, UMH_DEVICE_CHANNEL_COUNT);
        memcpy(wire.coverage, calibration_result.coverage, UMH_DEVICE_CHANNEL_COUNT);
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
      if (mod_owns != 0u || motion_owns != 0u || hologram_owns != 0u ||
          gun_owns != 0u) { status = UMH_STATUS_BUSY; break; }
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
      if (mod_owns != 0u || motion_owns != 0u || hologram_owns != 0u ||
          gun_owns != 0u) { status = UMH_STATUS_BUSY; break; }
      if (calibration_task_handle == NULL) status = UMH_STATUS_INVALID_STATE;
      else if (calibration_raw_mode != 0u ||
               device_gui_calibration_busy(&device_gui) != 0u) status = UMH_STATUS_BUSY;
      else {
        device_gui_calibration_begin(&device_gui);
        calibration_result.progress = 0u;
        calibration_committed = 0u;
        system_status_get()->cal_state = DEVICE_GUI_CAL_WAIT;
        (void)xTaskNotifyGive((TaskHandle_t)calibration_task_handle);
        status = UMH_STATUS_OK;
      }
      break;
    case UMH_MSG_CAL_SELFTEST:
      if (frame->payload_size != 0u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
      if (mod_owns != 0u || motion_owns != 0u || hologram_owns != 0u ||
          gun_owns != 0u) { status = UMH_STATUS_BUSY; break; }
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
    case UMH_MSG_CAL_PROBE:
      /* Asynchronous: no ACK.  The calibration task replies with a
       * CAL_PROBE frame carrying the same transaction id. */
      if (frame->payload_size != sizeof(umh_cal_probe_request_t)) {
        send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return;
      }
      if (mod_owns != 0u || motion_owns != 0u || hologram_owns != 0u ||
          gun_owns != 0u ||
          calibration_task_handle == NULL || calibration_probe_mode != 0u ||
          calibration_raw_mode != 0u || calibration_selftest_mode != 0u ||
          device_gui_calibration_busy(&device_gui) != 0u) {
        send_result(frame, UMH_STATUS_BUSY, NULL, 0u); return;
      }
      memcpy(&calibration_probe_request, frame->payload, sizeof(calibration_probe_request));
      if (calibration_probe_request.gate_count == 0u ||
          calibration_probe_request.gate_count > FPGA_MIC_MAX_GATES ||
          calibration_probe_request.gate_width == 0u ||
          calibration_probe_request.gate_step < calibration_probe_request.gate_width ||
          calibration_probe_request.burst_us == 0u) {
        send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return;
      }
      calibration_probe_tid = frame->header.transaction_id;
      calibration_probe_mode = 1u;
      (void)xTaskNotifyGive((TaskHandle_t)calibration_task_handle);
      return;
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
    case UMH_MSG_LIB_BEGIN:
    case UMH_MSG_LIB_DATA:
    case UMH_MSG_LIB_END:
    case UMH_MSG_LIB_STATUS:
    case UMH_MSG_LIB_START:
    case UMH_MSG_LIB_STOP:
      if (queue_storage_request(frame) == 0) return;
      status = UMH_STATUS_BUSY;
      break;
    case UMH_MSG_MOD_CONFIGURE:
      if (frame->payload_size < sizeof(umh_mod_config_wire_t) ||
          frame->payload_size > (uint16_t)(sizeof(umh_mod_config_wire_t) + 1u +
                                           UMH_MOD_MAX_EXTRA_POINTS * sizeof(umh_mod_point_wire_t))) {
        status = UMH_STATUS_BAD_LENGTH;
        break;
      }
      if (device_gui_calibration_busy(&device_gui) != 0u || demo_building != 0u ||
          motion_owns != 0u || gun_owns != 0u) {
        status = UMH_STATUS_BUSY;
        break;
      }
      if (playback_plan.running != 0u || block_stream_active != 0u ||
          block_parser.block.active != 0u || frame_ring_count(&frame_ring) != 0u) {
        status = UMH_STATUS_BUSY;
        break;
      }
      if (aperture_mod_owns_output(&aperture_mod) != 0u)
        abort_aperture_output();
      (void)fpga_link_set_ws2812(&fpga_link, 0u, 0u, 0u); if (aperture_mod_configure(&aperture_mod, &renderer, &fpga_link,
                                 frame->payload, frame->payload_size) != 0)
        status = UMH_STATUS_INVALID_STATE;
      break;
    case UMH_MSG_MOD_START:
      if (frame->payload_size != 0u) { status = UMH_STATUS_BAD_LENGTH; break; }
      if (usb_audio_owns != 0u) { status = UMH_STATUS_BUSY; break; }
      if (aperture_mod_start(&aperture_mod) != 0) status = UMH_STATUS_INVALID_STATE;
      break;
    case UMH_MSG_MOD_FEED:
      if (frame->payload_size == 0u || usb_audio_owns != 0u) return;
      (void)aperture_mod_feed(&aperture_mod, frame->payload, frame->payload_size,
                              frame->header.stream_sequence);
      return;
    case UMH_MSG_MOD_STOP:
      if (frame->payload_size != 0u) { status = UMH_STATUS_BAD_LENGTH; break; }
      usb_audio_cancel();
      gun_voice_cancel_pending();
      aperture_mod_request_stop(&aperture_mod);
      system_status_clear(UMH_SYSTEM_UNDERRUN);
      break;
    case UMH_MSG_MOD_STATUS:
      if (frame->payload_size != 0u) { status = UMH_STATUS_BAD_LENGTH; break; }
      {
        umh_mod_status_wire_t mod_status;
        aperture_mod_get_status(&aperture_mod, &mod_status);
        send_response(frame, UMH_MSG_MOD_STATUS, &mod_status,
                      (uint16_t)sizeof(mod_status));
        return;
      }
    case UMH_MSG_RAMP_BURST:
      if (frame->payload_size != 4u) { status = UMH_STATUS_BAD_LENGTH; break; }
      /* A burst owns the output the way the aperture stream does, so every
       * other producer has to be idle or explicitly given up below. */
      if (device_gui_calibration_busy(&device_gui) != 0u || demo_building != 0u ||
          motion_owns != 0u || gun_owns != 0u || hologram_owns != 0u ||
          playback_plan.running != 0u || block_stream_active != 0u ||
          block_parser.block.active != 0u || frame_ring_count(&frame_ring) != 0u) {
        status = UMH_STATUS_BUSY;
        break;
      }
      {
        int burst = ramp_burst_start(frame->payload[0], frame->payload[1],
                                     frame->payload[2]);
        if (burst == -1) status = UMH_STATUS_BAD_LENGTH;
        else if (burst != 0) status = UMH_STATUS_IO;
      }
      break;
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
    uint8_t gun_owns_before;
    count = umh_rx_ring_read(&umh_usb_rx_ring, data, sizeof(data));
    if (count != 0u) umh_protocol_parser_feed(&protocol_parser, data, count);
    else if (umh_usb_audio_stats.queued_samples == 0u)
      (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(
          umh_usb_audio_streaming() != 0u ? 2u : 10u));
    /* Hardware fire button.  Sampling happens in the input task; the edges are
     * handled here, next to every other output takeover, and the 10 ms notify
     * timeout above bounds the press-to-emission latency when USB is idle. */
    gun_owns_before = gun_trigger_owns_output(&gun_trigger);
    gun_trigger_poll();
    if (count != 0u || gun_trigger_owns_output(&gun_trigger) != gun_owns_before)
      render_wake();
    usb_audio_service();
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
    /* A 1 us request would program ARR = 0, which blocks the STM32 basic
     * timer: no update event, so the task only woke on the 2 ms notify
 * timeout.  At 20 kHz that stalled the envelope stream for 1-2 ms and then
     * replayed the backlog in a burst (measured p99 lag 1.8 ms), heard as
     * noise and smeared/slowed sound.  Spin out spans too short to be worth
     * a context switch instead. */
    if (delta < 4u) continue;
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

/* Render-path cycle profile, readable from a debugger as an untyped word array
 * so no DWARF type lookup is involved.
 *   [0] frames submitted          [1] sum loop-entry -> submit-entry cycles
 *   [2] max of [1]                [3] sum cycles inside fpga_link_submit
 *   [4] max of [3]                [5] sum whole-iteration cycles
 *   [6] max of [5]                [7] credit-wait retries */
volatile uint32_t umh_render_profile[8];
volatile uint32_t umh_render_burst_yields;

static void render_task(void *argument)
{
  umh_output_frame_t *frame;
  uint64_t last_time_us = system_time_us();
  uint64_t now_us;
  uint64_t elapsed_us;
  uint16_t previous_fpga_flags = 0u;
  uint8_t render_burst = 0u;
  uint32_t render_idle_poll_ms = 0u;
  (void)argument;
  render_task_handle = xTaskGetCurrentTaskHandle();
  for (;;) {
    uint32_t loop_start_cycles = DWT->CYCCNT;
    /* Voice capture and playback are serviced below the static gun branch. */
    /* A held PA9 shot is one static image the FPGA already has: nothing is
     * queued and no other producer may touch the channels.  Only the link needs
     * servicing, so this branch keeps the SPI traffic (and with it the FPGA link
     * watchdog) alive, advances the fade envelope, and re-sends the image if the
     * output was stopped underneath us, for instance by a host STOP_PLAN.
     * The gate is holds_image(), not owns_output(): the takeover claims the
     * output before it has rebuilt the frame, and that window must not be
     * served with the previous producer's image. */
    if (gun_trigger_holds_image(&gun_trigger) != 0u) {
      (void)gun_trigger_refresh(&gun_trigger, &renderer, &fpga_link);
      /* This branch is the authoritative writer of the emitting flag while the
       * shot is held, exactly like the motion and hologram branches: the press
       * edge can arrive between two render iterations, and the host and the OLED
       * must see "emitting" for as long as the switch is down. */
      system_status_set(UMH_SYSTEM_PLAYING);
      system_status_clear(UMH_SYSTEM_UNDERRUN);
      now_us = system_time_us();
      system_status_get()->device_time = (uint32_t)now_us;
      system_status_get()->frame_count = frame_ring_count(&frame_ring);
      system_status_get()->frame_free = frame_ring_free(&frame_ring);
      system_status_get()->fpga_credit = fpga_link_status(&fpga_link)->fifo_credit;
      system_status_get()->fpga_depth = fpga_link_status(&fpga_link)->fifo_depth;
      last_time_us = now_us;
      render_burst = 0u;
      osDelay(1u);
      continue;
    }
    /* Capture drains up to two 16-sample batches per tick.  Playback feeds
     * before aperture service and also runs after that module releases output,
     * so an ordinary end or link abort both clear the gun ownership. */
    if (gun_voice_state == GUN_VOICE_RECORDING || gun_voice_state == GUN_VOICE_FULL)
      gun_voice_capture_service();
    if (demo_building != 0u) {
      render_burst = 0u;
      osDelay(1u);
      continue;
    }
    if (gun_voice_state == GUN_VOICE_READY) {
      int prepared = gun_voice_prepare_service();
      if (gun_voice_state == GUN_VOICE_READY && prepared > 0)
        (void)gun_voice_start_playback();
      else if (gun_voice_state == GUN_VOICE_READY) {
        osDelay(1u);
        continue;
      }
    }
    if (gun_voice_state == GUN_VOICE_PLAYBACK || gun_voice_state == GUN_VOICE_DRAINING)
      gun_voice_playback_service();
    if (aperture_mod_owns_output(&aperture_mod) != 0u) {
      /* The FPGA level FIFO paces the 20 kHz output from its own clock; one
       * service per millisecond keeps it topped up, so task or USB jitter
       * never reaches the envelope.  System flags only change on edges. */
      uint32_t mod_flags;
      aperture_mod_service(&aperture_mod);
      mod_flags = (aperture_mod.state == UMH_MOD_RUNNING ? UMH_SYSTEM_PLAYING : 0u) |
                    ((aperture_mod.underrun_count != 0u &&
                      aperture_mod.state != UMH_MOD_OFF) ? UMH_SYSTEM_UNDERRUN : 0u);
      if ((system_status_get()->flags & (UMH_SYSTEM_PLAYING | UMH_SYSTEM_UNDERRUN)) != mod_flags)
        system_status_modify(UMH_SYSTEM_PLAYING | UMH_SYSTEM_UNDERRUN, mod_flags);
      now_us = system_time_us();
      system_status_get()->device_time = (uint32_t)now_us;
      system_status_get()->frame_count = 0u;
      system_status_get()->frame_free = 0u;
      last_time_us = now_us;
      render_burst = 0u;
      osDelay(1u);
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
    /* Hologram keyframe executor.  It owns the output the same way the motion
     * engine does, but it never touches the frame ring: it holds its own
     * 84-channel keyframes and interpolates them here, in the render task. */
    if (hologram_engine_owns_output(&hologram_engine) != 0u) {
      uint32_t hologram_wait_us = 1000u;
      now_us = system_time_us();
      (void)hologram_engine_service(&hologram_engine, &fpga_link, now_us,
                                    &hologram_wait_us);
      if (hologram_engine_owns_output(&hologram_engine) != 0u) {
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
      if (hologram_wait_us == 0u) hologram_wait_us = 250u;
      if (hologram_wait_us > 50000u) hologram_wait_us = 50000u;
      render_wait_until_us(now_us + (uint64_t)hologram_wait_us);
      last_time_us = system_time_us();
      render_burst = 0u;
      continue;
    }
    if (flash_play_active != 0u &&
        frame_ring_count(&frame_ring) < UMH_FLASH_PLAY_HIGH_WATER &&
        storage_wake_sem != NULL) {
      (void)xSemaphoreGive(storage_wake_sem);
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
          uint32_t t_before = DWT->CYCCNT;
          uint32_t t_pre = t_before - loop_start_cycles;
          int submit_result = fpga_link_submit_allow_hold(&fpga_link, frame, 1u);
          uint32_t t_submit = DWT->CYCCNT - t_before;
          ++umh_render_profile[0];
          umh_render_profile[1] += t_pre;
          if (t_pre > umh_render_profile[2]) umh_render_profile[2] = t_pre;
          umh_render_profile[3] += t_submit;
          if (t_submit > umh_render_profile[4]) umh_render_profile[4] = t_submit;
          umh_render_profile[5] += t_pre + t_submit;
          if ((t_pre + t_submit) > umh_render_profile[6])
            umh_render_profile[6] = t_pre + t_submit;
          if (submit_result == 0) {
            (void)frame_ring_release_read(&frame_ring);
            if (flash_play_active != 0u &&
                frame_ring_count(&frame_ring) < UMH_FLASH_PLAY_HIGH_WATER &&
                storage_wake_sem != NULL)
              (void)xSemaphoreGive(storage_wake_sem);
            playback_plan_frame_submitted(&playback_plan);
            if (++render_burst >= 16u) {
              render_burst = 0u;
              ++umh_render_burst_yields;
              taskYIELD();
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
          ++umh_render_profile[7];
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
      uint32_t poll_ms = HAL_GetTick();
      if ((uint32_t)(poll_ms - render_idle_poll_ms) >= 8u) {
        render_idle_poll_ms = poll_ms;
        (void)fpga_link_poll_status(&fpga_link);
      }
      render_waiting_for_work = 1u;
      if (frame_ring_count(&frame_ring) != 0u) {
        render_waiting_for_work = 0u;
        continue;
      }
      if (flash_play_active != 0u) {
        render_wait_until_us(system_time_us() + 200u);
      } else if (render_task_handle != NULL) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(8u));
      } else {
        osDelay(8u);
      }
      render_waiting_for_work = 0u;
    }
    /* Voice playback leaves its own PLAYING flag in place; this tail belongs to
     * the frame-plan path, and a voice shot that has not drained yet must not
     * have its flag cleared underneath it by a plan that is not running. */
    if (gun_voice_state == GUN_VOICE_PLAYBACK || gun_voice_state == GUN_VOICE_DRAINING) {
      /* serviced in the aperture branch above */
    } else if (playback_plan.running != 0u) system_status_set(UMH_SYSTEM_PLAYING);
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
  uint8_t flash_read_waited = 0u;
  (void)argument;
  storage_task_handle = xTaskGetCurrentTaskHandle();
  for (;;) {
    if (storage_wake_sem != NULL)
      (void)xSemaphoreTake(storage_wake_sem, portMAX_DELAY);
    else
      (void)xSemaphoreTake(storage_ready_sem, portMAX_DELAY);
    for (;;) {
      if (storage_ready_sem != NULL &&
          xSemaphoreTake(storage_ready_sem, 0u) == pdTRUE) {
        storage_process(&storage_request_buffer);
        (void)xSemaphoreGive(storage_free_sem);
        continue;
      }
      if (flash_play_stop_request != 0u) {
        flash_library_reader_close(&flash_reader);
        flash_nor_dma_drain();
        flash_read_waited = 0u;
        flash_play_active = 0u;
        flash_play_stop_request = 0u;
        continue;
      }
      if (flash_play_active == 0u ||
          frame_ring_count(&frame_ring) >= UMH_FLASH_PLAY_HIGH_WATER)
        break;
      if (flash_reader.prefetch_in_flight != 0u && flash_read_waited == 0u) {
        if (flash_nor_wait_dma(20u) != FLASH_NOR_OK) {
          flash_library_reader_close(&flash_reader);
          flash_nor_dma_drain();
          flash_read_waited = 0u;
          flash_play_active = 0u;
          flash_play_stop_request = 0u;
          playback_plan_stop(&playback_plan);
          (void)fpga_link_safe_stop(&fpga_link);
          system_status_fault(UMH_FAULT_FLASH_IO, 3u, UMH_FAULT_CRITICAL);
          continue;
        }
        flash_read_waited = 1u;
        continue;
      }
      flash_read_waited = 0u;
      flash_nor_dma_drain();
      {
        int pump_result = flash_library_reader_pump(&flash_reader,
                                                    flash_library_ring_sink,
                                                    &flash_ring_sink, 4u);
        if (pump_result < 0) {
          flash_library_reader_close(&flash_reader);
          flash_nor_dma_drain();
          flash_play_active = 0u;
          flash_play_stop_request = 0u;
          playback_plan_stop(&playback_plan);
          (void)fpga_link_safe_stop(&fpga_link);
          system_status_fault(UMH_FAULT_FLASH_IO, (uint32_t)(-pump_result),
                              UMH_FAULT_CRITICAL);
          continue;
        }
        if (pump_result == 0) {
          if (flash_library_reader_done(&flash_reader) != 0u) {
            if (frame_ring_count(&frame_ring) == 0u) {
              flash_library_reader_close(&flash_reader);
              flash_nor_dma_drain();
              flash_play_active = 0u;
              flash_play_stop_request = 0u;
              playback_plan_stop(&playback_plan);
              (void)fpga_link_safe_stop(&fpga_link);
            }
            break;
          }
          if (flash_reader.prefetch_in_flight == 0u) break;
        }
        continue;
      }
    }
  }
}

static void input_task(void *argument)
{
  TickType_t last_wake = xTaskGetTickCount();
  (void)argument;
  for (;;) {
    input_events_sample();
    gun_trigger_sample(&gun_trigger);
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10u));
  }
}

static void ui_task(void *argument)
{
  input_event_t event;
  (void)argument;
  for (;;) {
    while (input_events_poll(&event) != 0u) {
      device_gui_handle_event(&device_gui, &event);
    }
    device_gui_render(&device_gui, HAL_GetTick());
    /* Keep the cursor animation responsive while input scanning remains
     * independent at 10 ms. */
    osDelay(30u);
  }
}

/* The OLED is pushed to the panel from its own task, deliberately not from
 * ui_task.
 *
 * A page transfer is a blocking I2C call, and a panel that stops acknowledging
 * costs one full HAL timeout per page -- up to ~800 ms per refresh.  While the
 * refresh ran in ui_task, that was also 800 ms in which no key event was
 * dispatched: the gun menu's long press would be detected by the input task and
 * queued, but the UI would not act on it, which reads as "the long press does
 * nothing".  Splitting them means a slow or dead display can only make the
 * display lag; keys keep working.
 *
 * oled_ssd1315_refresh() also backs off after repeated failures, and the I2C bus
 * is shared with the EEPROM under its own mutex, so this task cannot corrupt
 * another master's transaction. */
static void display_task(void *argument)
{
  uint32_t last_init_attempt = HAL_GetTick();
  (void)argument;
  for (;;) {
    if (oled.initialized == 0u) {
      if ((HAL_GetTick() - last_init_attempt) >= 1000u) {
        last_init_attempt = HAL_GetTick();
        oled_ssd1315_init(&oled, &hi2c1);
      }
    } else if (oled_ssd1315_refresh(&oled) != 0) {
      oled.initialized = 0u;
      last_init_attempt = HAL_GetTick();
      /* A display/I2C fault must not disable the FPGA output path.  Keep it
       * visible in diagnostics while allowing the render task to continue. */
      system_status_fault(UMH_FAULT_HAL_INIT, 1u, UMH_FAULT_WARNING);
    }
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
    /* One scheduler snapshot per heartbeat.  The CPU shares the SYSTEM page
     * shows and the ones a debugger reads come from this single 4 Hz sample,
     * so the render path never walks the task list. */
    umh_task_stats_update();
    osDelay(250u);
  }
}

void MX_FREERTOS_Init(void)
{
  umh_protocol_task_handle = osThreadNew(protocol_task, NULL, &protocol_task_attributes);
}

static void application_init(void)
{
  __HAL_RCC_CCM_CLK_SLEEP_ENABLE();
  umh_rx_ring_init(&umh_usb_rx_ring);
  device_profile_init(&device_profile);
  system_status_init();
  umh_task_stats_init();
  i2c_bus_init();
  frame_ring_init(&frame_ring);
  spatial_renderer_init(&renderer, &device_profile);
  block_parser_init(&block_parser, &renderer, &frame_ring);
  aperture_mod_init(&aperture_mod);
  motion_engine_init(&motion_engine);
  hologram_engine_init(&hologram_engine);
  gun_trigger_init(&gun_trigger);
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
  flash_library_init(&flash_library, &hspi3);
  flash_ring_sink.ring = &frame_ring;
  flash_store_init(&flash_store);
  {
    uint8_t flash_id[3];
    if (flash_nor_read_jedec(flash_id) == FLASH_NOR_OK &&
        flash_nor_is_present() != 0u && flash_store_mount(&flash_store) == 0)
      system_status_set(UMH_SYSTEM_FLASH_READY);
    else
      system_status_fault(UMH_FAULT_FLASH_IO, 1u, UMH_FAULT_WARNING);
  }
  if (flash_library_mount(&flash_library) != FLASH_LIBRARY_OK)
    system_status_fault(UMH_FAULT_FLASH_IO, 2u, UMH_FAULT_WARNING);
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
                  gui_calibration, gui_selftest, gui_demo,
                  gui_ws2812_set,
                  gui_gun_select, gui_gun_exit,
                  demo_engine_count(), NULL);
  {
    storage_free_sem = xSemaphoreCreateBinaryStatic(&storage_free_mem);
    storage_ready_sem = xSemaphoreCreateBinaryStatic(&storage_ready_mem);
    storage_wake_sem = xSemaphoreCreateBinaryStatic(&storage_wake_mem);
    if (storage_free_sem != NULL) (void)xSemaphoreGive(storage_free_sem);
  }
  umh_protocol_parser_init(&protocol_parser, protocol_frame_received, NULL);
  render_timer_init();
  (void)osThreadNew(render_task, NULL, &render_task_attributes);
  (void)osThreadNew(storage_task, NULL, &storage_task_attributes);
  (void)osThreadNew(ui_task, NULL, &ui_task_attributes);
  (void)osThreadNew(display_task, NULL, &display_task_attributes);
  (void)osThreadNew(input_task, NULL, &input_task_attributes);
  (void)osThreadNew(health_task, NULL, &health_task_attributes);
  calibration_task_handle = osThreadNew(calibration_task, NULL, &calibration_task_attributes);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == fpga_link.spi) fpga_link_spi_txrx_complete(&fpga_link);
  else flash_nor_dma_complete(hspi);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi != fpga_link.spi) flash_nor_dma_complete(hspi);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == fpga_link.spi) {
    fpga_link_spi_error(&fpga_link);
    system_status_get()->fpga_errors++;
    system_status_fault(UMH_FAULT_FPGA_SPI_DMA, HAL_SPI_GetError(hspi), UMH_FAULT_CRITICAL);
  } else {
    flash_nor_dma_error(hspi);
  }
}
