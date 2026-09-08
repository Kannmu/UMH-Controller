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
#include "spi.h"
#include "i2c.h"
#include "i2c_bus.h"
#include <string.h>

osThreadId_t umh_protocol_task_handle;
umh_rx_ring_t umh_usb_rx_ring;

static umh_device_profile_t device_profile;
static umh_spatial_renderer_t renderer;
static umh_frame_ring_t frame_ring;
static umh_block_parser_t block_parser;
static umh_playback_plan_t playback_plan;
static fpga_link_t fpga_link;
static flash_store_t flash_store;
static eeprom_profile_t eeprom_profile;
static oled_ssd1315_t oled;
static umh_protocol_parser_t protocol_parser;
static device_gui_t device_gui;

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
static uint8_t block_stream_active;
static uint32_t block_stream_expected_sequence;

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
static StackType_t protocol_task_stack[1024], render_task_stack[512], storage_task_stack[512], ui_task_stack[384], health_task_stack[256];
static const osThreadAttr_t protocol_task_attributes = {
  .name = "protocol", .cb_mem = &protocol_task_cb, .cb_size = sizeof(protocol_task_cb),
  .stack_mem = protocol_task_stack, .stack_size = sizeof(protocol_task_stack),
  .priority = osPriorityHigh
};
static const osThreadAttr_t render_task_attributes = {
  .name = "render", .cb_mem = &render_task_cb, .cb_size = sizeof(render_task_cb),
  .stack_mem = render_task_stack, .stack_size = sizeof(render_task_stack),
  .priority = osPriorityAboveNormal
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
static const osThreadAttr_t health_task_attributes = {
  .name = "health", .cb_mem = &health_task_cb, .cb_size = sizeof(health_task_cb),
  .stack_mem = health_task_stack, .stack_size = sizeof(health_task_stack),
  .priority = osPriorityLow
};

static void send_response(const umh_protocol_frame_t *request, uint8_t type,
                          const void *payload, uint16_t length);
static void application_init(void);

static uint32_t read_u32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int gui_start(void *context)
{
  (void)context;
  if (playback_plan.configured == 0u || frame_ring_count(&frame_ring) == 0u ||
      playback_plan.wire.block_id != block_parser.block.header.block_id) {
    system_status_fault(UMH_FAULT_PLAN_START, 1u, UMH_FAULT_CRITICAL);
    return -1;
  }
  if (playback_plan_start(&playback_plan, system_time_us(),
                          frame_ring_count(&frame_ring),
                          block_parser.block.header.start_time) != 0)
  { system_status_fault(UMH_FAULT_PLAN_START, 2u, UMH_FAULT_CRITICAL); return -1; }
  return 0;
}

static int gui_stop(void *context)
{
  int result;
  (void)context;
  playback_plan_stop(&playback_plan);
  result = fpga_link_safe_stop(&fpga_link);
  return result;
}

static int gui_clear(void *context)
{
  int result;
  (void)context;
  playback_plan_stop(&playback_plan);
  result = fpga_link_safe_stop(&fpga_link);
  playback_plan_clear(&playback_plan);
  block_parser_cancel(&block_parser);
  block_stream_active = 0u;
  frame_ring_init(&frame_ring);
  return result;
}

static int gui_trigger(void *context)
{
  (void)context;
  playback_plan_notify_trigger(&playback_plan);
  return 0;
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
  (void)context;
  if (frame == NULL) return;
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
      if (playback_plan.running != 0u ||
          frame->payload_size != sizeof(umh_playback_plan_wire_t) ||
          playback_plan_set(&playback_plan, (const umh_playback_plan_wire_t *)frame->payload) != 0) status = UMH_STATUS_INVALID_STATE;
      break;
    case UMH_MSG_START_PLAN:
      if (frame->payload_size != 0u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
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
      playback_plan_stop(&playback_plan);
      (void)fpga_link_safe_stop(&fpga_link);
      break;
    case UMH_MSG_CLEAR_PLAN:
      if (frame->payload_size != 0u) { send_result(frame, UMH_STATUS_BAD_LENGTH, NULL, 0u); return; }
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
      else eeprom_pending_valid = 0u;
      break;
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

static void render_task(void *argument)
{
  umh_output_frame_t *frame;
  uint64_t last_time_us = system_time_us();
  uint64_t now_us;
  uint64_t elapsed_us;
  uint16_t previous_fpga_flags = 0u;
  (void)argument;
  for (;;) {
    now_us = system_time_us();
    elapsed_us = now_us - last_time_us;
    if (elapsed_us > UINT32_MAX) elapsed_us = UINT32_MAX;
    playback_plan_tick(&playback_plan, now_us, (uint32_t)elapsed_us,
                       frame_ring_count(&frame_ring));
    last_time_us = now_us;
    frame = frame_ring_peek_read(&frame_ring);
    if (frame != NULL && playback_plan_frame_due(&playback_plan, frame->deadline, now_us) != 0u &&
        fpga_link_status(&fpga_link)->fifo_credit != 0u) {
      int submit_result = fpga_link_submit(&fpga_link, frame);
      if (submit_result == 0) {
        (void)frame_ring_release_read(&frame_ring);
        playback_plan_frame_submitted(&playback_plan);
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
      if (fpga_flags != 0u) system_status_set(UMH_SYSTEM_ERROR);
      if (fpga_flags != 0u && fpga_flags != previous_fpga_flags) {
        umh_fault_code_t code = (fpga_flags & FPGA_STATUS_OUTPUT_FAULT) != 0u ? UMH_FAULT_FPGA_OUTPUT :
                                 (fpga_flags & FPGA_STATUS_INVALID_FRAME) != 0u ? UMH_FAULT_FPGA_INVALID_FRAME :
                                 (fpga_flags & FPGA_STATUS_OVERFLOW) != 0u ? UMH_FAULT_FPGA_OVERFLOW :
                                 UMH_FAULT_FPGA_UNDERRUN;
        system_status_fault(code, fpga_flags, UMH_FAULT_CRITICAL);
      }
      previous_fpga_flags = fpga_flags;
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

static void ui_task(void *argument)
{
  input_event_t event;
  (void)argument;
  for (;;) {
    input_events_sample();
    while (input_events_poll(&event) != 0u) {
      device_gui_handle_event(&device_gui, &event);
    }
    device_gui_render(&device_gui, HAL_GetTick());
    (void)oled_ssd1315_refresh(&oled);
    osDelay(100u);
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
  playback_plan_clear(&playback_plan);
  fpga_link_init(&fpga_link, &hspi1);
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
  if (eeprom_profile_load(&eeprom_profile) == 0) {
    umh_channel_calibration_t calibration[UMH_DEVICE_CHANNEL_COUNT];
    uint16_t i;
    const eeprom_profile_record_t *record = eeprom_profile_current(&eeprom_profile);
    for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) {
      calibration[i].phase = record->phase[i];
      calibration[i].gain = record->gain[i];
      calibration[i].enabled = (uint8_t)((record->enabled[i / 8u] >> (i % 8u)) & 1u);
    }
    spatial_renderer_set_calibration(&renderer, calibration, UMH_DEVICE_CHANNEL_COUNT);
    spatial_renderer_set_rgb_calibration(&renderer, record->rgb_gain);
    device_profile_set_calibration_generation(&device_profile,
                                              record->version,
                                              record->generation);
    system_status_set(UMH_SYSTEM_CALIBRATION_VALID);
  } else system_status_fault(UMH_FAULT_EEPROM_IO, 1u, UMH_FAULT_WARNING);
  oled_ssd1315_init(&oled, &hi2c1);
  if (oled.initialized == 0u) system_status_fault(UMH_FAULT_HAL_INIT, 1u, UMH_FAULT_CRITICAL);
  input_events_init();
  device_gui_init(&device_gui, &oled, &device_profile, system_status_get(),
                  &playback_plan, &fpga_link, &flash_store, &eeprom_profile,
                  gui_start, gui_stop, gui_clear, gui_trigger, NULL);
  {
    const osMessageQueueAttr_t storage_queue_attributes = {
      .name = "storage", .cb_mem = &storage_queue_cb, .cb_size = sizeof(storage_queue_cb),
      .mq_mem = storage_queue_memory, .mq_size = sizeof(storage_queue_memory)
    };
    storage_queue = osMessageQueueNew(2u, sizeof(storage_request_t), &storage_queue_attributes);
  }
  umh_protocol_parser_init(&protocol_parser, protocol_frame_received, NULL);
  (void)osThreadNew(render_task, NULL, &render_task_attributes);
  (void)osThreadNew(storage_task, NULL, &storage_task_attributes);
  (void)osThreadNew(ui_task, NULL, &ui_task_attributes);
  (void)osThreadNew(health_task, NULL, &health_task_attributes);
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
