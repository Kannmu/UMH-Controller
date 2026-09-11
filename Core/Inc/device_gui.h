#ifndef DEVICE_GUI_H
#define DEVICE_GUI_H

#include <stdint.h>
#include "oled_ssd1315.h"
#include "device_profile.h"
#include "system_status.h"
#include "playback_plan.h"
#include "fpga_link.h"
#include "flash_store.h"
#include "eeprom_profile.h"
#include "input_events.h"

typedef enum {
  DEVICE_GUI_HOME = 0u,
  DEVICE_GUI_PLAYBACK,
  DEVICE_GUI_DEVICE,
  DEVICE_GUI_CALIBRATION,
  DEVICE_GUI_STORAGE,
  DEVICE_GUI_DIAGNOSTICS,
  DEVICE_GUI_SYSTEM,
  DEVICE_GUI_DEMOS,
  DEVICE_GUI_CONTROL,
  DEVICE_GUI_WS2812_TEST,
  DEVICE_GUI_PAGE_COUNT
} device_gui_page_t;

typedef int (*device_gui_action_t)(void *context);

typedef struct {
  oled_ssd1315_t *oled;
  const umh_device_profile_t *profile;
  umh_system_status_t *status;
  const umh_playback_plan_t *plan;
  const fpga_link_t *fpga;
  const flash_store_t *flash;
  const eeprom_profile_t *eeprom;
  device_gui_action_t start;
  device_gui_action_t stop;
  device_gui_action_t clear;
  device_gui_action_t trigger;
  device_gui_action_t demo;
  device_gui_action_t ws2812_set;
  uint8_t demo_count;
  uint8_t selected_demo;
  uint8_t ws2812_mode;
  uint8_t ws2812_active;
  void *action_context;
  device_gui_page_t page;
  uint8_t row;
  uint8_t scroll_offset;
  uint8_t cursor_y;
  uint8_t content_focused;
  uint8_t action_message;
  uint32_t message_until;
  uint32_t transition_until;
} device_gui_t;

void device_gui_init(device_gui_t *gui, oled_ssd1315_t *oled,
                     const umh_device_profile_t *profile,
                     umh_system_status_t *status,
                     const umh_playback_plan_t *plan,
                     const fpga_link_t *fpga,
                     const flash_store_t *flash,
                     const eeprom_profile_t *eeprom,
                     device_gui_action_t start,
                     device_gui_action_t stop,
                     device_gui_action_t clear,
                     device_gui_action_t trigger,
                     device_gui_action_t demo,
                     device_gui_action_t ws2812_set,
                     uint8_t demo_count,
                     void *action_context);
void device_gui_handle_event(device_gui_t *gui, const input_event_t *event);
void device_gui_render(device_gui_t *gui, uint32_t now_ms);

#endif
