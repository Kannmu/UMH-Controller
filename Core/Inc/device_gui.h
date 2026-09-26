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
  DEVICE_GUI_LEVITATION,
  DEVICE_GUI_DEFOAM,
  DEVICE_GUI_DEMOS,
  DEVICE_GUI_WS2812_TEST,
  DEVICE_GUI_PAGE_COUNT
} device_gui_page_t;

/* Defoam page rows.  Rows 0..1 arm the two vortex programs, rows 2..3 report
 * the focus and the reversal period, and row 4 stops whatever is running. */
#define DEVICE_GUI_VORTEX_STEADY 0u
#define DEVICE_GUI_VORTEX_ALT    1u
#define DEVICE_GUI_DEFOAM_STOP_ROW 4u
/* Not a row: the value put in device_gui_t.defoam_mode when the STOP row was
 * confirmed, so the action can tell "stop" from "run program N". */
#define DEVICE_GUI_DEFOAM_STOP  0xFFu

typedef int (*device_gui_action_t)(void *context);

typedef enum {
  DEVICE_GUI_CAL_IDLE = 0u,
  DEVICE_GUI_CAL_WAIT,
  DEVICE_GUI_CAL_MEASURE,
  DEVICE_GUI_CAL_SOLVE,
  DEVICE_GUI_CAL_VERIFY,
  DEVICE_GUI_CAL_OK,
  DEVICE_GUI_CAL_FAIL,
  DEVICE_GUI_CAL_STATE_COUNT
} device_gui_cal_state_t;

typedef struct {
  oled_ssd1315_t *oled;
  const umh_device_profile_t *profile;
  umh_system_status_t *status;
  const umh_playback_plan_t *plan;
  const fpga_link_t *fpga;
  const flash_store_t *flash;
  const eeprom_profile_t *eeprom;
  device_gui_action_t demo;
  device_gui_action_t levitation_toggle;
  device_gui_action_t defoam_set;
  device_gui_action_t ws2812_set;
  device_gui_action_t calibration;
  device_gui_action_t self_test;
  volatile uint8_t calibration_busy;
  volatile uint8_t calibration_state;
  volatile uint8_t calibration_progress;
  uint8_t demo_count;
  uint8_t selected_demo;
  uint8_t ws2812_mode;
  uint8_t ws2812_active;
  /* Last defoam row the user confirmed, passed to defoam_set through the
   * action context the same way ws2812_mode is passed to ws2812_set. */
  uint8_t defoam_mode;
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
                     device_gui_action_t calibration,
                     device_gui_action_t self_test,
                     device_gui_action_t demo,
                     device_gui_action_t levitation_toggle,
                     device_gui_action_t defoam_set,
                     device_gui_action_t ws2812_set,
                     uint8_t demo_count,
                     void *action_context);
void device_gui_handle_event(device_gui_t *gui, const input_event_t *event);
void device_gui_render(device_gui_t *gui, uint32_t now_ms);
uint8_t device_gui_calibration_busy(const device_gui_t *gui);
void device_gui_calibration_begin(device_gui_t *gui);
void device_gui_calibration_state(device_gui_t *gui, uint8_t state, uint8_t progress);

#endif
