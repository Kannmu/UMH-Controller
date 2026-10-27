#include "device_gui.h"
#include "main.h"
#include "demo_engine.h"
#include "gun_trigger.h"
#include "task_stats.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

#define GUI_TITLE_Y 0u
#define GUI_BODY_Y 10u
#define GUI_ROW_HEIGHT 9u
#define GUI_VISIBLE_ROWS 6u
#define GUI_MESSAGE_MS 1200u
#define GUI_TRANSITION_MS 180u
#define GUI_CURSOR_STEP 3u

static void line(device_gui_t *gui, uint8_t row, const char *label, const char *value);
static void number(char *out, size_t size, uint32_t value);
static size_t put_u32(char *out, size_t size, uint32_t value, char suffix);
static size_t append_text(char *out, size_t size, size_t pos, const char *text);

static const char *page_title(device_gui_page_t page)
{
  static const char *const titles[DEVICE_GUI_PAGE_COUNT] = {
    "STATUS", "PLAYBACK", "DEVICE", "CALIB", "STORAGE", "DEBUG", "SYSTEM",
    "DEMOS", "LED TEST", "GUN"
  };
  return page < DEVICE_GUI_PAGE_COUNT ? titles[page] : "UMH-84";
}

static uint8_t page_item_count(const device_gui_t *gui)
{
  switch (gui->page) {
    case DEVICE_GUI_HOME: return 6u;
    case DEVICE_GUI_PLAYBACK: return 6u;
    case DEVICE_GUI_DEVICE: return 7u;
    case DEVICE_GUI_CALIBRATION: return 8u;
    case DEVICE_GUI_STORAGE: return 6u;
    case DEVICE_GUI_DIAGNOSTICS: return 32u;
    case DEVICE_GUI_SYSTEM: return (uint8_t)(4u + uxTaskGetNumberOfTasks());
    case DEVICE_GUI_DEMOS: return gui->demo_count;
    case DEVICE_GUI_WS2812_TEST: return 5u;
    case DEVICE_GUI_GUN: return UMH_GUN_MODE_COUNT;
    default: return 0u;
  }
}

static void render_system(device_gui_t *gui)
{
  const umh_task_stats_t *stats = umh_task_stats_get();
  uint32_t count = stats->count;
  uint32_t i;
  char value[16];
  if (count > UMH_TASK_STATS_MAX) count = UMH_TASK_STATS_MAX;
  line(gui, 0u, "CPU", "MONITOR");
  number(value, sizeof(value), xPortGetFreeHeapSize()); line(gui, 1u, "HEAP", value);
  number(value, sizeof(value), count); line(gui, 2u, "TASKS", value);
  (void)put_u32(value, sizeof(value), HAL_GetTick() / 1000u, 's');
  line(gui, 3u, "TICK", value);
  for (i = 0u; i < count; ++i) {
    /* The sampler stores the CPU share in 1/100 %, so the page keeps showing
     * whole percent without a scheduler call in the render path. */
    uint32_t percent = stats->entries[i].ulRunTimeCounter / 100u;
    (void)snprintf(value, sizeof(value), "%lu%%", (unsigned long)percent);
    line(gui, (uint8_t)(4u + i), stats->entries[i].pcTaskName, value);
  }
}

static void keep_selected_visible(device_gui_t *gui)
{
  uint8_t count = page_item_count(gui);
  if (count == 0u) {
    gui->row = 0u;
    gui->scroll_offset = 0u;
    return;
  }
  if (gui->row >= count) gui->row = (uint8_t)(count - 1u);
  if (gui->row < gui->scroll_offset) gui->scroll_offset = gui->row;
  if (gui->row >= (uint8_t)(gui->scroll_offset + GUI_VISIBLE_ROWS))
    gui->scroll_offset = (uint8_t)(gui->row - GUI_VISIBLE_ROWS + 1u);
}

static const char *flag_word(uint32_t flags, uint32_t flag, const char *yes, const char *no)
{
  return (flags & flag) != 0u ? yes : no;
}

static void line(device_gui_t *gui, uint8_t row, const char *label, const char *value)
{
  char text[22];
  uint8_t selected;
  size_t pos;
  if (row < gui->scroll_offset || row >= (uint8_t)(gui->scroll_offset + GUI_VISIBLE_ROWS)) return;
  selected = (uint8_t)(gui->content_focused != 0u && row == gui->row);
  /* 5x7 glyphs are six pixels wide. Keep a full blank column between fields:
   * the label is left-justified in 9 characters, exactly like "%-9s%s". */
  pos = append_text(text, sizeof(text), 0u, label);
  while (pos < 9u && pos + 1u < sizeof(text)) text[pos++] = ' ';
  text[pos] = '\0';
  (void)append_text(text, sizeof(text), pos, value != NULL ? value : "-");
  oled_ssd1315_draw_text(gui->oled, 0u,
                         (uint8_t)(GUI_BODY_Y + (row - gui->scroll_offset) * GUI_ROW_HEIGHT),
                         text, selected);
}

static void separator(device_gui_t *gui)
{
  uint8_t x;
  for (x = 0u; x < OLED_WIDTH; ++x) oled_ssd1315_set_pixel(gui->oled, x, 8u, 1u);
}

static void page_header(device_gui_t *gui)
{
  char title[22];
  size_t pos = append_text(title, sizeof(title), 0u, page_title(gui->page));
  pos = append_text(title, sizeof(title), pos, " ");
  pos += put_u32(title + pos, sizeof(title) - pos, (uint32_t)(gui->page + 1u), '/');
  (void)put_u32(title + pos, sizeof(title) - pos, (uint32_t)DEVICE_GUI_PAGE_COUNT, '\0');
  /* The title is the top-level menu item. Highlight it until OK enters the page. */
  oled_ssd1315_draw_text(gui->oled, 0u, GUI_TITLE_Y, title,
                         (uint8_t)(gui->content_focused == 0u));
  separator(gui);
  /* A compact key legend makes the four-button workflow discoverable. */
  oled_ssd1315_draw_text(gui->oled, 92u, 0u, gui->content_focused != 0u ? "OK" : "SEL", 0u);
}

static void draw_cursor(device_gui_t *gui)
{
  uint8_t visible_row;
  uint8_t y;
  if (gui->content_focused == 0u || page_item_count(gui) == 0u) return;
  keep_selected_visible(gui);
  visible_row = (uint8_t)(gui->row - gui->scroll_offset);
  y = (uint8_t)(GUI_BODY_Y + visible_row * GUI_ROW_HEIGHT + 2u);
  oled_ssd1315_set_pixel(gui->oled, 125u, y, 1u);
  oled_ssd1315_set_pixel(gui->oled, 126u, (uint8_t)(y + 1u), 1u);
  oled_ssd1315_set_pixel(gui->oled, 127u, y, 1u);
}

/* Decimal formatting without printf.
 *
 * Every repaint formats the title plus one string per visible row, and the UI
 * repaints at 33 Hz.  newlib-nano charges several microseconds per snprintf(),
 * which made the string building a visible slice of the UI task; these helpers
 * are a few dozen cycles each.  Only the text that appears on the common pages
 * is converted -- the calibration page's rare formats keep snprintf(). */
static size_t put_u32(char *out, size_t size, uint32_t value, char suffix)
{
  char tmp[11];
  size_t digits = 0u;
  size_t pos = 0u;
  if (out == NULL || size == 0u) return 0u;
  do {
    tmp[digits++] = (char)('0' + (char)(value % 10u));
    value /= 10u;
  } while (value != 0u && digits < sizeof(tmp));
  while (digits != 0u && pos + 2u < size) out[pos++] = tmp[--digits];
  if (suffix != '\0' && pos + 1u < size) out[pos++] = suffix;
  out[pos] = '\0';
  return pos;
}

static size_t append_text(char *out, size_t size, size_t pos, const char *text)
{
  if (out == NULL || text == NULL || size == 0u) return pos;
  while (*text != '\0' && pos + 1u < size) out[pos++] = *text++;
  out[pos] = '\0';
  return pos;
}

static void number(char *out, size_t size, uint32_t value)
{
  (void)put_u32(out, size, value, '\0');
}

static const char *fault_name(uint16_t code)
{
  static const char *const names[UMH_FAULT_COUNT] = {
    "NONE", "FPGA START", "FPGA TIMEOUT", "FPGA DMA", "FPGA PROTOCOL",
    "FPGA UNDERRUN", "FPGA OVERFLOW", "FPGA FRAME", "FPGA OUTPUT",
    "PROTOCOL NACK", "PROTOCOL PARSE", "BLOCK PARSE", "PLAN START",
    "FRAME RING", "FLASH IO", "EEPROM IO", "USB RX DROP", "USB TX DROP",
    "HAL INIT", "CAL MIC", "CAL SOLVE", "CAL QUAL"
  };
  return code < UMH_FAULT_COUNT ? names[code] : "UNKNOWN";
}

static const char *fault_severity(uint8_t severity)
{
  return severity >= UMH_FAULT_CRITICAL ? "CRIT" :
         severity == UMH_FAULT_WARNING ? "WARN" : "INFO";
}

static void render_home(device_gui_t *gui)
{
  char value[14];
  const umh_system_status_t *s = gui->status;
  line(gui, 0u, "LINK", flag_word(s->flags, UMH_SYSTEM_CONNECTED, "ON", "OFF"));
  line(gui, 1u, "FPGA", flag_word(s->flags, UMH_SYSTEM_FPGA_READY, "READY", "WAIT"));
  line(gui, 2u, "OUTPUT", gui->fpga != NULL && gui->fpga->running != 0u ?
       ((s->flags & UMH_SYSTEM_PLAYING) != 0u ? "RUN" : "HOLD") : "IDLE");
  number(value, sizeof(value), s->frame_count); line(gui, 3u, "FRAMES", value);
  number(value, sizeof(value), s->fpga_credit); line(gui, 4u, "CREDIT", value);
  (void)put_u32(value, sizeof(value), s->uptime_ms / 1000u, 's');
  line(gui, 5u, "UPTIME", value);
}

static void render_playback(device_gui_t *gui)
{
  char value[16];
  const umh_playback_plan_t *p = gui->plan;
  const char *start_names[] = {"NOW", "TIME", "TRIG", "BOUND"};
  const char *repeat_names[] = {"ONCE", "RAM", "STREAM", "HOLD", "STOP"};
  line(gui, 0u, "STATE", p->running != 0u ? "RUN" :
       (gui->fpga != NULL && gui->fpga->running != 0u ? "HOLD" :
        (p->configured != 0u ? "READY" : "EMPTY")));
  number(value, sizeof(value), p->wire.block_id); line(gui, 1u, "BLOCK", value);
  number(value, sizeof(value), p->current_frame); line(gui, 2u, "FRAME", value);
  (void)snprintf(value, sizeof(value), "%lu/%lu", (unsigned long)p->wire.rate_numerator,
                 (unsigned long)p->wire.rate_denominator); line(gui, 3u, "RATE", value);
  line(gui, 4u, "START", p->wire.start_mode < 4u ? start_names[p->wire.start_mode] : "-");
  line(gui, 5u, "REPEAT", p->wire.repeat_mode < 5u ? repeat_names[p->wire.repeat_mode] : "-");
}

static void render_device(device_gui_t *gui)
{
  char value[16];
  const umh_device_profile_t *p = gui->profile;
  number(value, sizeof(value), p->channel_count); line(gui, 0u, "CHANNELS", value);
  number(value, sizeof(value), p->rgb_count); line(gui, 1u, "RGB", value);
  number(value, sizeof(value), p->microphone_count); line(gui, 2u, "MIC", value);
  number(value, sizeof(value), p->carrier_hz); line(gui, 3u, "CARRIER", value);
  number(value, sizeof(value), p->timebase_hz); line(gui, 4u, "TIMEBASE", value);
  number(value, sizeof(value), p->fpga_ref_clock_hz); line(gui, 5u, "FPGA CLK", value);
  line(gui, 6u, "GEOMETRY", (p->capability_flags & UMH_PROFILE_CAP_GEOMETRY_VALID) != 0u ? "OK" : "--");
}

static const char *cal_state_name(uint8_t state)
{
  static const char *const names[DEVICE_GUI_CAL_STATE_COUNT] = {
    "IDLE", "WAIT", "MEAS", "SOLVE", "VERIFY", "OK", "FAIL"
  };
  return state < DEVICE_GUI_CAL_STATE_COUNT ? names[state] : "?";
}

static void render_calibration(device_gui_t *gui)
{
  char value[16];
  uint16_t enabled = 0u;
  uint16_t i;
  const eeprom_profile_record_t *record = gui->eeprom != NULL ? &gui->eeprom->record : NULL;
  const umh_system_status_t *s = gui->status;
  if (record != NULL) {
    for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i)
      if ((record->enabled[i / 8u] & (uint8_t)(1u << (i % 8u))) != 0u) ++enabled;
  }
  if (gui->calibration_busy != 0u) {
    line(gui, 0u, "STATE", cal_state_name(gui->calibration_state));
    number(value, sizeof(value), record != NULL ? record->version : 0u); line(gui, 1u, "VERSION", value);
    number(value, sizeof(value), record != NULL ? record->generation : 0u); line(gui, 2u, "GEN", value);
    (void)snprintf(value, sizeof(value), "%u%%", (unsigned)gui->calibration_progress);
    line(gui, 3u, "PROG", value);
    number(value, sizeof(value), s != NULL ? s->cal_rms_deg_x10 : 0u); line(gui, 4u, "FITRMS", value);
    if (s != NULL) {
      int16_t t = s->cal_tilt_x_x10;
      (void)snprintf(value, sizeof(value), "%d.%d", (int)(t / 10), (int)((t < 0 ? -t : t) % 10));
    } else (void)snprintf(value, sizeof(value), "-");
    line(gui, 5u, "GAIN", value);
    line(gui, 6u, "RUN", gui->calibration_state == DEVICE_GUI_CAL_FAIL ? "RETRY" : "BUSY");
    if (s != NULL && s->self_test_valid != 0u)
      line(gui, 7u, "STEST", s->self_test_pass != 0u ? "PASS" : "FAIL");
    else
      line(gui, 7u, "STEST", "WAIT");
  } else {
    line(gui, 0u, "STATE", gui->eeprom != NULL && gui->eeprom->present != 0u ?
         (gui->eeprom->valid != 0u ? "VALID" : "DEFAULT") : "OFFLINE");
    number(value, sizeof(value), record != NULL ? record->version : 0u); line(gui, 1u, "VERSION", value);
    number(value, sizeof(value), record != NULL ? record->generation : 0u); line(gui, 2u, "GEN", value);
    number(value, sizeof(value), enabled); line(gui, 3u, "ENABLED", value);
    number(value, sizeof(value), s != NULL ? s->cal_rms_deg_x10 : 0u); line(gui, 4u, "FITRMS", value);
    if (s != NULL && s->cal_state == DEVICE_GUI_CAL_OK) {
      int16_t t = s->cal_tilt_x_x10;
      (void)snprintf(value, sizeof(value), "%d.%d", (int)(t / 10), (int)((t < 0 ? -t : t) % 10));
    } else (void)snprintf(value, sizeof(value), "-");
    line(gui, 5u, "GAIN", value);
    line(gui, 6u, "RUN", "PRESS OK");
    if (s != NULL && s->self_test_valid != 0u)
      line(gui, 7u, "STEST", s->self_test_pass != 0u ? "PASS" : "FAIL");
    else
      line(gui, 7u, "STEST", "RUN");
  }
}
static void render_storage(device_gui_t *gui)
{
  char value[16];
  const umh_system_status_t *s = gui->status;
  number(value, sizeof(value), gui->flash != NULL ? gui->flash->count : 0u); line(gui, 0u, "OBJECTS", value);
  line(gui, 1u, "FLASH", flag_word(s->flags, UMH_SYSTEM_FLASH_READY, "READY", "WAIT"));
  line(gui, 2u, "EEPROM", gui->eeprom != NULL && gui->eeprom->present != 0u ?
       (gui->eeprom->valid != 0u ? "VALID" : "DEFAULT") : "OFFLINE");
  if (gui->eeprom != NULL) { number(value, sizeof(value), gui->eeprom->record.generation); line(gui, 3u, "GEN", value); }
  number(value, sizeof(value), s->usb_dropped); line(gui, 4u, "USB DROP", value);
  number(value, sizeof(value), gui->eeprom != NULL ? gui->eeprom->io_errors : 0u); line(gui, 5u, "EEP ERR", value);
}

static void render_diagnostic_item(device_gui_t *gui, uint8_t item)
{
  char value[22];
  const umh_system_status_t *s = gui->status;
  const fpga_status_wire_t *f = gui->fpga != NULL ? &gui->fpga->status : NULL;
  const umh_playback_plan_t *p = gui->plan;
  const umh_device_profile_t *profile = gui->profile;
  uint32_t age;
  uint16_t fault_code;
  uint32_t fault_arg;
  uint32_t fault_time;
  uint8_t fault_level;
  if ((s->flags & UMH_SYSTEM_ERROR) != 0u && s->critical_fault_valid != 0u) {
    fault_code = s->critical_fault_code;
    fault_arg = s->critical_fault_arg;
    fault_time = s->critical_fault_time_ms;
    fault_level = UMH_FAULT_CRITICAL;
  } else {
    fault_code = s->last_fault_code;
    fault_arg = s->last_fault_arg;
    fault_time = s->last_fault_time_ms;
    fault_level = s->last_fault_severity;
  }
  switch (item) {
    case 0u:
      line(gui, 0u, "SEVERITY", fault_code == UMH_FAULT_NONE ? "OK" : fault_severity(fault_level));
      line(gui, 1u, "CAUSE", fault_name(fault_code));
      number(value, sizeof(value), fault_arg); line(gui, 2u, "ARG", value);
      age = fault_code == UMH_FAULT_NONE ? 0u : HAL_GetTick() - fault_time;
      number(value, sizeof(value), age); line(gui, 3u, "AGE MS", value);
      number(value, sizeof(value), s->fault_count); line(gui, 4u, "FAULTS", value);
      line(gui, 5u, "FLAGS", (s->flags & UMH_SYSTEM_ERROR) != 0u ? "ERROR" : "NORMAL");
      break;
    case 6u: number(value, sizeof(value), s->protocol_errors); line(gui, 6u, "PROTO", value); break;
    case 7u: number(value, sizeof(value), s->parser_errors); line(gui, 7u, "PARSER", value); break;
    case 8u: number(value, sizeof(value), s->fpga_errors); line(gui, 8u, "FPGA", value); break;
    case 9u: number(value, sizeof(value), s->underruns); line(gui, 9u, "UNDERRUN", value); break;
    case 10u: number(value, sizeof(value), s->usb_dropped); line(gui, 10u, "USB TX", value); break;
    case 11u: number(value, sizeof(value), s->rx_dropped); line(gui, 11u, "USB RX", value); break;
    case 12u:
      if (f == NULL) {
        line(gui, 12u, "FPGA", "NO LINK");
      } else {
        number(value, sizeof(value), f->protocol_version); line(gui, 12u, "PROTO VER", value);
        number(value, sizeof(value), f->fifo_credit); line(gui, 13u, "CREDIT", value);
        number(value, sizeof(value), f->fifo_depth); line(gui, 14u, "DEPTH", value);
        number(value, sizeof(value), f->status_flags); line(gui, 15u, "FLAGS", value);
        number(value, sizeof(value), f->accepted_sequence); line(gui, 16u, "ACCEPTED", value);
        number(value, sizeof(value), f->fpga_time); line(gui, 17u, "FPGA TIME", value);
      }
      break;
    case 18u: number(value, sizeof(value), s->uptime_ms / 1000u); line(gui, 18u, "UPTIME S", value); break;
    case 19u: number(value, sizeof(value), s->device_time / 1000u); line(gui, 19u, "DEVICE MS", value); break;
    case 20u: number(value, sizeof(value), s->frame_count); line(gui, 20u, "FRAMES", value); break;
    case 21u: number(value, sizeof(value), s->frame_free); line(gui, 21u, "FREE", value); break;
    case 22u: number(value, sizeof(value), s->frame_dropped); line(gui, 22u, "FRAME DROP", value); break;
    case 23u: line(gui, 23u, "PLAY", p != NULL && p->running != 0u ? "RUN" : "STOP"); break;
    case 24u: line(gui, 24u, "MODEL", profile != NULL ? profile->model : "-"); break;
    case 25u: line(gui, 25u, "FIRMWARE", profile != NULL ? profile->firmware : "-"); break;
    case 26u: line(gui, 26u, "PROTOCOL", profile != NULL ? profile->protocol : "-"); break;
    case 27u: number(value, sizeof(value), profile != NULL ? profile->ram_bytes / 1024u : 0u); line(gui, 27u, "RAM KB", value); break;
    case 28u: number(value, sizeof(value), profile != NULL ? profile->max_frame_rate : 0u); line(gui, 28u, "MAX FPS", value); break;
    case 29u: number(value, sizeof(value), profile != NULL ? profile->calibration_generation : 0u); line(gui, 29u, "CAL GEN", value); break;
    case 30u: number(value, sizeof(value), gui->eeprom != NULL ? gui->eeprom->io_errors : 0u); line(gui, 30u, "EEP ERR", value); break;
    default: line(gui, 31u, "HEART", s->heartbeat != 0u ? "ON" : "OFF"); break;
  }
}

static void render_diagnostics(device_gui_t *gui)
{
  uint8_t item;
  for (item = 0u; item < page_item_count(gui); ++item) render_diagnostic_item(gui, item);
}

static void render_demos(device_gui_t *gui)
{
  uint8_t i;
  for (i = 0u; i < gui->demo_count; ++i) {
    const umh_demo_descriptor_t *demo = demo_engine_descriptor(i);
    line(gui, i, demo != NULL ? demo->name : "-", i == gui->selected_demo ? "SELECT" : "READY");
  }
}

static void render_ws2812_test(device_gui_t *gui)
{
  const char *modes[] = {"OFF", "RED", "GREEN", "BLUE", "WHITE"};
  line(gui, 0u, "MODE", modes[gui->ws2812_mode]);
  line(gui, 1u, "STATUS", gui->ws2812_active ? "ACTIVE" : "IDLE");
  line(gui, 2u, "ACTIVATE", "PRESS OK");
  line(gui, 3u, "CLEAR", "PRESS OK");
  line(gui, 4u, "INFO", "4 LED TEST");
}

/* One row per firing mode.  The selected row is the mode PA9 fires; the cursor
 * (draw_cursor) marks it as well.  The trigger state is on every row's value
 * column so an operator can see the gun firing without leaving the menu. */
static void render_gun(device_gui_t *gui)
{
  uint8_t i;
  const char *state = "READY";
  if (gui->status != NULL) {
    if ((gui->status->flags & UMH_SYSTEM_RECORDING) != 0u) state = "REC";
    else if ((gui->status->flags & UMH_SYSTEM_PLAYING) != 0u) state = "FIRE";
  }
  for (i = 0u; i < UMH_GUN_MODE_COUNT; ++i)
    line(gui, i, umh_gun_mode_names[i], i == gui->row ? state : "-");
}

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
                     device_gui_action_t ws2812_set,
                     device_gui_action_t gun_select,
                     device_gui_action_t gun_exit,
                     uint8_t demo_count,
                     void *action_context)
{
  if (gui == NULL) return;
  memset(gui, 0, sizeof(*gui));
  gui->oled = oled; gui->profile = profile; gui->status = status; gui->plan = plan;
  gui->fpga = fpga; gui->flash = flash; gui->eeprom = eeprom;
  gui->demo = demo;
  gui->ws2812_set = ws2812_set; gui->calibration = calibration;
  gui->self_test = self_test;
  gui->gun_select = gun_select; gui->gun_exit = gun_exit;
  gui->demo_count = demo_count;
  gui->action_context = action_context; gui->page = DEVICE_GUI_HOME;
  gui->cursor_y = GUI_BODY_Y + 1u;
  gui->ws2812_mode = 0u;
  gui->ws2812_active = 0u;
}

/* Enter or leave the gun menu.
 *
 * Entering always starts at the first mode and fires it, so the menu is
 * immediately usable and the default a long press leaves behind is the single
 * focus.  Leaving hands the trigger back to that same default through gun_exit,
 * whatever the menu had selected. */
static void gun_menu_set(device_gui_t *gui, uint8_t active);

static void select_page(device_gui_t *gui, device_gui_page_t page)
{
  if (gui->page == page) return;
  gui->page = page;
  gui->row = 0u;
  gui->content_focused = 0u;
  gui->scroll_offset = 0u;
  gui->transition_until = HAL_GetTick() + GUI_TRANSITION_MS;
  gui->cursor_y = GUI_BODY_Y + 1u;
}

static void gun_menu_set(device_gui_t *gui, uint8_t active)
{
  if (active != 0u) {
    select_page(gui, DEVICE_GUI_GUN);
    gui->row = 0u;
    gui->content_focused = 1u;
    if (gui->gun_select != NULL) (void)gui->gun_select(gui->action_context);
  } else {
    if (gui->gun_exit != NULL) (void)gui->gun_exit(gui->action_context);
    select_page(gui, DEVICE_GUI_HOME);
  }
  gui->action_message = 0u;
}

/* Short press of either back-key candidate inside the gun menu steps to the
 * next firing mode.  gun_select can refuse while a shot is held, in which case
 * the cursor returns to the mode still being emitted. */
static void gun_menu_step(device_gui_t *gui)
{
  uint8_t next;
  if (gui == NULL || gui->page != DEVICE_GUI_GUN) return;
  next = (uint8_t)((gui->row + 1u) % UMH_GUN_MODE_COUNT);
  gui->row = next;
  if (gui->gun_select != NULL && gui->gun_select(gui->action_context) != 0)
    gui->row = (uint8_t)((next + UMH_GUN_MODE_COUNT - 1u) % UMH_GUN_MODE_COUNT);
}

static void move_page(device_gui_t *gui, int8_t direction)
{
  int16_t next = (int16_t)gui->page + direction;
  if (next < (int16_t)DEVICE_GUI_HOME) next = (int16_t)DEVICE_GUI_GUN - 1;
  else if (next >= (int16_t)DEVICE_GUI_GUN) next = DEVICE_GUI_HOME;
  select_page(gui, (device_gui_page_t)next);
}

void device_gui_handle_event(device_gui_t *gui, const input_event_t *event)
{
  uint8_t count;
  if (gui == NULL || event == NULL) return;
  /* A calibration run owns the device: ignore all key presses until it ends. */
  if (gui->calibration_busy != 0u) return;

  /* The gun menu is a mode of the whole UI, entered and left with one long press
   * of either back-key candidate (KEY0/SW2 or KEY3/SW5) from anywhere, so this is handled before any page logic and
   * applies to both the press and the long report.  A short press of the same
   * key is an ordinary event and falls through to the page handlers below --
   * inside the gun menu that is what steps through the firing modes. */
  if ((event->key == INPUT_KEY0 || event->key == INPUT_KEY3) &&
      event->type == INPUT_EVENT_LONG) {
    gun_menu_set(gui, (uint8_t)(gui->page != DEVICE_GUI_GUN));
    return;
  }
  if (event->type != INPUT_EVENT_PRESS) return;
  /* Inside the gun menu both back-key candidates are mode selectors: a short
   * press steps to the next row.  This runs before the normal page handlers so
   * KEY0 does not try to leave the menu and KEY3 does not try to page away. */
  if (gui->page == DEVICE_GUI_GUN &&
      (event->key == INPUT_KEY0 || event->key == INPUT_KEY3)) {
    gun_menu_step(gui);
    return;
  }


  if (event->key == INPUT_KEY0) {
    if (gui->content_focused != 0u) {
      gui->content_focused = 0u;
      gui->scroll_offset = 0u;
      gui->row = 0u;
    } else if (gui->page != DEVICE_GUI_HOME) {
      select_page(gui, DEVICE_GUI_HOME);
    }
    return;
  }

  if (gui->content_focused == 0u) {
    if (event->key == INPUT_KEY2) move_page(gui, 1);
    else if (event->key == INPUT_KEY3) move_page(gui, -1);
    else if (event->key == INPUT_KEY1) {
      gui->content_focused = 1u;
      gui->row = 0u;
      gui->scroll_offset = 0u;
      gui->selected_demo = 0u;
    }
    return;
  }

  count = page_item_count(gui);
  if (event->key == INPUT_KEY2 && count != 0u) {
    if (gui->row + 1u < count) ++gui->row;
    keep_selected_visible(gui);
    /* In the gun menu the down key moves the same cursor as the back key, so the
     * firing mode has to follow it too: leaving them apart would let the display
     * point at one mode while the trigger fires another. */
    if (gui->page == DEVICE_GUI_GUN && gui->gun_select != NULL)
      (void)gui->gun_select(gui->action_context);
    if (gui->page == DEVICE_GUI_DEMOS && gui->row < gui->demo_count) gui->selected_demo = gui->row;
    return;
  }
  if (event->key == INPUT_KEY3 && count != 0u) {
    if (gui->row != 0u) --gui->row;
    keep_selected_visible(gui);
    if (gui->page == DEVICE_GUI_DEMOS && gui->row < gui->demo_count) gui->selected_demo = gui->row;
    return;
  }
  if (event->key != INPUT_KEY1) return;

  /* The gun menu is driven by the back key (mode cycling) and the trigger; OK has
   * no action there, and falling through would run the demo action on whatever
   * row index the menu happens to be on. */
  if (gui->page == DEVICE_GUI_GUN) return;

  if (gui->page == DEVICE_GUI_CALIBRATION && gui->row == 6u) {
    int result = gui->calibration != NULL ? gui->calibration(gui->action_context) : -1;
    gui->action_message = result == 0 ? 1u : 2u;
    gui->message_until = HAL_GetTick() + 3000u;
    return;
  }
  if (gui->page == DEVICE_GUI_CALIBRATION && gui->row == 7u) {
    int result = gui->self_test != NULL ? gui->self_test(gui->action_context) : -1;
    gui->action_message = result == 0 ? 1u : 2u;
    gui->message_until = HAL_GetTick() + 3000u;
    return;
  }

  if (gui->page == DEVICE_GUI_WS2812_TEST && gui->row < 4u) {
    int result = -1;
    if (gui->row == 0u) {
      /* Cycle through modes */
      gui->ws2812_mode = (gui->ws2812_mode + 1u) % 5u;
      result = 0;
    } else if (gui->row == 2u && gui->ws2812_set != NULL) {
      /* Activate selected mode */
      result = gui->ws2812_set(gui->action_context);
      if (result == 0) gui->ws2812_active = 1u;
    } else if (gui->row == 3u && gui->ws2812_set != NULL) {
      /* Clear - turn off all LEDs */
      gui->ws2812_mode = 0u;
      result = gui->ws2812_set(gui->action_context);
      if (result == 0) gui->ws2812_active = 0u;
    }
    if (gui->row != 1u && gui->row != 4u) {
      gui->action_message = result == 0 ? 1u : 2u;
      gui->message_until = HAL_GetTick() + GUI_MESSAGE_MS;
    }
  } else if (gui->page == DEVICE_GUI_DEMOS && gui->row < gui->demo_count && gui->demo != NULL) {
    int result = gui->demo(gui->action_context);
    gui->action_message = result == 0 ? 1u : 2u;
    gui->message_until = HAL_GetTick() + GUI_MESSAGE_MS;
  }
}


uint8_t device_gui_calibration_busy(const device_gui_t *gui)
{
  return gui != NULL ? gui->calibration_busy : 0u;
}

void device_gui_calibration_begin(device_gui_t *gui)
{
  if (gui == NULL) return;
  gui->calibration_busy = 1u;
  gui->calibration_state = DEVICE_GUI_CAL_WAIT;
  gui->calibration_progress = 0u;
  gui->action_message = 0u;
}

void device_gui_calibration_state(device_gui_t *gui, uint8_t state, uint8_t progress)
{
  if (gui == NULL) return;
  gui->calibration_state = state;
  gui->calibration_progress = progress;
  if (state == DEVICE_GUI_CAL_OK || state == DEVICE_GUI_CAL_FAIL) {
    gui->calibration_busy = 0u;
    gui->action_message = state == DEVICE_GUI_CAL_OK ? 1u : 2u;
    gui->message_until = HAL_GetTick() + 3000u;
  }
}
void device_gui_render(device_gui_t *gui, uint32_t now_ms)
{
  uint8_t progress;
  uint8_t x;
  if (gui == NULL || gui->oled == NULL) return;
  oled_ssd1315_clear(gui->oled);
  page_header(gui);
  switch (gui->page) {
    case DEVICE_GUI_HOME: render_home(gui); break;
    case DEVICE_GUI_PLAYBACK: render_playback(gui); break;
    case DEVICE_GUI_DEVICE: render_device(gui); break;
    case DEVICE_GUI_CALIBRATION: render_calibration(gui); break;
    case DEVICE_GUI_STORAGE: render_storage(gui); break;
    case DEVICE_GUI_DIAGNOSTICS: render_diagnostics(gui); break;
    case DEVICE_GUI_SYSTEM: render_system(gui); break;
    case DEVICE_GUI_DEMOS: render_demos(gui); break;
    case DEVICE_GUI_WS2812_TEST: render_ws2812_test(gui); break;
    case DEVICE_GUI_GUN: render_gun(gui); break;
    default: break;
  }
  draw_cursor(gui);
  if ((int32_t)(gui->transition_until - now_ms) > 0) {
    uint32_t elapsed = GUI_TRANSITION_MS - (gui->transition_until - now_ms);
    progress = (uint8_t)((elapsed * OLED_WIDTH) / GUI_TRANSITION_MS);
    for (x = 0u; x < progress; ++x) oled_ssd1315_set_pixel(gui->oled, x, 63u, 1u);
  }
  if (gui->action_message != 0u && (int32_t)(gui->message_until - now_ms) > 0)
    oled_ssd1315_draw_text(gui->oled, 78u, 56u, gui->action_message == 1u ? "OK" : "ERR", 1u);
  else gui->action_message = 0u;
}
