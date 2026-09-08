#include "device_gui.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

#define GUI_TITLE_Y 0u
#define GUI_BODY_Y 9u
#define GUI_ROW_HEIGHT 8u
#define GUI_VISIBLE_ROWS 6u
#define GUI_MESSAGE_MS 1200u

static const char *page_title(device_gui_page_t page)
{
  static const char *const titles[DEVICE_GUI_PAGE_COUNT] = {
    "UMH-84", "PLAYBACK", "DEVICE", "CALIB", "STORAGE", "DEBUG", "CONTROL"
  };
  return page < DEVICE_GUI_PAGE_COUNT ? titles[page] : "UMH-84";
}

static const char *flag_word(uint32_t flags, uint32_t flag, const char *yes, const char *no)
{
  return (flags & flag) != 0u ? yes : no;
}

static void line(device_gui_t *gui, uint8_t row, const char *label, const char *value)
{
  char text[22];
  uint8_t selected = (uint8_t)(gui->page == DEVICE_GUI_CONTROL && row == gui->row);
  (void)snprintf(text, sizeof(text), "%-8s %s", label, value != NULL ? value : "-");
  oled_ssd1315_draw_text(gui->oled, 0u, (uint8_t)(GUI_BODY_Y + row * GUI_ROW_HEIGHT), text, selected);
}

static void number(char *out, size_t size, uint32_t value)
{
  (void)snprintf(out, size, "%lu", (unsigned long)value);
}

static const char *fault_name(uint16_t code)
{
  static const char *const names[UMH_FAULT_COUNT] = {
    "NONE", "FPGA START", "FPGA TIMEOUT", "FPGA DMA", "FPGA PROTOCOL",
    "FPGA UNDERRUN", "FPGA OVERFLOW", "FPGA FRAME", "FPGA OUTPUT",
    "PROTOCOL NACK", "PROTOCOL PARSE", "BLOCK PARSE", "PLAN START",
    "FRAME RING", "FLASH IO", "EEPROM IO", "USB RX DROP", "USB TX DROP",
    "HAL INIT"
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
  oled_ssd1315_draw_text(gui->oled, 0u, GUI_BODY_Y, "LINK     ", 0u);
  oled_ssd1315_draw_text(gui->oled, 54u, GUI_BODY_Y, flag_word(s->flags, UMH_SYSTEM_CONNECTED, "ON", "OFF"), 0u);
  oled_ssd1315_draw_text(gui->oled, 0u, 17u, "FPGA     ", 0u);
  oled_ssd1315_draw_text(gui->oled, 54u, 17u, flag_word(s->flags, UMH_SYSTEM_FPGA_READY, "READY", "WAIT"), 0u);
  oled_ssd1315_draw_text(gui->oled, 0u, 25u, "OUTPUT   ", 0u);
  oled_ssd1315_draw_text(gui->oled, 54u, 25u,
                         gui->fpga != NULL && gui->fpga->running != 0u ?
                         (s->flags & UMH_SYSTEM_PLAYING) != 0u ? "RUN" : "HOLD" : "IDLE", 0u);
  number(value, sizeof(value), s->frame_count); line(gui, 3u, "FRAMES", value);
  number(value, sizeof(value), s->fpga_credit); line(gui, 4u, "CREDIT", value);
  number(value, sizeof(value), s->fault_count); line(gui, 5u, "ERRORS", value);
  if ((s->flags & UMH_SYSTEM_ERROR) != 0u && s->critical_fault_valid != 0u) {
    oled_ssd1315_draw_text(gui->oled, 0u, 56u, "ERR ", 1u);
    oled_ssd1315_draw_text(gui->oled, 24u, 56u, fault_name(s->critical_fault_code), 1u);
  }
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
  oled_ssd1315_draw_text(gui->oled, 0u, 56u, (p->capability_flags & UMH_PROFILE_CAP_GEOMETRY_VALID) != 0u ? "GEOMETRY OK" : "GEOMETRY --", 0u);
}

static void render_calibration(device_gui_t *gui)
{
  char value[16];
  uint16_t enabled = 0u;
  uint16_t i;
  const eeprom_profile_record_t *record = gui->eeprom != NULL ? &gui->eeprom->record : NULL;
  if (record != NULL) {
    for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i)
      if ((record->enabled[i / 8u] & (uint8_t)(1u << (i % 8u))) != 0u) ++enabled;
  }
  line(gui, 0u, "STATE", gui->status != NULL && (gui->status->flags & UMH_SYSTEM_CALIBRATION_VALID) != 0u ? "VALID" : "DEFAULT");
  number(value, sizeof(value), record != NULL ? record->version : 0u); line(gui, 1u, "VERSION", value);
  number(value, sizeof(value), record != NULL ? record->generation : 0u); line(gui, 2u, "GEN", value);
  number(value, sizeof(value), enabled); line(gui, 3u, "ENABLED", value);
  number(value, sizeof(value), gui->profile != NULL ? gui->profile->phase_bits : 0u); line(gui, 4u, "PHASE", value);
  number(value, sizeof(value), gui->profile != NULL ? gui->profile->intensity_bits : 0u); line(gui, 5u, "LEVEL", value);
}

static void render_storage(device_gui_t *gui)
{
  char value[16];
  const umh_system_status_t *s = gui->status;
  number(value, sizeof(value), gui->flash != NULL ? gui->flash->count : 0u); line(gui, 0u, "OBJECTS", value);
  oled_ssd1315_draw_text(gui->oled, 0u, 17u, "FLASH    ", 0u);
  oled_ssd1315_draw_text(gui->oled, 54u, 17u, flag_word(s->flags, UMH_SYSTEM_FLASH_READY, "READY", "WAIT"), 0u);
  oled_ssd1315_draw_text(gui->oled, 0u, 25u, "EEPROM   ", 0u);
  oled_ssd1315_draw_text(gui->oled, 54u, 25u, gui->eeprom != NULL && gui->eeprom->valid != 0u ? "VALID" : "DEFAULT", 0u);
  if (gui->eeprom != NULL) { number(value, sizeof(value), gui->eeprom->record.generation); line(gui, 3u, "GEN", value); }
  number(value, sizeof(value), s->usb_dropped); line(gui, 4u, "USB DROP", value);
  oled_ssd1315_draw_text(gui->oled, 0u, 49u, "DATA LOCAL", 0u);
}

static void render_diagnostics(device_gui_t *gui)
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
  switch (gui->debug_view % 5u) {
    case 0u:
      line(gui, 0u, "SEVERITY", fault_code == UMH_FAULT_NONE ? "OK" : fault_severity(fault_level));
      line(gui, 1u, "CAUSE", fault_name(fault_code));
      number(value, sizeof(value), fault_arg); line(gui, 2u, "ARG", value);
      age = fault_code == UMH_FAULT_NONE ? 0u : HAL_GetTick() - fault_time;
      number(value, sizeof(value), age); line(gui, 3u, "AGE MS", value);
      number(value, sizeof(value), s->fault_count); line(gui, 4u, "FAULTS", value);
      line(gui, 5u, "FLAGS", (s->flags & UMH_SYSTEM_ERROR) != 0u ? "ERROR" : "NORMAL");
      oled_ssd1315_draw_text(gui->oled, 0u, 56u, "K1 NEXT DETAIL", 1u);
      break;
    case 1u:
      number(value, sizeof(value), s->protocol_errors); line(gui, 0u, "PROTO", value);
      number(value, sizeof(value), s->parser_errors); line(gui, 1u, "PARSER", value);
      number(value, sizeof(value), s->fpga_errors); line(gui, 2u, "FPGA", value);
      number(value, sizeof(value), s->underruns); line(gui, 3u, "UNDERRUN", value);
      number(value, sizeof(value), s->usb_dropped); line(gui, 4u, "USB TX", value);
      number(value, sizeof(value), s->rx_dropped); line(gui, 5u, "USB RX", value);
      oled_ssd1315_draw_text(gui->oled, 0u, 56u, "K1 NEXT LINK", 1u);
      break;
    case 2u:
      if (f == NULL) {
        line(gui, 0u, "FPGA", "NO LINK");
      } else {
        number(value, sizeof(value), f->protocol_version); line(gui, 0u, "PROTO VER", value);
        number(value, sizeof(value), f->fifo_credit); line(gui, 1u, "CREDIT", value);
        number(value, sizeof(value), f->fifo_depth); line(gui, 2u, "DEPTH", value);
        number(value, sizeof(value), f->status_flags); line(gui, 3u, "FLAGS", value);
        number(value, sizeof(value), f->accepted_sequence); line(gui, 4u, "ACCEPTED", value);
        number(value, sizeof(value), f->fpga_time); line(gui, 5u, "FPGA TIME", value);
      }
      oled_ssd1315_draw_text(gui->oled, 0u, 56u, "K1 NEXT RUNTIME", 1u);
      break;
    case 3u:
      number(value, sizeof(value), s->uptime_ms / 1000u); line(gui, 0u, "UPTIME S", value);
      number(value, sizeof(value), s->device_time / 1000u); line(gui, 1u, "DEVICE MS", value);
      number(value, sizeof(value), s->frame_count); line(gui, 2u, "FRAMES", value);
      number(value, sizeof(value), s->frame_free); line(gui, 3u, "FREE", value);
      number(value, sizeof(value), s->frame_dropped); line(gui, 4u, "FRAME DROP", value);
      line(gui, 5u, "PLAY", p != NULL && p->running != 0u ? "RUN" : "STOP");
      oled_ssd1315_draw_text(gui->oled, 0u, 56u, "K1 NEXT DEVICE", 1u);
      break;
    default:
      line(gui, 0u, "MODEL", profile != NULL ? profile->model : "-");
      line(gui, 1u, "FIRMWARE", profile != NULL ? profile->firmware : "-");
      line(gui, 2u, "PROTOCOL", profile != NULL ? profile->protocol : "-");
      number(value, sizeof(value), profile != NULL ? profile->ram_bytes / 1024u : 0u); line(gui, 3u, "RAM KB", value);
      number(value, sizeof(value), profile != NULL ? profile->max_frame_rate : 0u); line(gui, 4u, "MAX FPS", value);
      number(value, sizeof(value), profile != NULL ? profile->calibration_generation : 0u); line(gui, 5u, "CAL GEN", value);
      oled_ssd1315_draw_text(gui->oled, 0u, 56u, "K1 NEXT FAULT", 1u);
      break;
  }
}

static void render_control(device_gui_t *gui)
{
  line(gui, 0u, "START", "PLAN");
  line(gui, 1u, "STOP", "OUTPUT");
  line(gui, 2u, "CLEAR", "PLAN");
  line(gui, 3u, "TRIGGER", "WAIT");
  oled_ssd1315_draw_text(gui->oled, 0u, 49u, "CONFIRM ACTION", 0u);
}

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
                     void *action_context)
{
  if (gui == NULL) return;
  memset(gui, 0, sizeof(*gui));
  gui->oled = oled; gui->profile = profile; gui->status = status; gui->plan = plan;
  gui->fpga = fpga; gui->flash = flash; gui->eeprom = eeprom;
  gui->start = start; gui->stop = stop; gui->clear = clear; gui->trigger = trigger;
  gui->action_context = action_context; gui->page = DEVICE_GUI_HOME; gui->debug_view = 0u;
}

void device_gui_handle_event(device_gui_t *gui, const input_event_t *event)
{
  if (gui == NULL || event == NULL || event->type != INPUT_EVENT_PRESS) return;
  if (event->key == INPUT_KEY0) { gui->page = DEVICE_GUI_HOME; gui->row = 0u; return; }
  if (event->key == INPUT_KEY2) {
    if (gui->page == DEVICE_GUI_CONTROL) gui->row = (uint8_t)((gui->row + 1u) % 4u);
    else gui->page = (device_gui_page_t)((gui->page + 1u) % DEVICE_GUI_PAGE_COUNT);
    return;
  }
  if (event->key == INPUT_KEY3) {
    if (gui->page == DEVICE_GUI_CONTROL) gui->row = gui->row == 0u ? 3u : (uint8_t)(gui->row - 1u);
    else gui->page = gui->page == DEVICE_GUI_HOME ? (device_gui_page_t)(DEVICE_GUI_PAGE_COUNT - 1u) : (device_gui_page_t)(gui->page - 1u);
    return;
  }
  if (event->key == INPUT_KEY1 && gui->page == DEVICE_GUI_DIAGNOSTICS) {
    gui->debug_view = (uint8_t)((gui->debug_view + 1u) % 5u);
    return;
  }
  if (event->key == INPUT_KEY1 && gui->page == DEVICE_GUI_CONTROL) {
    int result = -1;
    if (gui->row == 0u && gui->start != NULL) result = gui->start(gui->action_context);
    else if (gui->row == 1u && gui->stop != NULL) result = gui->stop(gui->action_context);
    else if (gui->row == 2u && gui->clear != NULL) result = gui->clear(gui->action_context);
    else if (gui->row == 3u && gui->trigger != NULL) result = gui->trigger(gui->action_context);
    gui->action_message = result == 0 ? 1u : 2u; gui->message_until = HAL_GetTick() + GUI_MESSAGE_MS;
  }
}

void device_gui_render(device_gui_t *gui, uint32_t now_ms)
{
  if (gui == NULL || gui->oled == NULL) return;
  oled_ssd1315_clear(gui->oled);
  oled_ssd1315_draw_text(gui->oled, 0u, GUI_TITLE_Y, page_title(gui->page), 1u);
  switch (gui->page) {
    case DEVICE_GUI_HOME: render_home(gui); break;
    case DEVICE_GUI_PLAYBACK: render_playback(gui); break;
    case DEVICE_GUI_DEVICE: render_device(gui); break;
    case DEVICE_GUI_CALIBRATION: render_calibration(gui); break;
    case DEVICE_GUI_STORAGE: render_storage(gui); break;
    case DEVICE_GUI_DIAGNOSTICS: render_diagnostics(gui); break;
    case DEVICE_GUI_CONTROL: render_control(gui); break;
    default: break;
  }
  if (gui->action_message != 0u && (int32_t)(gui->message_until - now_ms) > 0)
    oled_ssd1315_draw_text(gui->oled, 78u, 56u, gui->action_message == 1u ? "OK" : "ERR", 1u);
  else gui->action_message = 0u;
}
