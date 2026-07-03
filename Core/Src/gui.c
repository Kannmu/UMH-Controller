/* GUI — Multi-level menu framework (128x32 landscape OLED, SSD1306)
 *
 * Navigation: UP/DOWN = cursor, CONFIRM = enter/execute, RETURN = back/cancel.
 * Layout: left 10px sidebar (vertical title) + 118px scrollable content area.
 * Root page: 1-column N-row scrollable list — data slots + folder buttons.
 */
#include "gui.h"
#include "font.h"
#include "buttons.h"
#include "dma_manager.h"
#include "stimulation.h"
#include "calibration.h"
#include "utiles.h"
#include "calib_semi.h"
#include "calib_adc.h"
#include "eeprom.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ---- Layout constants ---- */
#define SIDEBAR_W     10
#define CONTENT_X     (SIDEBAR_W + 2)
#define CONTENT_W     (GUI_WIDTH - CONTENT_X)
#define HEADER_H      10
#define ROW_H         10
#define VISIBLE_ROWS  2
#define STACK_DEPTH   6

/* ---- Animation tuning ---- */
#define SCROLL_SPEED   14.0f       /* exponential decay rate (higher = faster) */
#define CURSOR_SPEED   18.0f

/* ---- State ---- */
static const MenuPage *nav_stack[STACK_DEPTH];
static uint8_t  nav_depth;
static uint8_t  nav_cursor;
static int16_t  target_scroll;
static float    scroll_y_smooth;
static float    cursor_y_smooth;
static uint8_t  editing, edit_val_idx;
static int16_t  slide_x;
static int8_t   slide_dir;
static uint32_t slide_start;
static uint32_t last_frame_us;
static uint8_t  gui_dirty;

#define SLIDE_DURATION 200

static float    cached_temp, cached_vdda;
static uint32_t last_adc_read;

/* ---- Forward decls for page input handlers ---- */
static bool root_on_input(NavAction nav);
static bool demo_on_input(NavAction nav);
static bool calib_on_input(NavAction nav);
static bool about_on_input(NavAction nav);
static bool semi_calib_on_input(NavAction nav);
static bool refresh_on_input(NavAction nav);
static void refresh_on_event(PageEvent event);

/* ================================================================
 *  Smart float formatting (no %f — nano.specs disables float printf)
 * ================================================================ */
static void format_fixed(char *buf, size_t sz, float val, uint8_t dec)
{
    if (sz == 0) return;
    buf[0] = '\0';  /* ensure null-terminated on early-return paths */
    if (dec > 6) dec = 6;
    float mul = 1.0f;
    for (uint8_t d = 0; d < dec; d++) mul *= 10.0f;
    /* Use 64-bit math to avoid overflow for large magnitudes (e.g. val=3000,
     * dec=6 -> 3e9 which exceeds int32_t range). int64_t is more than enough
     * for any reasonable float magnitude at dec<=6. */
    int64_t sv = (int64_t)(val * mul + (val >= 0 ? 0.5f : -0.5f));
    int64_t ip = sv / (int64_t)mul;
    uint64_t fp = (uint64_t)(sv >= 0 ? sv : -sv) % (uint64_t)mul;

    char tmp[16]; int pos = 0;
    int64_t n = ip;
    if (n == 0) { tmp[pos++] = '0'; }
    else {
        if (n < 0) { n = -n; }
        int64_t m = n; int digits = 0;
        while (m > 0) { digits++; m /= 10; }
        pos = digits;
        for (int i = digits-1; i >= 0; i--) { tmp[i] = (char)('0'+(int)(n%10)); n /= 10; }
    }
    if (sz < (size_t)(pos + 1 + dec + 1)) return;
    size_t w = 0;
    if (ip < 0) ((char*)buf)[w++] = '-';
    for (int i = 0; i < pos; i++) ((char*)buf)[w++] = tmp[i];
    if (dec > 0) {
        ((char*)buf)[w++] = '.';
        for (int d = dec-1; d >= 0; d--) {
            uint64_t p10 = 1;
            for (int x = 0; x < d; x++) p10 *= 10;
            ((char*)buf)[w++] = (char)('0' + (int)((fp / p10) % 10));
        }
    }
    ((char*)buf)[w] = '\0';
}

void GUI_FormatSmartUnits(char *buf, size_t sz, float value, const char *suffix, uint8_t dec)
{
    float av = fabsf(value);
    const char *pfx = "";
    float sc = value;
    if      (av >= 1000.0f)      { pfx = "k"; sc = value / 1000.0f; }
    else if (av >= 1.0f)         { /* native */ }
    else if (av >= 0.001f)       { pfx = "m"; sc = value * 1000.0f; }
    else if (av >= 0.000001f)    { pfx = "u"; sc = value * 1000000.0f; }
    else if (av >= 0.000000001f) { pfx = "n"; sc = value * 1000000000.0f; }
    else if (av < 0.000000001f && av > 0.0f) { sc = 0.0f; }
    char num[20];
    format_fixed(num, sizeof(num), sc, dec);
    snprintf(buf, sz, "%s %s%s", num, pfx, suffix);
}

/* ---- Draw vertical sidebar title (one char per row, top→down) ----
 * Vertical sidebar fits at most 4 chars (32px / 7px per char). Use short
 * aliases for longer page titles so they are not visually truncated
 * (e.g. "ABOUT"→"INF", "CALIB"→"CAL", "REFRESH"→"RFS", "SEMI CAL"→"SEM"). */
static const char *sidebar_alias(const char *title)
{
    static const struct { const char *full; const char *alias; } aliases[] = {
        {"ABOUT",     "INF"},
        {"CALIB",     "CAL"},
        {"REFRESH",   "RFS"},
        {"SEMI CAL",  "SEM"},
    };
    for (size_t i = 0; i < sizeof(aliases)/sizeof(aliases[0]); i++) {
        if (strcmp(title, aliases[i].full) == 0) return aliases[i].alias;
    }
    return title;
}
static void draw_sidebar(const char *title)
{
    const char *t = sidebar_alias(title);
    SSD1306_FillRect(0, 0, SIDEBAR_W, GUI_HEIGHT, WHITE);
    int len = (int)strlen(t);
    int max_chars = GUI_HEIGHT / 7;
    if (len > max_chars) len = max_chars;
    int start_y = (GUI_HEIGHT - len * 7) / 2;
    if (start_y < 0) start_y = 0;
    for (int i = 0; i < len; i++) {
        Font_DrawChar(SIDEBAR_W / 2 - 2, start_y + i * 7, t[i], 0, 1, BLACK);
    }
}

/* ---- Data slot descriptor table ---- */
typedef struct { const char *suffix; uint8_t decimals; } DataSlotDesc;
static const DataSlotDesc data_slots[] = {
    [0] = {"Hz",  3},  /* Refresh Rate */
    [1] = {"Hz",  0},  /* Sys Loop Freq */
    [2] = {"C",   1},  /* Temperature */
    [3] = {"V",   3},  /* VDDA */
    [4] = {"ms",  3},  /* DMA Time */
    [5] = {"",    0},  /* Cal Mode */
    [6] = {"",    0},  /* Stim State */
    [7] = {"",    0},  /* Demo Name */
    [8] = {"",    0},  /* Phase Mode */
};

/* ---- Action callbacks ---- */
void GUI_Action_ToggleCalibBypass(void) {
    calibration_mode = 1 - Get_Calibration_Mode();
    if (calibration_mode) Enter_Calibration_Mode();
    else                  Load_Calib_to_Transducers();
    Update_Full_Waveform_Buffer();
}
void GUI_Action_SetDemo(int idx) {
    if (idx < 0 || idx >= Get_Num_Demo_Stimulations()) return;
    const StimDemoDescriptor *d = Stim_Get_Demo_By_Index((uint8_t)idx);
    if (!d) return;
    demo_mode = idx;
    Set_Stimulation_From_Demo(d);
    phase_set_mode = 0;
}
void GUI_Action_StartSemiAutoCalib(void) {
    SemiCalib_Init();
    calib_state = CALIB_PROMPT;
    calib_current_element = 0;
    GUI_PushPage(&Page_SemiCalib);
}

/* ---- Navigation ---- */
void GUI_PushPage(const MenuPage *page) {
    if (nav_depth < STACK_DEPTH) {
        nav_stack[nav_depth++] = page;
        nav_cursor = 0; target_scroll = 0; scroll_y_smooth = 0; cursor_y_smooth = HEADER_H;
        editing = 0; gui_dirty = 1;
        slide_x = GUI_WIDTH; slide_dir = -1;
        slide_start = HAL_GetTick();
        if (page->on_event) page->on_event(EVENT_ENTER);
    }
}
void GUI_PopPage(void) {
    if (nav_depth > 1) {
        if (nav_stack[nav_depth-1]->on_event) nav_stack[nav_depth-1]->on_event(EVENT_EXIT);
        nav_depth--;
        nav_cursor = 0; target_scroll = 0; scroll_y_smooth = 0; cursor_y_smooth = HEADER_H;
        editing = 0; gui_dirty = 1;
        slide_x = -GUI_WIDTH; slide_dir = 1;
        slide_start = HAL_GetTick();
    }
}
const MenuPage* GUI_CurrentPage(void) {
    return (nav_depth > 0) ? nav_stack[nav_depth-1] : NULL;
}

/* ================================================================
 *  PAGE DEFINITIONS
 * ================================================================ */
extern const MenuPage Page_Root, Page_Demo, Page_Calibration, Page_About;
extern volatile int calibration_mode;

/* --- Root page (12 rows: 9 data slots + 3 folders) --- */
static const MenuItem items_root[] = {
    {"Refresh",  MENU_DATA_FLOAT, .df={0}},
    {"Sys Freq", MENU_DATA_FLOAT, .df={1}},
    {"Temp",     MENU_DATA_FLOAT, .df={2}},
    {"VDDA",     MENU_DATA_FLOAT, .df={3}},
    {"DMA Time", MENU_DATA_FLOAT, .df={4}},
    {"Cal Mode", MENU_DATA_FLOAT, .df={5}},
    {"Stim St",  MENU_DATA_FLOAT, .df={6}},
    {"Demo",     MENU_DATA_FLOAT, .df={7}},
    {"Phs Mode", MENU_DATA_FLOAT, .df={8}},
    {"DEMO",     MENU_FOLDER, .submenu=&Page_Demo},
    {"CALIB",    MENU_FOLDER, .submenu=&Page_Calibration},
    {"ABOUT",    MENU_FOLDER, .submenu=&Page_About},
};
const MenuPage Page_Root = {"UMH", items_root, 12, 0, root_on_input, NULL};

/* --- Root input handler --- */
static bool root_on_input(NavAction nav)
{
    int max_item = (int)Page_Root.item_count - 1;

    if (editing) {
        const MenuItem *it = &Page_Root.items[edit_val_idx];
        if (it->type == MENU_VALUE_INT || it->type == MENU_VALUE_ENUM) {
            if (nav == NAV_UP)   { int v=*it->iv.value+it->iv.step; if(v<=it->iv.max)*it->iv.value=v; gui_dirty=1; }
            if (nav == NAV_DOWN) { int v=*it->iv.value-it->iv.step; if(v>=it->iv.min)*it->iv.value=v; gui_dirty=1; }
            if (nav == NAV_CONFIRM || nav == NAV_RETURN) { editing = 0; gui_dirty = 1; }
        } else { editing = 0; }
        return true;
    }

    if (nav == NAV_UP   && nav_cursor > 0)                  { nav_cursor--; gui_dirty = 1; return true; }
    if (nav == NAV_DOWN && nav_cursor < (uint8_t)max_item)  { nav_cursor++; gui_dirty = 1; return true; }

    if (nav == NAV_CONFIRM) {
        const MenuItem *it = &Page_Root.items[nav_cursor];
        switch (it->type) {
        case MENU_FOLDER: if (it->submenu) GUI_PushPage(it->submenu); break;
        case MENU_ACTION: if (it->action)  it->action(); break;
        case MENU_VALUE_INT: case MENU_VALUE_ENUM:
            editing=1; edit_val_idx=nav_cursor; gui_dirty=1; break;
        case MENU_DATA_FLOAT:
            if (it->df.slot_idx == 0) GUI_PushPage(&Page_Refresh);
            break;
        case MENU_BACK: GUI_PopPage(); break;
        default: break;
        }
        return true;
    }

    if (nav == NAV_RETURN) { /* root: ignore */ return true; }
    return false;
}

/* --- About page --- */
static void render_about(const void *ctx) {
    (void)ctx;
    draw_sidebar("A");
    Font_DrawStr(CONTENT_X, 0, "UMH V5.5", 0, 1, WHITE);
    SSD1306_DrawHLine(CONTENT_X, 8, CONTENT_W, WHITE);
    Font_DrawStr(CONTENT_X, 10, "Designed by", 0, 1, WHITE);
    Font_DrawStr(CONTENT_X, 18, "Kannmu @SEU", 0, 1, WHITE);
    Font_DrawStr(CONTENT_X, 26, "[<] back", 0, 1, WHITE);
}
static const MenuItem items_about[] = {
    {"ABOUT", MENU_ACTION, .action=NULL, .custom_render=render_about},
};
const MenuPage Page_About = {"ABOUT", items_about, 1, 0, about_on_input, NULL};

static bool about_on_input(NavAction nav) {
    if (nav == NAV_RETURN) { GUI_PopPage(); return true; }
    return false;
}

/* --- Refresh Rate page (dynamic: one row per StimulationType + back) --- */

static void render_refresh(const void *ctx) {
    (void)ctx;
    draw_sidebar("R");
    Font_DrawStr(CONTENT_X, 0, "REFRESH RATE", 0, 1, WHITE);
    SSD1306_DrawHLine(CONTENT_X, HEADER_H - 2, CONTENT_W, WHITE);

    uint8_t ntypes = Stim_Num_Types();
    if (ntypes > STIM_MAX_TYPES) ntypes = STIM_MAX_TYPES;
    for (uint8_t i = 0; i < ntypes; i++) {
        int16_t y = HEADER_H + (int16_t)i * ROW_H - (int16_t)scroll_y_smooth;
        if (y < HEADER_H - ROW_H || y > GUI_HEIGHT) continue;
        uint8_t cur = (i == nav_cursor);
        Colour bg = cur ? WHITE : BLACK;
        Colour fg = cur ? BLACK : WHITE;
        SSD1306_FillRect(CONTENT_X, y, CONTENT_W, ROW_H - 1, bg);

        char buf[36];
        double dma_ms = updateDMABufferDeltaTimeByType[i];
        float rate = (dma_ms > 0.0) ? (float)(1000.0 / dma_ms) : 0.0f;
        const StimTypeDescriptor *td = Stim_Get_Type_By_Index(i);
        snprintf(buf, sizeof(buf), "%s", td ? td->name : "?");
        Font_DrawStr(CONTENT_X + 1, y, buf, 0, 1, fg);

        char val[20];
        GUI_FormatSmartUnits(val, sizeof(val), rate, "Hz", 1);
        Font_DrawStr(GUI_WIDTH - Font_StrWidth(val, 0, 1) - 1, y, val, 0, 1, fg);
    }
}
static const MenuItem items_refresh[] = {
    {"REF", MENU_ACTION, .action=NULL, .custom_render=render_refresh},
};
MenuPage Page_Refresh = {"REFRESH", items_refresh, 1, 0, refresh_on_input, refresh_on_event};

static bool refresh_on_input(NavAction nav) {
    uint8_t ntypes = Stim_Num_Types();
    if (nav == NAV_UP   && nav_cursor > 0)           { nav_cursor--; gui_dirty = 1; return true; }
    if (nav == NAV_DOWN && nav_cursor < ntypes - 1)  { nav_cursor++; gui_dirty = 1; return true; }
    if (nav == NAV_RETURN) { GUI_PopPage(); return true; }
    return false;
}

static void refresh_on_event(PageEvent event) {
    if (event != EVENT_ENTER) return;
    /* Update scroll_rows to match the current number of registered types */
    Page_Refresh.scroll_rows = Stim_Num_Types();
    /* Cycle through every registered StimulationType to capture per-type timing */
    uint8_t ntypes = Stim_Num_Types();
    for (uint8_t i = 0; i < ntypes; i++) {
        const StimTypeDescriptor *td = Stim_Get_Type_By_Index(i);
        if (!td) continue;
        Stimulation s;
        memset(&s, 0, sizeof(s));
        strncpy(s.name, td->name, sizeof(s.name) - 1);
        s.type_id   = td->type_id;
        s.type_desc = td;
        s.strength  = DMA_STRENGTH_MAX;
        s.frequency = (float)STIMULATION_FREQ;
        if (td->init) td->init(&s);
        Set_Stimulation(&s);
    }
    /* Restore the original stimulation from the current demo (or empty default) */
    if (demo_mode >= 0) {
        const StimDemoDescriptor *d = Stim_Get_Demo_By_Index((uint8_t)demo_mode);
        if (d) Set_Stimulation_From_Demo(d);
    } else {
        /* restore a sensible default */
        const StimTypeDescriptor *td0 = Stim_Get_Type_By_Index(0);
        if (td0) {
            Stimulation s;
            memset(&s, 0, sizeof(s));
            strncpy(s.name, td0->name, sizeof(s.name) - 1);
            s.type_id   = td0->type_id;
            s.type_desc = td0;
            s.strength  = DMA_STRENGTH_MAX;
            s.frequency = (float)STIMULATION_FREQ;
            if (td0->init) td0->init(&s);
            Set_Stimulation(&s);
        }
    }
}

/* --- Demo page (N choices + back, scrollable, dynamic from registry) --- */
static void render_demo(const void *ctx) {
    (void)ctx;
    draw_sidebar("D");
    Font_DrawStr(CONTENT_X, 0, "SELECT DEMO", 0, 1, WHITE);
    SSD1306_DrawHLine(CONTENT_X, HEADER_H - 2, CONTENT_W, WHITE);

    uint8_t num = Stim_Num_Demos();
    for (uint8_t i = 0; i < num; i++) {
        int16_t y = HEADER_H + (int16_t)i * ROW_H - (int16_t)scroll_y_smooth;
        if (y < HEADER_H - ROW_H || y > GUI_HEIGHT) continue;
        const StimDemoDescriptor *d = Stim_Get_Demo_By_Index(i);
        uint8_t active = (demo_mode == (int)i);
        uint8_t cur = (i == nav_cursor);
        Colour bg = cur ? WHITE : BLACK;
        Colour fg = cur ? BLACK : WHITE;
        SSD1306_FillRect(CONTENT_X, y, CONTENT_W, ROW_H - 1, bg);
        char buf[22];
        snprintf(buf, sizeof(buf), "%s%s", active ? ">" : " ", d ? d->name : "?");
        Font_DrawStr(CONTENT_X + 1, y, buf, 0, 1, fg);
    }
    /* BACK row */
    int16_t yy = HEADER_H + num * ROW_H - (int16_t)scroll_y_smooth;
    {
        uint8_t cur = (nav_cursor == num);
        Colour bg = cur ? WHITE : BLACK;
        Colour fg = cur ? BLACK : WHITE;
        SSD1306_FillRect(CONTENT_X, yy, CONTENT_W, ROW_H - 1, bg);
        Font_DrawStr(CONTENT_X + 1, yy, "< BACK", 0, 1, fg);
    }
}
static const MenuItem items_demo[] = {
    {"DEMO", MENU_ACTION, .action=NULL, .custom_render=render_demo},
};
const MenuPage Page_Demo = {"DEMO", items_demo, 1, 6, demo_on_input, NULL};

static bool demo_on_input(NavAction nav) {
    uint8_t num = Stim_Num_Demos();
    if (nav == NAV_UP   && nav_cursor > 0)           { nav_cursor--; gui_dirty = 1; return true; }
    if (nav == NAV_DOWN && nav_cursor < num)         { nav_cursor++; gui_dirty = 1; return true; }
    if (nav == NAV_CONFIRM) {
        if (nav_cursor < num) GUI_Action_SetDemo((int)nav_cursor);
        else                  GUI_PopPage();
        return true;
    }
    if (nav == NAV_RETURN) { GUI_PopPage(); return true; }
    return false;
}

/* --- Calibration page (3 rows, scrollable) --- */
static void render_calib(const void *ctx) {
    (void)ctx;
    draw_sidebar("C");
    Font_DrawStr(CONTENT_X, 0, "CALIBRATION", 0, 1, WHITE);
    SSD1306_DrawHLine(CONTENT_X, HEADER_H - 2, CONTENT_W, WHITE);

    static const char *labels[] = {"LOAD from EEPROM", "BYPASS", "SEMI-AUTO"};
    for (int i = 0; i < 3; i++) {
        int16_t y = HEADER_H + (int16_t)i * ROW_H - (int16_t)scroll_y_smooth;
        if (y < HEADER_H - ROW_H || y > GUI_HEIGHT) continue;
        uint8_t cur = (i == (int)nav_cursor);
        Colour bg = cur ? WHITE : BLACK;
        Colour fg = cur ? BLACK : WHITE;
        SSD1306_FillRect(CONTENT_X, y, CONTENT_W, ROW_H - 1, bg);
        char buf[26];
        if (i == 1)
            snprintf(buf, sizeof(buf), "%s [%s]", labels[i], calibration_mode ? "ON" : "OFF");
        else
            snprintf(buf, sizeof(buf), "%s", labels[i]);
        Font_DrawStr(CONTENT_X + 1, y, buf, 0, 1, fg);
    }
}
static const MenuItem items_cal[] = {
    {"CAL", MENU_ACTION, .action=NULL, .custom_render=render_calib},
};
const MenuPage Page_Calibration = {"CALIB", items_cal, 1, 3, calib_on_input, NULL};

static bool calib_on_input(NavAction nav) {
    if (nav == NAV_UP   && nav_cursor > 0) { nav_cursor--; gui_dirty = 1; return true; }
    if (nav == NAV_DOWN && nav_cursor < 2) { nav_cursor++; gui_dirty = 1; return true; }

    if (nav == NAV_CONFIRM) {
        if (nav_cursor == 0) {
            extern float Transducer_Calibration_Array[];
            float ecal[NUM_REAL_TRANSDUCER];
            if (EEPROM_LoadCalibration(ecal))
                for (int i=0;i<NUM_REAL_TRANSDUCER;i++) Transducer_Calibration_Array[i]=ecal[i];
            Load_Calib_to_Transducers();
            Update_Full_Waveform_Buffer();
        } else if (nav_cursor == 1) {
            GUI_Action_ToggleCalibBypass();
        } else {
            GUI_Action_StartSemiAutoCalib();
        }
        gui_dirty = 1;
        return true;
    }
    if (nav == NAV_RETURN) { GUI_PopPage(); return true; }
    return false;
}

/* --- Semi-auto calibration page --- */
static void render_semi_calib(const void *ctx) {
    (void)ctx; char buf[22];
    draw_sidebar("S");
    switch (calib_state) {
    case CALIB_PROMPT:
        Font_DrawStr(CONTENT_X, 0, "SEMI-AUTO CAL", 0, 1, WHITE);
        snprintf(buf,sizeof(buf),"Elem %d/%u", calib_current_element+1, (unsigned int)NUM_REAL_TRANSDUCER);
        Font_DrawStr(CONTENT_X, 8, buf, 0, 1, WHITE);
        Font_DrawStr(CONTENT_X, 16, "Place probe", 0, 1, WHITE);
        Font_DrawStr(CONTENT_X, 24, "[OK]go [<]skip", 0, 1, WHITE);
        break;
    case CALIB_MEASURING: {
        Font_DrawStr(CONTENT_X, 0, "Measuring...", 0, 1, WHITE);
        char sp[]={'|','/','-','\\'}, s2[2]={sp[(HAL_GetTick()/200)%4],0};
        Font_DrawStr(CONTENT_X+50, 12, s2, 0, 1, WHITE);
        break;
    }
    case CALIB_SHOW_RESULT:
        Font_DrawStr(CONTENT_X, 0, "RESULT", 0, 1, WHITE);
        snprintf(buf,sizeof(buf),"N%d: %.2fus",calib_current_element+1,
                 (double)calib_results[calib_current_element].calib_us);
        Font_DrawStr(CONTENT_X, 8, buf, 0, 1, WHITE);
        { const char *q = calib_results[calib_current_element].quality>=2?"OK":
                           calib_results[calib_current_element].quality>=1?"LOW":"BAD";
          snprintf(buf,sizeof(buf),"Amp:%s",q); }
        Font_DrawStr(CONTENT_X, 16, buf, 0, 1, WHITE);
        Font_DrawStr(CONTENT_X, 24, "[OK]next [<]retry", 0, 1, WHITE);
        break;
    case CALIB_DONE:
        /* Transient: CALIB_DONE immediately hands off to CALIB_SAVING.
         * Rendered only if a frame happens to be drawn between the two. */
        Font_DrawStr(CONTENT_X, 0, "CAL COMPLETE", 0, 1, WHITE);
        Font_DrawStr(CONTENT_X, 10, "Saving...", 0, 1, WHITE);
        break;
    case CALIB_SAVING: {
        Font_DrawStr(CONTENT_X, 0, "CAL COMPLETE", 0, 1, WHITE);
        char sp[]={'|','/','-','\\'}, s2[2]={sp[(HAL_GetTick()/200)%4],0};
        Font_DrawStr(CONTENT_X, 10, "Saving", 0, 1, WHITE);
        Font_DrawStr(CONTENT_X+50, 12, s2, 0, 1, WHITE);
        break;
    }
    case CALIB_ERROR:
        Font_DrawStr(CONTENT_X, 0, "ERROR", 0, 1, WHITE);
        Font_DrawStr(CONTENT_X, 12, "Save failed", 0, 1, WHITE);
        break;
    default: break;
    }
}
static const MenuItem items_semi_calib[] = {
    {"CAL", MENU_ACTION, .action=NULL, .custom_render=render_semi_calib},
};
const MenuPage Page_SemiCalib = {"SEMI CAL", items_semi_calib, 1, 0, semi_calib_on_input, NULL};

static bool semi_calib_on_input(NavAction nav) {
    /* SemiCalib state machine controls button routing. CALIB_SAVING
     * (EEPROM write in progress) swallows all input so the user cannot
     * pop the page mid-save. IDLE/DONE/ERROR exit on RETURN or CONFIRM. */
    switch (calib_state) {
    case CALIB_PROMPT:
    case CALIB_MEASURING:
    case CALIB_SHOW_RESULT:
        /* Active calibration: forward button events to the state machine. */
        if (nav == NAV_CONFIRM)      calib_button_pressed = 1;
        else if (nav == NAV_RETURN)  calib_button_pressed = 2;
        gui_dirty = 1;
        return true;
    case CALIB_SAVING:
        /* Saving in progress: consume input without effect. */
        gui_dirty = 1;
        return true;
    case CALIB_IDLE:
    case CALIB_ERROR:
    case CALIB_DONE:
    default:
        /* IDLE / DONE / ERROR: RETURN or CONFIRM exits the page. */
        if (nav == NAV_RETURN || nav == NAV_CONFIRM) { GUI_PopPage(); return true; }
        return false;
    }
}

/* ================================================================
 *  GUI INIT
 * ================================================================ */
void GUI_Init(void)
{
    if (SSD1306_Init() != 0) {
        for (volatile int i = 0; i < 10; i++)
            HAL_GPIO_TogglePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin);
        return;
    }
    Buttons_Init();
    Buttons_Tick();
    nav_depth = 0; nav_cursor = 0;
    target_scroll = 0; scroll_y_smooth = 0; cursor_y_smooth = HEADER_H;
    editing = 0; gui_dirty = 1;
    slide_x = 0; slide_dir = 0;
    cached_temp = 0; cached_vdda = 0; last_adc_read = 0;
    last_frame_us = HAL_GetTick() * 1000;
    GUI_PushPage(&Page_Root);
}

/* ================================================================
 *  GUI TICK — main loop (~30Hz)
 * ================================================================ */
void GUI_Tick(void)
{
    const MenuPage *page = GUI_CurrentPage();
    if (!page) return;

    uint32_t now = HAL_GetTick();

    /* ADC cache (~10Hz) */
    if (now - last_adc_read >= 100) {
        float t = Get_Temperature();
        float v = Get_Voltage_VDDA();
        if (fabsf(cached_temp - t) > 0.05f || fabsf(cached_vdda - v) > 0.005f) {
            cached_temp = t;
            cached_vdda = v;
            gui_dirty = 1;
        }
        last_adc_read = now;
    }

    /* Tick calibration state machine */
    SemiCalib_Tick();

    /* ---- Delta-time animation ---- */
    float dt = (float)(now * 1000 - last_frame_us) / 1000000.0f;
    last_frame_us = now * 1000;
    if (dt > 0.05f) dt = 0.05f;  /* clamp: avoid jump on first frame / stall */

    /* Slide animation (page transitions) */
    if (slide_dir != 0) {
        uint32_t elapsed = now - slide_start;
        if (elapsed >= SLIDE_DURATION) { slide_x = 0; slide_dir = 0; }
        else {
            float t = (float)elapsed / (float)SLIDE_DURATION;
            t = t * t * (3.0f - 2.0f * t);
            if (slide_dir < 0) slide_x =   GUI_WIDTH - (int16_t)((float)GUI_WIDTH * t);
            else               slide_x = -(GUI_WIDTH - (int16_t)((float)GUI_WIDTH * t));
        }
        gui_dirty = 1;
    }

    /* Scroll + cursor smoothing */
    {
        int row_count = (page->scroll_rows > 0) ? (int)page->scroll_rows
                      : (page->items[0].custom_render) ? 0
                      : (int)page->item_count;

        if (row_count > 0) {
            target_scroll = (int16_t)nav_cursor * ROW_H - (int16_t)((VISIBLE_ROWS/2) * ROW_H);
            if (target_scroll < 0) target_scroll = 0;
            int16_t max_scroll = (int16_t)(row_count - VISIBLE_ROWS) * ROW_H + ROW_H;
            if (max_scroll < 0) max_scroll = 0;
            if (target_scroll > max_scroll) target_scroll = max_scroll;
        }

        float blend = 1.0f - expf(-SCROLL_SPEED * dt);
        if (blend > 1.0f) blend = 1.0f;
        float prev = scroll_y_smooth;
        scroll_y_smooth += (target_scroll - scroll_y_smooth) * blend;
        if (fabsf(scroll_y_smooth - target_scroll) < 0.3f) scroll_y_smooth = target_scroll;
        if (fabsf(prev - scroll_y_smooth) > 0.1f) gui_dirty = 1;

        /* Cursor position smoothing */
        int target_cursor_y = HEADER_H + (int)nav_cursor * ROW_H - (int)scroll_y_smooth;
        float cblend = 1.0f - expf(-CURSOR_SPEED * dt);
        if (cblend > 1.0f) cblend = 1.0f;
        float cprev = cursor_y_smooth;
        cursor_y_smooth += (target_cursor_y - cursor_y_smooth) * cblend;
        if (fabsf(cursor_y_smooth - target_cursor_y) < 0.3f) cursor_y_smooth = target_cursor_y;
        if (fabsf(cprev - cursor_y_smooth) > 0.1f) gui_dirty = 1;
    }

    /* ---- Button input: delegate to page handler ---- */
    {
        NavAction nav = Buttons_GetNav();
        if (nav != NAV_NONE) gui_dirty = 1;
        if (page->on_input) {
            page->on_input(nav);
        }
    }

    /* ---- Render (only when dirty and at min 30ms intervals) ---- */
    {
        static uint32_t last_render;
        if (now - last_render < 30) return;
        if (!gui_dirty) return;

        SSD1306_Fill(BLACK);
        const MenuItem *it = page->items;
        int count = (int)page->item_count;

        if (page->items[0].custom_render) {
            page->items[0].custom_render((void*)page);
        } else {
            /* Standard sidebar + list layout */
            draw_sidebar(page->title);
            int max_item = count - 1;
            if (page->items[count-1].type == MENU_BACK) max_item = count - 2;

            for (int i = 0; i < count && i <= max_item; i++) {
                int16_t y = HEADER_H + (int16_t)i * ROW_H - (int16_t)scroll_y_smooth;
                if (y < HEADER_H - ROW_H || y > GUI_HEIGHT) continue;

                const char *label = it[i].label;
                if (it[i].type == MENU_BACK) label = "<";

                uint8_t cur = (i == (int)nav_cursor && !editing);
                Colour bg = cur ? WHITE : BLACK;
                Colour fg = cur ? BLACK : WHITE;

                SSD1306_FillRect(CONTENT_X, y, CONTENT_W, ROW_H - 1, bg);

                if (it[i].type == MENU_DATA_FLOAT) {
                    char buf[36];
                    uint8_t slot = it[i].df.slot_idx;
                    Font_DrawStr(CONTENT_X + 1, y, label, 0, 1, fg);

                    if (slot == 5) {
                        snprintf(buf, sizeof(buf), "%s", calibration_mode ? "ON" : "OFF");
                    } else if (slot == 6) {
                        if (Get_Stimulation_Enabled())
                            snprintf(buf, sizeof(buf), "%s", CurrentStimulation.name);
                        else
                            snprintf(buf, sizeof(buf), "OFF");
                    } else if (slot == 7) {
                        const StimDemoDescriptor *d = Stim_Get_Demo_By_Index((uint8_t)demo_mode);
                        if (d)
                            snprintf(buf, sizeof(buf), "%s", d->name);
                        else
                            snprintf(buf, sizeof(buf), "None");
                    } else if (slot == 8) {
                        snprintf(buf, sizeof(buf), "%s", Get_Phase_Set_Mode() ? "Custom" : "Auto");
                    } else {
                        float val = 0.0f;
                        switch (slot) {
                        case 0: val = Get_Refresh_Rate(); break;
                        case 1: val = System_Loop_Freq; break;
                        case 2: val = cached_temp; break;
                        case 3: val = cached_vdda; break;
                        case 4: val = (float)updateDMABufferDeltaTime; break;
                        default: break;
                        }
                        GUI_FormatSmartUnits(buf, sizeof(buf), val,
                                             data_slots[slot].suffix, data_slots[slot].decimals);
                    }
                    Font_DrawStr(GUI_WIDTH - Font_StrWidth(buf,0,1) - 1, y, buf, 0, 1, fg);
                } else {
                    Font_DrawStr(CONTENT_X + 1, y, label, 0, 1, fg);
                    if (it[i].type == MENU_VALUE_INT) {
                        char b[8]; snprintf(b,sizeof(b),it[i].iv.fmt,*it[i].iv.value);
                        Font_DrawStr(GUI_WIDTH-Font_StrWidth(b,0,1)-1, y, b, 0, 1, fg);
                    } else if (it[i].type == MENU_VALUE_ENUM && *it[i].ev.value < it[i].ev.count) {
                        const char *v=it[i].ev.names[*it[i].ev.value];
                        Font_DrawStr(GUI_WIDTH-Font_StrWidth(v,0,1)-1, y, v, 0, 1, fg);
                    }
                }
            }

            /* Scroll indicator */
            int row_count = (page->scroll_rows > 0) ? (int)page->scroll_rows : count;
            int vis = VISIBLE_ROWS;
            int16_t ms = (int16_t)(row_count - vis) * ROW_H + ROW_H;
            if (ms > 0 && row_count > vis) {
                int16_t bh = vis * ROW_H * vis / row_count;
                if (bh < 4) bh = 4;
                int16_t by = HEADER_H + (int32_t)scroll_y_smooth * vis * ROW_H / (ms + vis * ROW_H);
                SSD1306_DrawVLine(GUI_WIDTH - 1, by, bh, WHITE);
            }
        }

        SSD1306_Flush();
        last_render = now;
        gui_dirty = 0;
    }
}
