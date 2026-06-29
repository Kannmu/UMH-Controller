/* GUI — Multi-level menu framework (portrait 32x128 OLED)
 *
 * Navigation: UP/DOWN = cursor, CONFIRM = enter/execute, RETURN = back/cancel.
 * Animation: smooth scroll easing, page slide transitions.
 * Extensible: add new pages by defining MenuPage+MenuItem arrays, no framework edits.
 *
 * Root page: 1-column N-row scrollable list.
 *   Data slots (MENU_DATA_FLOAT): left label, right live value with smart SI units.
 *   Button slots (MENU_FOLDER): centered text, enter submenu on CONFIRM.
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

#define STACK_DEPTH   6
#define HEADER_H      8     /* 7px font + 1px breathing room */
#define ROW_H         9     /* 7px font + 2px gap */
#define VISIBLE_ROWS  2     /* (32 - 8) / 9 = 2 full rows + partial 3rd */

static const MenuPage *nav_stack[STACK_DEPTH];
static uint8_t  nav_depth;
static uint8_t  nav_cursor;
static int16_t  scroll_y;      /* animated scroll offset */
static int16_t  target_scroll;
static uint8_t  editing;       /* 0=navigate, 1=editing value */
static uint8_t  edit_val_idx;  /* which item's value */

/* Page slide animation */
static int16_t  slide_x;       /* 0 = current page, ±128 = sliding */
static int8_t   slide_dir;     /* 0=none, -1=push-left, 1=pop-right */
static uint32_t slide_start;
#define SLIDE_DURATION 200      /* ms */

/* ADC read caching (avoid I2C contention; ADC polls are ~200us each) */
static float    cached_temp;
static float    cached_vdda;
static uint32_t last_adc_read;

/* ---- Smart unit formatting (no %f dependency — nano.specs disables float printf) ---- */
static void format_fixed(char *buf, size_t buf_size, float val, uint8_t decimals)
{
    /* Manual fixed-point: integer part + '.' + fractional part */
    if (decimals > 6) decimals = 6;
    float multiplier = 1.0f;
    for (uint8_t d = 0; d < decimals; d++) multiplier *= 10.0f;

    int32_t scaled_val = (int32_t)(val * multiplier + (val >= 0 ? 0.5f : -0.5f));
    int32_t int_part   = scaled_val / (int32_t)multiplier;
    uint32_t frac_part = (uint32_t)(scaled_val >= 0 ? scaled_val : -scaled_val) % (uint32_t)multiplier;

    /* integer portion */
    char tmp[16];
    int pos = 0;
    int32_t n = int_part;
    if (n == 0) { tmp[pos++] = '0'; }
    else {
        if (n < 0) { n = -n; }
        int32_t m = n;
        int digits = 0;
        while (m > 0) { digits++; m /= 10; }
        pos = digits;
        for (int i = digits - 1; i >= 0; i--) {
            tmp[i] = (char)('0' + (n % 10));
            n /= 10;
        }
    }

    if (buf_size < (size_t)(pos + 1 + decimals + 1)) return;  /* overflow safety */
    size_t written = 0;
    if (int_part < 0) { buf[written++] = '-'; }
    for (int i = 0; i < pos; i++) buf[written++] = tmp[i];
    if (decimals > 0) {
        buf[written++] = '.';
        for (int d = decimals - 1; d >= 0; d--) {
            uint32_t p10 = 1;
            for (int x = 0; x < d; x++) p10 *= 10;
            buf[written++] = (char)('0' + ((frac_part / p10) % 10));
        }
    }
    buf[written] = '\0';
}

void GUI_FormatSmartUnits(char *buf, size_t buf_size, float value, const char *suffix, uint8_t decimals)
{
    float abs_val = fabsf(value);
    const char *prefix = "";
    float scaled = value;

    if (abs_val >= 1000.0f)           { prefix = "k"; scaled = value / 1000.0f; }
    else if (abs_val >= 1.0f)         { /* no prefix */ }
    else if (abs_val >= 0.001f)       { prefix = "m"; scaled = value * 1000.0f; }
    else if (abs_val >= 0.000001f)    { prefix = "u"; scaled = value * 1000000.0f; }
    else if (abs_val >= 0.000000001f) { prefix = "n"; scaled = value * 1000000000.0f; }
    else if (abs_val < 0.000000001f && abs_val > 0.0f) { scaled = 0.0f; }

    char num[20];
    format_fixed(num, sizeof(num), scaled, decimals);
    snprintf(buf, buf_size, "%s %s%s", num, prefix, suffix);
}

/* ---- Data slot descriptor table ---- */
typedef struct {
    const char *suffix;
    uint8_t     decimals;
} DataSlotDesc;

static const DataSlotDesc data_slots[] = {
    [0] = {"Hz",  3},  /* Refresh Rate */
    [1] = {"Hz",  0},  /* Sys Loop Freq */
    [2] = {"C",   1},  /* Temperature */
    [3] = {"V",   3},  /* VDDA */
    [4] = {"ms",  3},  /* DMA Buffer Time */
    [5] = {"",    0},  /* Calibration Mode (text) */
};

/* ---- Action callbacks (callable from menu items) ---- */
void GUI_Action_ToggleCalibBypass(void)
{
    int mode = Get_Calibration_Mode();
    calibration_mode = 1 - mode;
    if (calibration_mode)
        Enter_Calibration_Mode();
    else
        Load_Calib_to_Transducers();
    Update_Full_Waveform_Buffer();
}

void GUI_Action_SetDemo(int idx)
{
    if (idx < 0 || idx >= Get_Num_Demo_Stimulations()) return;
    demo_mode = idx;
    Set_Stimulation(DemoStimulations[idx]);
    phase_set_mode = 0;
}

void GUI_Action_StartSemiAutoCalib(void)
{
    SemiCalib_Init();
    calib_state = CALIB_PROMPT;
    calib_current_element = 0;
    GUI_PushPage(&Page_SemiCalib);
}

/* ---- Navigation stack ---- */
void GUI_PushPage(const MenuPage *page)
{
    if (nav_depth < STACK_DEPTH) {
        nav_stack[nav_depth++] = page;
        nav_cursor = 0;
        target_scroll = 0;
        scroll_y = 0;
        editing = 0;
        slide_x = GUI_WIDTH;
        slide_dir = -1;
        slide_start = HAL_GetTick();
    }
}

void GUI_PopPage(void)
{
    if (nav_depth > 1) {
        nav_depth--;
        nav_cursor = 0;
        target_scroll = 0;
        scroll_y = 0;
        editing = 0;
        slide_x = -(GUI_WIDTH);
        slide_dir = 1;
        slide_start = HAL_GetTick();
    }
}

const MenuPage* GUI_CurrentPage(void)
{
    return (nav_depth > 0) ? nav_stack[nav_depth-1] : NULL;
}

/* ---- Page definitions ---- */
extern const MenuPage Page_Root;
extern const MenuPage Page_Demo;
extern const MenuPage Page_Calibration;
extern const MenuPage Page_About;
extern int calibration_mode;

/* ---- Root page (1-column scrollable data + buttons) ---- */
static void render_about(const void *ctx)
{
    (void)ctx;
    Font_DrawStr(0, 0, "ABOUT", 0, 1, WHITE);
    SSD1306_DrawHLine(0, HEADER_H - 1, GUI_WIDTH, WHITE);
    Font_DrawStr(0, HEADER_H, "UMH V5.5", 0, 1, WHITE);
    Font_DrawStr(0, HEADER_H + 9, "Designed by", 0, 1, WHITE);
    Font_DrawStr(0, HEADER_H + 17, "Kannmu @ SEU", 0, 1, WHITE);
}

static const MenuItem items_root[] = {
    /* Data slots: label, MENU_DATA_FLOAT, slot_idx in .df */
    {"Refresh",  MENU_DATA_FLOAT, .df = {0}},
    {"Sys Freq", MENU_DATA_FLOAT, .df = {1}},
    {"Temp",     MENU_DATA_FLOAT, .df = {2}},
    {"VDDA",     MENU_DATA_FLOAT, .df = {3}},
    {"DMA Time", MENU_DATA_FLOAT, .df = {4}},
    {"Cal Mode", MENU_DATA_FLOAT, .df = {5}},
    /* Button slots */
    {"DEMO",     MENU_FOLDER, .submenu = &Page_Demo},
    {"CALIB",    MENU_FOLDER, .submenu = &Page_Calibration},
    {"ABOUT",    MENU_FOLDER, .submenu = &Page_About},
};
const MenuPage Page_Root = {"UMH V5.5", items_root, 9};

/* ---- ABOUT page ---- */
static const MenuItem items_about[] = {
    {"ABOUT", MENU_ACTION, .action = NULL, .custom_render = render_about},
};
const MenuPage Page_About = {"ABOUT", items_about, 1};

/* ---- Demo page: custom render showing active demo with ">" indicator ---- */
static void render_demo(const void *ctx)
{
    (void)ctx;
    Font_DrawStr(1, 0, "DEMO", 0, 1, WHITE);
    SSD1306_DrawHLine(0, HEADER_H - 1, GUI_WIDTH, WHITE);

    static const char *labels[] = {"DLM_2","DLM_3","ULM_L","LM_L","LM_C"};
    for (int i = 0; i < 5; i++) {
        int16_t y = HEADER_H + (int16_t)i * ROW_H;
        if (y < HEADER_H - ROW_H || y > GUI_HEIGHT) continue;
        uint8_t is_active = (demo_mode == i);
        uint8_t is_cursor = (i == (int)nav_cursor);
        Colour bg = is_cursor ? WHITE : BLACK;
        Colour fg = is_cursor ? BLACK : WHITE;
        SSD1306_FillRect(0, y, GUI_WIDTH, ROW_H - 1, bg);
        char buf[24];
        int off = snprintf(buf, sizeof(buf), "%s%s", is_active ? ">" : " ", labels[i]);
        (void)off;
        Font_DrawStr(1, y, buf, 0, 1, fg);
    }

    /* BACK row */
    int16_t y = HEADER_H + 5 * ROW_H;
    uint8_t is_cursor = (nav_cursor == 5);
    Colour bg = is_cursor ? WHITE : BLACK;
    Colour fg = is_cursor ? BLACK : WHITE;
    SSD1306_FillRect(0, y, GUI_WIDTH, ROW_H - 1, bg);
    Font_DrawStr(1, y, "<", 0, 1, fg);
}

static const MenuItem items_demo[] = {
    {"DEMO", MENU_ACTION, .action = NULL, .custom_render = render_demo},
};
const MenuPage Page_Demo = {"DEMO", items_demo, 1};

/* ---- Calibration page: custom render with Load/Save/Bypass/Semi buttons ---- */
static void render_calib(const void *ctx)
{
    (void)ctx;
    Font_DrawStr(1, 0, "CALIB", 0, 1, WHITE);
    SSD1306_DrawHLine(0, HEADER_H - 1, GUI_WIDTH, WHITE);

    static const char *labels[] = {"LOAD", "BYPASS", "SEMI"};
    for (int i = 0; i < 3; i++) {
        int16_t y = HEADER_H + (int16_t)i * ROW_H;
        if (y < HEADER_H - ROW_H || y > GUI_HEIGHT) continue;
        uint8_t is_cursor = (i == (int)nav_cursor);
        Colour bg = is_cursor ? WHITE : BLACK;
        Colour fg = is_cursor ? BLACK : WHITE;
        SSD1306_FillRect(0, y, GUI_WIDTH, ROW_H - 1, bg);

        char buf[32];
        if (i == 1) {
            /* BYPASS: show current toggle state */
            snprintf(buf, sizeof(buf), "%s  [%s]", labels[i], calibration_mode ? "ON" : "OFF");
        } else {
            snprintf(buf, sizeof(buf), "%s", labels[i]);
        }
        Font_DrawStr(1, y, buf, 0, 1, fg);
    }
}

static const MenuItem items_cal[] = {
    {"CAL", MENU_ACTION, .action = NULL, .custom_render = render_calib},
};
const MenuPage Page_Calibration = {"CALIB", items_cal, 1};

/* ---- Semi-auto calibration page custom render ---- */
static void render_semi_calib(const void *ctx)
{
    (void)ctx;
    char buf[22];

    switch (calib_state) {
    case CALIB_PROMPT:
        Font_DrawStr(0, 0, "SEMI-AUTO CAL", 0, 1, WHITE);
        snprintf(buf, sizeof(buf), "Elem %d/60", calib_current_element + 1);
        Font_DrawStr(0, 8, buf, 0, 1, WHITE);
        Font_DrawStr(0, 16, "Place probe", 0, 1, WHITE);
        Font_DrawStr(0, 24, "[OK]start [<]skip", 0, 1, WHITE);
        break;

    case CALIB_MEASURING: {
        Font_DrawStr(0, 0, "Measuring...", 0, 1, WHITE);
        char spin[] = {'|','/','-','\\'};
        char sp[2] = {spin[(HAL_GetTick()/200)%4], 0};
        Font_DrawStr(60, 12, sp, 0, 1, WHITE);
        break;
    }

    case CALIB_SHOW_RESULT:
        Font_DrawStr(0, 0, "RESULT", 0, 1, WHITE);
        snprintf(buf, sizeof(buf), "N%d: %.2fus", calib_current_element + 1,
                 (double)calib_results[calib_current_element].calib_us);
        Font_DrawStr(0, 8, buf, 0, 1, WHITE);
        {
            const char *qual = calib_results[calib_current_element].quality >= 2 ? "OK" :
                               calib_results[calib_current_element].quality >= 1 ? "LOW" : "BAD";
            snprintf(buf, sizeof(buf), "Amp:%s", qual);
        }
        Font_DrawStr(0, 16, buf, 0, 1, WHITE);
        Font_DrawStr(0, 24, "[OK]next [<]retry", 0, 1, WHITE);
        break;

    case CALIB_DONE:
        Font_DrawStr(0, 0, "CAL COMPLETE", 0, 1, WHITE);
        Font_DrawStr(0, 10, "Saved to", 0, 1, WHITE);
        Font_DrawStr(0, 20, "EEPROM", 0, 1, WHITE);
        break;

    case CALIB_ERROR:
        Font_DrawStr(0, 0, "ERROR", 0, 1, WHITE);
        Font_DrawStr(0, 12, "ADC timeout", 0, 1, WHITE);
        break;

    default:
        break;
    }
}

static const MenuItem items_semi_calib[] = {
    {"CAL", MENU_ACTION, .action = NULL, .custom_render = render_semi_calib},
};
const MenuPage Page_SemiCalib = {"SEMI CAL", items_semi_calib, 1};

/* ---- GUI Init ---- */
void GUI_Init(void)
{
    if (SSD1306_Init() != 0)
    {
        for (volatile int i = 0; i < 10; i++)
            HAL_GPIO_TogglePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin);
        return;
    }
    Buttons_Init();
    Buttons_Tick();

    nav_depth = 0;
    nav_cursor = 0;
    scroll_y = 0;
    target_scroll = 0;
    editing = 0;
    slide_x = 0;
    slide_dir = 0;
    cached_temp = 0.0f;
    cached_vdda = 0.0f;
    last_adc_read = 0;
    GUI_PushPage(&Page_Root);
}

/* ---- GUI Tick (call from main loop at ~30Hz) ---- */
void GUI_Tick(void)
{
    const MenuPage *page = GUI_CurrentPage();
    if (!page) return;

    uint32_t now = HAL_GetTick();

    /* Throttle to ~30Hz */
    static uint32_t gui_last_render;
    if (now - gui_last_render < 30) return;

    /* ADC read caching (~10Hz for temperature/VDDA) */
    if (now - last_adc_read >= 100) {
        cached_temp = Get_Temperature();
        cached_vdda = Get_Voltage_VDDA();
        last_adc_read = now;
    }

    /* ---- Semi-auto calibration: tick state machine ---- */
    SemiCalib_Tick();

    /* ---- Semi-auto calibration button intercept ---- */
    if (page == &Page_SemiCalib && calib_state != CALIB_IDLE && calib_state != CALIB_DONE && calib_state != CALIB_ERROR)
    {
        NavAction nav = Buttons_GetNav();
        if (nav == NAV_CONFIRM)      calib_button_pressed = 1;
        else if (nav == NAV_RETURN)  calib_button_pressed = 2;

        SSD1306_Fill(BLACK);
        if (page->items[0].custom_render)
            page->items[0].custom_render((void*)page);
        if (slide_dir != 0) {
            uint32_t dt = now - slide_start;
            if (dt >= SLIDE_DURATION) { slide_x = 0; slide_dir = 0; }
            else {
                float t = (float)dt / (float)SLIDE_DURATION;
                t = t * t * (3.0f - 2.0f * t);
                if (slide_dir < 0) slide_x = GUI_WIDTH - (int16_t)((float)GUI_WIDTH * t);
                else               slide_x = -(GUI_WIDTH) + (int16_t)((float)GUI_WIDTH * t);
            }
        }
        SSD1306_Flush();
        gui_last_render = now;
        return;
    }

    /* ---- Demo / Calib / SemiCalib custom-render navigation ---- */
    uint8_t is_custom_nav = (page == &Page_Demo) || (page == &Page_Calibration) || (page == &Page_SemiCalib);
    int custom_row_count = 0;
    if (page == &Page_Demo)            custom_row_count = 6;  /* 5 demos + BACK */
    else if (page == &Page_Calibration) custom_row_count = 3;  /* LOAD/BYPASS/SEMI */

    NavAction nav = Buttons_GetNav();

    if (is_custom_nav && !editing)
    {
        int max_c = custom_row_count - 1;
        if (nav == NAV_UP && nav_cursor > 0)
            nav_cursor--;
        if (nav == NAV_DOWN && nav_cursor < (uint8_t)max_c)
            nav_cursor++;

        if (nav == NAV_CONFIRM)
        {
            if (page == &Page_Demo)
            {
                if (nav_cursor < 5)
                    GUI_Action_SetDemo((int)nav_cursor);
                else
                    GUI_PopPage();
            }
            else if (page == &Page_Calibration)
            {
                if (nav_cursor == 0) {
                    extern float Transducer_Calibration_Array[];
                    float eeprom_cal[60];
                    if (EEPROM_LoadCalibration(eeprom_cal)) {
                        for (int i = 0; i < 60; i++)
                            Transducer_Calibration_Array[i] = eeprom_cal[i];
                    }
                    Load_Calib_to_Transducers();
                    Update_Full_Waveform_Buffer();
                } else if (nav_cursor == 1) {
                    GUI_Action_ToggleCalibBypass();
                } else {
                    GUI_Action_StartSemiAutoCalib();
                }
            }
        }
        if (nav == NAV_RETURN)
            GUI_PopPage();
    }
    else if (!is_custom_nav)
    {
    int max_item = (int)page->item_count - 1;
    if (page->items[page->item_count - 1].type == MENU_BACK)
        max_item = (int)page->item_count - 2;  /* skip BACK */

    /* ---- Editing mode ---- */
    if (editing)
    {
        const MenuItem *it = &page->items[edit_val_idx];
        if (it->type == MENU_VALUE_INT || it->type == MENU_VALUE_ENUM)
        {
            if (nav == NAV_UP)    { int v = *it->iv.value + it->iv.step; if (v <= it->iv.max) *it->iv.value = v; }
            if (nav == NAV_DOWN)  { int v = *it->iv.value - it->iv.step; if (v >= it->iv.min) *it->iv.value = v; }
            if (nav == NAV_CONFIRM || nav == NAV_RETURN) { editing = 0; }
        }
        else { editing = 0; }
    }
    else
    {
        /* Navigate */
        if (nav == NAV_UP && nav_cursor > 0)
            nav_cursor--;
        if (nav == NAV_DOWN && nav_cursor < (uint8_t)max_item)
            nav_cursor++;

        if (nav == NAV_CONFIRM)
        {
            const MenuItem *it = &page->items[nav_cursor];
            switch (it->type)
            {
            case MENU_FOLDER:
                if (it->submenu) GUI_PushPage(it->submenu);
                break;
            case MENU_ACTION:
                if (it->action) it->action();
                break;
            case MENU_VALUE_INT:
            case MENU_VALUE_ENUM:
                editing = 1;
                edit_val_idx = nav_cursor;
                break;
            case MENU_BACK:
                GUI_PopPage();
                break;
            case MENU_DATA_FLOAT:
                break;
            default: break;
            }
        }
        if (nav == NAV_RETURN && page != &Page_Root)
            GUI_PopPage();
    }
    }

    /* Update scroll target */
    target_scroll = (int16_t)nav_cursor * ROW_H - (int16_t)((VISIBLE_ROWS / 2) * ROW_H);
    if (target_scroll < 0) target_scroll = 0;
    int16_t max_scroll = ((int16_t)page->item_count - VISIBLE_ROWS) * ROW_H + ROW_H;
    if (max_scroll < 0) max_scroll = 0;
    if (target_scroll > max_scroll) target_scroll = max_scroll;

    /* Smooth scroll easing */
    scroll_y += (target_scroll - scroll_y) / 3;

    /* Slide animation */
    if (slide_dir != 0)
    {
        uint32_t dt = now - slide_start;
        if (dt >= SLIDE_DURATION) {
            slide_x = 0; slide_dir = 0;
        } else {
            float t = (float)dt / (float)SLIDE_DURATION;
            t = t * t * (3.0f - 2.0f * t);
            if (slide_dir < 0) slide_x = GUI_WIDTH - (int16_t)((float)GUI_WIDTH * t);
            else               slide_x = -(GUI_WIDTH) + (int16_t)((float)GUI_WIDTH * t);
        }
    }

    /* ---- Render ---- */
    SSD1306_Fill(BLACK);
    const MenuItem *it = page->items;
    int count = (int)page->item_count;

    if (page->items[0].custom_render)
    {
        /* Full-screen custom page */
        page->items[0].custom_render((void*)page);
    }
    else
    {
        /* Standard menu */
        int max_item = count - 1;
        if (page->items[page->item_count - 1].type == MENU_BACK)
            max_item = count - 2;

        int16_t title_w = Font_StrWidth(page->title, 0, 1);
        int16_t title_x = (GUI_WIDTH - title_w) / 2;
        if (title_x < 0) title_x = 1;
        Font_DrawStr(title_x, 0, page->title, 0, 1, WHITE);
        SSD1306_DrawHLine(0, HEADER_H - 1, GUI_WIDTH, WHITE);

        for (int i = 0; i < count && i <= max_item; i++)
        {
            int16_t y = HEADER_H + (int16_t)i * ROW_H - scroll_y;
            if (y < HEADER_H - ROW_H || y > GUI_HEIGHT) continue;

            const char *label = it[i].label;
            if (it[i].type == MENU_BACK) label = "<";

            uint8_t is_cursor = (i == (int)nav_cursor && !editing);
            Colour bg = is_cursor ? WHITE : BLACK;
            Colour fg = is_cursor ? BLACK : WHITE;

            /* Cursor bar */
            SSD1306_FillRect(0, y, GUI_WIDTH, ROW_H - 1, bg);

            /* ---- MENU_DATA_FLOAT: right-aligned value ---- */
            if (it[i].type == MENU_DATA_FLOAT)
            {
                char buf[22];
                uint8_t slot = it[i].df.slot_idx;

                /* Draw label on the left */
                Font_DrawStr(1, y, label, 0, 1, fg);

                if (slot == 5) {
                    /* Calibration Mode: text ON/OFF */
                    snprintf(buf, sizeof(buf), "%s", calibration_mode ? "ON" : "OFF");
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
                                         data_slots[slot].suffix,
                                         data_slots[slot].decimals);
                }
                Font_DrawStr(GUI_WIDTH - Font_StrWidth(buf, 0, 1) - 1, y, buf, 0, 1, fg);
            }
            else
            {
                /* Standard menu item: left text */
                Font_DrawStr(1, y, label, 0, 1, fg);

                /* Value display for int/enum editors */
                if (it[i].type == MENU_VALUE_INT)
                {
                    char buf[8];
                    snprintf(buf, sizeof(buf), it[i].iv.fmt, *it[i].iv.value);
                    Font_DrawStr(GUI_WIDTH - Font_StrWidth(buf, 0, 1) - 1, y, buf, 0, 1, fg);
                }
                else if (it[i].type == MENU_VALUE_ENUM && *it[i].ev.value < it[i].ev.count)
                {
                    const char *v = it[i].ev.names[*it[i].ev.value];
                    Font_DrawStr(GUI_WIDTH - Font_StrWidth(v, 0, 1) - 1, y, v, 0, 1, fg);
                }
            }
        }

        /* Scroll indicator */
        if (max_scroll > 0) {
            int16_t bar_h = VISIBLE_ROWS * ROW_H * VISIBLE_ROWS / count;
            if (bar_h < 4) bar_h = 4;
            int16_t bar_y = HEADER_H + (int32_t)scroll_y * VISIBLE_ROWS * ROW_H / (max_scroll + VISIBLE_ROWS * ROW_H);
            SSD1306_DrawVLine(GUI_WIDTH - 1, bar_y, bar_h, WHITE);
        }
    }

    SSD1306_Flush();
    gui_last_render = now;
}
