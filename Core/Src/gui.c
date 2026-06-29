/* GUI — Multi-level menu framework (portrait 32x128 OLED)
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
#define SIDEBAR_W     10       /* left vertical title strip */
#define CONTENT_X     (SIDEBAR_W + 2)   /* content starts here */
#define CONTENT_W     (GUI_WIDTH - CONTENT_X)  /* ~116 px */
#define HEADER_H      10       /* top bar height */
#define ROW_H         10       /* each row: 7px font + 3px gap */
#define VISIBLE_ROWS  2        /* (32 - 10) / 10 = 2 rows + partial 3rd */
#define STACK_DEPTH   6

/* ---- State ---- */
static const MenuPage *nav_stack[STACK_DEPTH];
static uint8_t  nav_depth;
static uint8_t  nav_cursor;
static int16_t  scroll_y, target_scroll;
static uint8_t  editing, edit_val_idx;
static int16_t  slide_x;
static int8_t   slide_dir;
static uint32_t slide_start;
#define SLIDE_DURATION 200

static float    cached_temp, cached_vdda;
static uint32_t last_adc_read;

/* ================================================================
 *  Smart float formatting (no %f — nano.specs disables float printf)
 * ================================================================ */
static void format_fixed(char *buf, size_t sz, float val, uint8_t dec)
{
    if (dec > 6) dec = 6;
    float mul = 1.0f;
    for (uint8_t d = 0; d < dec; d++) mul *= 10.0f;
    int32_t sv = (int32_t)(val * mul + (val >= 0 ? 0.5f : -0.5f));
    int32_t ip = sv / (int32_t)mul;
    uint32_t fp = (uint32_t)(sv >= 0 ? sv : -sv) % (uint32_t)mul;

    char tmp[16]; int pos = 0;
    int32_t n = ip;
    if (n == 0) { tmp[pos++] = '0'; }
    else {
        if (n < 0) { n = -n; }
        int32_t m = n, digits = 0;
        while (m > 0) { digits++; m /= 10; }
        pos = digits;
        for (int i = digits-1; i >= 0; i--) { tmp[i] = (char)('0'+(n%10)); n /= 10; }
    }
    if (sz < (size_t)(pos + 1 + dec + 1)) return;
    size_t w = 0;
    if (ip < 0) ((char*)buf)[w++] = '-';
    for (int i = 0; i < pos; i++) ((char*)buf)[w++] = tmp[i];
    if (dec > 0) {
        ((char*)buf)[w++] = '.';
        for (int d = dec-1; d >= 0; d--) {
            uint32_t p10 = 1;
            for (int x = 0; x < d; x++) p10 *= 10;
            ((char*)buf)[w++] = (char)('0' + ((fp / p10) % 10));
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

/* ---- Draw vertical sidebar title (one char per row, top→down) ---- */
static void draw_sidebar(const char *title)
{
    /* Fill sidebar background */
    SSD1306_FillRect(0, 0, SIDEBAR_W, GUI_HEIGHT, WHITE);
    /* Draw each character vertically, black-on-white */
    int len = (int)strlen(title);
    int max_chars = GUI_HEIGHT / 7;  /* 7px font = ~4 chars max */
    if (len > max_chars) len = max_chars;
    int start_y = (GUI_HEIGHT - len * 7) / 2;
    if (start_y < 0) start_y = 0;
    for (int i = 0; i < len; i++) {
        Font_DrawChar(SIDEBAR_W / 2 - 2, start_y + i * 7, title[i], 0, 1, BLACK);
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
    [6] = {"",    0},  /* Stim State — text */
    [7] = {"",    0},  /* Demo Name — text */
    [8] = {"",    0},  /* Phase Mode — text */
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
    demo_mode = idx;
    Set_Stimulation(DemoStimulations[idx]);
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
        nav_cursor = 0; target_scroll = 0; scroll_y = 0;
        editing = 0;
        slide_x = GUI_WIDTH; slide_dir = -1;
        slide_start = HAL_GetTick();
    }
}
void GUI_PopPage(void) {
    if (nav_depth > 1) {
        nav_depth--;
        nav_cursor = 0; target_scroll = 0; scroll_y = 0;
        editing = 0;
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
extern int calibration_mode;

/* ---------- Root page (scrollable data + buttons) ---------- */
static const MenuItem items_root[] = {
    {"Refresh",  MENU_DATA_FLOAT, .df={0}},
    {"Sys Freq", MENU_DATA_FLOAT, .df={1}},
    {"Temp",     MENU_DATA_FLOAT, .df={2}},
    {"VDDA",     MENU_DATA_FLOAT, .df={3}},
    {"DMA Time", MENU_DATA_FLOAT, .df={4}},
    {"Cal Mode", MENU_DATA_FLOAT, .df={5}},
    {"Stim St",  MENU_DATA_FLOAT, .df={6}},   /* stimulation on/off + type */
    {"Demo",     MENU_DATA_FLOAT, .df={7}},   /* current demo name, or "None" */
    {"Phs Mode", MENU_DATA_FLOAT, .df={8}},   /* phase set mode */
    {"DEMO",     MENU_FOLDER, .submenu=&Page_Demo},
    {"CALIB",    MENU_FOLDER, .submenu=&Page_Calibration},
    {"ABOUT",    MENU_FOLDER, .submenu=&Page_About},
};
const MenuPage Page_Root = {"UMH V5.5", items_root, 12};

/* ---------- ABOUT page ---------- */
static void render_about(const void *ctx) {
    (void)ctx;
    draw_sidebar("A");
    Font_DrawStr(CONTENT_X, 0, "UMH V5.5", 0, 1, WHITE);
    SSD1306_DrawHLine(CONTENT_X, 8, CONTENT_W, WHITE);
    Font_DrawStr(CONTENT_X, 11, "Designed by", 0, 1, WHITE);
    Font_DrawStr(CONTENT_X, 19, "Kannmu @SEU", 0, 1, WHITE);
    Font_DrawStr(CONTENT_X, 27, "[<] back", 0, 1, WHITE);
}
static const MenuItem items_about[] = {
    {"ABOUT", MENU_ACTION, .action=NULL, .custom_render=render_about},
};
const MenuPage Page_About = {"ABOUT", items_about, 1};

/* ---------- DEMO page (6 rows, scrollable) ---------- */
static void render_demo(const void *ctx) {
    (void)ctx;
    draw_sidebar("D");
    Font_DrawStr(CONTENT_X, 0, "SELECT DEMO", 0, 1, WHITE);
    SSD1306_DrawHLine(CONTENT_X, HEADER_H - 2, CONTENT_W, WHITE);

    static const char *labels[] = {"DLM_2","DLM_3","ULM_L","LM_L","LM_C"};
    for (int i = 0; i < 5; i++) {
        int16_t y = HEADER_H + (int16_t)i * ROW_H - scroll_y;
        if (y < HEADER_H - ROW_H || y > GUI_HEIGHT) continue;
        uint8_t active = (demo_mode == i);
        uint8_t cur = (i == (int)nav_cursor);
        Colour bg = cur ? WHITE : BLACK;
        Colour fg = cur ? BLACK : WHITE;
        SSD1306_FillRect(CONTENT_X, y, CONTENT_W, ROW_H - 1, bg);
        char buf[22];
        snprintf(buf, sizeof(buf), "%s%s", active ? ">" : " ", labels[i]);
        Font_DrawStr(CONTENT_X + 1, y, buf, 0, 1, fg);
    }
    /* BACK row */
    int16_t yy = HEADER_H + 5 * ROW_H - scroll_y;
    {
        uint8_t cur = (nav_cursor == 5);
        Colour bg = cur ? WHITE : BLACK;
        Colour fg = cur ? BLACK : WHITE;
        SSD1306_FillRect(CONTENT_X, yy, CONTENT_W, ROW_H - 1, bg);
        Font_DrawStr(CONTENT_X + 1, yy, "< BACK", 0, 1, fg);
    }
}
static const MenuItem items_demo[] = {
    {"DEMO", MENU_ACTION, .action=NULL, .custom_render=render_demo},
};
const MenuPage Page_Demo = {"DEMO", items_demo, 1};

/* ---------- CALIB page (3 rows, scrollable) ---------- */
static void render_calib(const void *ctx) {
    (void)ctx;
    draw_sidebar("C");
    Font_DrawStr(CONTENT_X, 0, "CALIBRATION", 0, 1, WHITE);
    SSD1306_DrawHLine(CONTENT_X, HEADER_H - 2, CONTENT_W, WHITE);

    static const char *labels[] = {"LOAD from EEPROM", "BYPASS", "SEMI-AUTO"};
    for (int i = 0; i < 3; i++) {
        int16_t y = HEADER_H + (int16_t)i * ROW_H - scroll_y;
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
const MenuPage Page_Calibration = {"CALIB", items_cal, 1};

/* ---------- Semi-auto calibration page ---------- */
static void render_semi_calib(const void *ctx) {
    (void)ctx; char buf[22];
    draw_sidebar("S");
    switch (calib_state) {
    case CALIB_PROMPT:
        Font_DrawStr(CONTENT_X, 0, "SEMI-AUTO CAL", 0, 1, WHITE);
        snprintf(buf,sizeof(buf),"Elem %d/60", calib_current_element+1);
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
        Font_DrawStr(CONTENT_X, 0, "CAL COMPLETE", 0, 1, WHITE);
        Font_DrawStr(CONTENT_X, 10, "Saved to EEPROM", 0, 1, WHITE);
        Font_DrawStr(CONTENT_X, 22, "[<] exit", 0, 1, WHITE);
        break;
    case CALIB_ERROR:
        Font_DrawStr(CONTENT_X, 0, "ERROR", 0, 1, WHITE);
        Font_DrawStr(CONTENT_X, 12, "ADC timeout", 0, 1, WHITE);
        break;
    default: break;
    }
}
static const MenuItem items_semi_calib[] = {
    {"CAL", MENU_ACTION, .action=NULL, .custom_render=render_semi_calib},
};
const MenuPage Page_SemiCalib = {"SEMI CAL", items_semi_calib, 1};

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
    scroll_y = 0; target_scroll = 0;
    editing = 0;
    slide_x = 0; slide_dir = 0;
    cached_temp = 0; cached_vdda = 0; last_adc_read = 0;
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
    static uint32_t gui_last_render;
    if (now - gui_last_render < 30) return;

    /* ADC cache (~10Hz) */
    if (now - last_adc_read >= 100) {
        cached_temp = Get_Temperature();
        cached_vdda = Get_Voltage_VDDA();
        last_adc_read = now;
    }

    /* Tick calibration state machine */
    SemiCalib_Tick();

    /* Determine page type */
    int is_custom_page = (page == &Page_Demo) || (page == &Page_Calibration)
                      || (page == &Page_About) || (page == &Page_SemiCalib);
    int custom_rows = 0;
    if (page == &Page_Demo)       custom_rows = 6;  /* 5 demo choices + back */
    else if (page == &Page_Calibration) custom_rows = 3;
    else if (page == &Page_About) custom_rows = 0;  /* static, only back */
    else if (page == &Page_SemiCalib) custom_rows = 0;  /* handled specially */

    /* ---- SemiCalib active: intercept buttons ---- */
    if (page == &Page_SemiCalib && calib_state != CALIB_IDLE
        && calib_state != CALIB_DONE && calib_state != CALIB_ERROR)
    {
        NavAction nv = Buttons_GetNav();
        if (nv == NAV_CONFIRM)      calib_button_pressed = 1;
        else if (nv == NAV_RETURN)  calib_button_pressed = 2;
        goto render_only;
    }

    /* ---- About / SemiCalib-IDLE pages: RETURN to go back ---- */
    if ((page == &Page_About) ||
        (page == &Page_SemiCalib && calib_state == CALIB_IDLE) ||
        (page == &Page_SemiCalib && calib_state == CALIB_DONE))
    {
        NavAction nv = Buttons_GetNav();
        if (nv == NAV_RETURN) { GUI_PopPage(); goto render_only; }
        if (page == &Page_SemiCalib && calib_state == CALIB_DONE && nv == NAV_CONFIRM)
            { GUI_PopPage(); goto render_only; }
    }

    {
    NavAction nav = Buttons_GetNav();

    /* ---- Navigation ---- */
    if (is_custom_page && !editing) {
        if (page == &Page_Demo || page == &Page_Calibration) {
            int maxc = custom_rows - 1;
            if (nav == NAV_UP   && nav_cursor > 0)     nav_cursor--;
            if (nav == NAV_DOWN && nav_cursor < (uint8_t)maxc) nav_cursor++;

            if (nav == NAV_CONFIRM) {
                if (page == &Page_Demo) {
                    if (nav_cursor < 5) GUI_Action_SetDemo((int)nav_cursor);
                    else                GUI_PopPage();
                } else if (page == &Page_Calibration) {
                    if (nav_cursor == 0) {
                        extern float Transducer_Calibration_Array[];
                        float ecal[60];
                        if (EEPROM_LoadCalibration(ecal))
                            for (int i=0;i<60;i++) Transducer_Calibration_Array[i]=ecal[i];
                        Load_Calib_to_Transducers();
                        Update_Full_Waveform_Buffer();
                    } else if (nav_cursor == 1) {
                        GUI_Action_ToggleCalibBypass();
                    } else {
                        GUI_Action_StartSemiAutoCalib();
                    }
                }
            }
            if (nav == NAV_RETURN) GUI_PopPage();
        }
    } else if (!is_custom_page) {
        /* Standard menu navigation */
        int max_item = (int)page->item_count - 1;
        if (page->items[page->item_count-1].type == MENU_BACK)
            max_item = (int)page->item_count - 2;

        if (editing) {
            const MenuItem *it = &page->items[edit_val_idx];
            if (it->type == MENU_VALUE_INT || it->type == MENU_VALUE_ENUM) {
                if (nav == NAV_UP)   { int v=*it->iv.value+it->iv.step; if(v<=it->iv.max)*it->iv.value=v; }
                if (nav == NAV_DOWN) { int v=*it->iv.value-it->iv.step; if(v>=it->iv.min)*it->iv.value=v; }
                if (nav == NAV_CONFIRM || nav == NAV_RETURN) editing = 0;
            } else { editing = 0; }
        } else {
            if (nav == NAV_UP   && nav_cursor > 0)            nav_cursor--;
            if (nav == NAV_DOWN && nav_cursor < (uint8_t)max_item) nav_cursor++;
            if (nav == NAV_CONFIRM) {
                const MenuItem *it = &page->items[nav_cursor];
                switch (it->type) {
                case MENU_FOLDER: if (it->submenu) GUI_PushPage(it->submenu); break;
                case MENU_ACTION: if (it->action)  it->action(); break;
                case MENU_VALUE_INT: case MENU_VALUE_ENUM:
                    editing=1; edit_val_idx=nav_cursor; break;
                case MENU_BACK: GUI_PopPage(); break;
                default: break;
                }
            }
            if (nav == NAV_RETURN && page != &Page_Root) GUI_PopPage();
        }
    }
    }

    /* Scroll target update (only for scrollable custom pages & standard pages) */
    {
        int row_count;
        if (page == &Page_Root) row_count = (int)page->item_count;
        else if (page == &Page_Demo) row_count = 6;
        else if (page == &Page_Calibration) row_count = 3;
        else row_count = 0;

        target_scroll = (int16_t)nav_cursor * ROW_H - (int16_t)((VISIBLE_ROWS/2)*ROW_H);
        if (target_scroll < 0) target_scroll = 0;
        int16_t ms = (int16_t)(row_count - VISIBLE_ROWS) * ROW_H + ROW_H;
        if (ms < 0) ms = 0;
        if (target_scroll > ms) target_scroll = ms;
        scroll_y += (target_scroll - scroll_y) / 3;
    }

    /* Slide animation */
    if (slide_dir != 0) {
        uint32_t dt = now - slide_start;
        if (dt >= SLIDE_DURATION) { slide_x = 0; slide_dir = 0; }
        else {
            float t = (float)dt / (float)SLIDE_DURATION;
            t = t * t * (3.0f - 2.0f * t);
            if (slide_dir < 0) slide_x =  GUI_WIDTH - (int16_t)((float)GUI_WIDTH * t);
            else               slide_x = -GUI_WIDTH + (int16_t)((float)GUI_WIDTH * t);
        }
    }

render_only:
    /* ---- RENDER ---- */
    SSD1306_Fill(BLACK);
    const MenuItem *it = page->items;
    int count = (int)page->item_count;

    if (page->items[0].custom_render) {
        page->items[0].custom_render((void*)page);
    } else {
        /* Standard sidebar+list layout */
        draw_sidebar(page->title);
        int max_item = count - 1;
        if (page->items[count-1].type == MENU_BACK) max_item = count - 2;

        for (int i = 0; i < count && i <= max_item; i++) {
            int16_t y = HEADER_H + (int16_t)i * ROW_H - scroll_y;
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
                    /* Stim state: enabled/disabled + type name */
                    if (Get_Stimulation_Enabled())
                        snprintf(buf, sizeof(buf), "%s", CurrentStimulation.name);
                    else
                        snprintf(buf, sizeof(buf), "OFF");
                } else if (slot == 7) {
                    /* Demo name */
                    if (demo_mode >= 0 && demo_mode < Get_Num_Demo_Stimulations())
                        snprintf(buf, sizeof(buf), "%s", DemoStimulations[demo_mode]->name);
                    else
                        snprintf(buf, sizeof(buf), "None");
                } else if (slot == 8) {
                    /* Phase set mode */
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
        int row_count = count;
        int vis = VISIBLE_ROWS;
        int16_t ms = (int16_t)(row_count - vis) * ROW_H + ROW_H;
        if (ms > 0) {
            int16_t bh = vis * ROW_H * vis / row_count;
            if (bh < 4) bh = 4;
            int16_t by = HEADER_H + (int32_t)scroll_y * vis * ROW_H / (ms + vis * ROW_H);
            SSD1306_DrawVLine(GUI_WIDTH - 1, by, bh, WHITE);
        }
    }

    SSD1306_Flush();
    gui_last_render = now;
}
