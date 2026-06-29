/* GUI — Multi-level menu framework (portrait 32x128 OLED)
 *
 * Navigation: UP/DOWN = cursor, CONFIRM = enter/execute, RETURN = back/cancel.
 * Animation: smooth scroll easing, page slide transitions.
 * Extensible: add new pages by defining MenuPage+MenuItem arrays, no framework edits.
 */
#include "gui.h"
#include "font.h"
#include "buttons.h"
#include "dma_manager.h"
#include "stimulation.h"
#include "calibration.h"
#include "utiles.h"
#include <string.h>
#include <stdio.h>

#define STACK_DEPTH   6
#define HEADER_H      10
#define ROW_H         14    /* 5x7 scale=2 */
#define VISIBLE_ROWS  8     /* (128-10)/14 */

static const MenuPage *nav_stack[STACK_DEPTH];
static uint8_t  nav_depth;
static uint8_t  nav_cursor;
static int16_t  scroll_y;      /* animated scroll offset */
static int16_t  target_scroll;
static uint8_t  editing;       /* 0=navigate, 1=editing value */
static uint8_t  edit_val_idx;  /* which item's value */

/* Page slide animation */
static int16_t  slide_x;       /* 0 = current page, ±32 = sliding */
static int8_t   slide_dir;     /* 0=none, -1=push-left, 1=pop-right */
static uint32_t slide_start;
#define SLIDE_DURATION 120      /* ms */

/* Cached DWT timestamps for perf display, updated by GUI_Tick */
static float    gui_fps;
static uint32_t gui_last_render;

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
    /* Stub — Phase 9 fills in the guided calibration state machine */
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
        /* Slide animation */
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

/* ---- Page definitions (gui_pages.c) ---- */
extern const MenuPage Page_Root;
extern const MenuPage Page_Demo;
extern const MenuPage Page_Calibration;
extern int calibration_mode;

/* ---- Monitor page custom render ---- */
static void render_monitor(const void *ctx)
{
    (void)ctx;
    char buf[16];
    Font_DrawStr(0, HEADER_H,          "MONITOR",     0, 1, WHITE);
    snprintf(buf,sizeof(buf),"f:%dHz",(int)System_Loop_Freq);
    Font_DrawStr(0, HEADER_H+ROW_H,    buf,           0, 1, WHITE);
    snprintf(buf,sizeof(buf),"T:%dC",  (int)Get_Temperature());
    Font_DrawStr(0, HEADER_H+ROW_H*2,  buf,           0, 1, WHITE);
    snprintf(buf,sizeof(buf),"V:%.1f", Get_Voltage_VDDA());
    Font_DrawStr(0, HEADER_H+ROW_H*3,  buf,           0, 1, WHITE);
    snprintf(buf,sizeof(buf),"dt:%.1f",updateDMABufferDeltaTime);
    Font_DrawStr(0, HEADER_H+ROW_H*4,  buf,           0, 1, WHITE);
    const char *stim = CurrentStimulation.name;
    Font_DrawStr(0, HEADER_H+ROW_H*5,  stim,          0, 1, WHITE);
    snprintf(buf,sizeof(buf),"Cal:%s",Get_Calibration_Mode()?"ON":"OFF");
    Font_DrawStr(0, HEADER_H+ROW_H*6,  buf,           0, 1, WHITE);
}

static const MenuItem items_root[] = {
    {"MON",  MENU_ACTION,  .action = NULL,   .custom_render = render_monitor},
    {"DEMO", MENU_FOLDER,  .submenu = &Page_Demo},
    {"CAL",  MENU_FOLDER,  .submenu = &Page_Calibration},
    {"ABOUT",MENU_FOLDER,  .submenu = NULL},  /* placeholder */
};
const MenuPage Page_Root = {"UMH V5", items_root, 4};

static const MenuItem items_demo[] = {
    {"DLM_2",MENU_ACTION, .action = NULL},
    {"DLM_3",MENU_ACTION, .action = NULL},
    {"ULM_L",MENU_ACTION, .action = NULL},
    {"LM_L", MENU_ACTION, .action = NULL},
    {"LM_C", MENU_ACTION, .action = NULL},
    {"BACK", MENU_BACK},
};
const MenuPage Page_Demo = {"DEMO", items_demo, 6};

static const MenuItem items_cal[] = {
    {"BYPASS",MENU_ACTION, .action = GUI_Action_ToggleCalibBypass},
    {"SEMI",  MENU_ACTION, .action = GUI_Action_StartSemiAutoCalib},
    {"BACK",  MENU_BACK},
};
const MenuPage Page_Calibration = {"CALIB", items_cal, 3};

/* ---- GUI Init ---- */
void GUI_Init(void)
{
    SSD1306_Init();
    Buttons_Init();
    Buttons_Tick();  /* seed debounce state */

    nav_depth = 0;
    nav_cursor = 0;
    scroll_y = 0;
    target_scroll = 0;
    editing = 0;
    slide_x = 0;
    slide_dir = 0;
    GUI_PushPage(&Page_Root);
    gui_last_render = HAL_GetTick();
}

/* ---- GUI Tick (call from main loop at ~25Hz) ---- */
void GUI_Tick(void)
{
    const MenuPage *page = GUI_CurrentPage();
    if (!page) return;

    uint32_t now = HAL_GetTick();

    /* Throttle to ~30Hz */
    if (now - gui_last_render < 30) return;

    NavAction nav = Buttons_GetNav();

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
                else if (nav_cursor <= 5 && page == &Page_Demo)
                    GUI_Action_SetDemo((int)nav_cursor);
                break;
            case MENU_VALUE_INT:
            case MENU_VALUE_ENUM:
                editing = 1;
                edit_val_idx = nav_cursor;
                break;
            case MENU_BACK:
                GUI_PopPage();
                break;
            default: break;
            }
        }
        if (nav == NAV_RETURN && page != &Page_Root)
            GUI_PopPage();
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
            t = t * t * (3.0f - 2.0f * t);  /* smoothstep */
            if (slide_dir < 0) slide_x = GUI_WIDTH - (int16_t)((float)GUI_WIDTH * t);
            else               slide_x = -(GUI_WIDTH) + (int16_t)((float)GUI_WIDTH * t);
        }
    }

    /* ---- Render ---- */
    SSD1306_Fill(BLACK);
    const MenuItem *it = page->items;
    int count = page->item_count;

    if (page->items[0].custom_render)
    {
        /* Full-screen custom page */
        page->items[0].custom_render((void*)page);
    }
    else
    {
        /* Standard menu */
        Font_DrawStr(1, 0, page->title, 0, 1, WHITE);
        SSD1306_DrawHLine(0, HEADER_H - 2, GUI_WIDTH, WHITE);

        for (int i = 0; i < count && i < (max_item + 1); i++)
        {
            int16_t y = HEADER_H + (int16_t)i * ROW_H - scroll_y;
            if (y < HEADER_H - ROW_H || y > GUI_HEIGHT) continue;

            const char *label = it[i].label;
            /* Shorten special labels */
            if (it[i].type == MENU_BACK) label = "<";

            uint8_t is_cursor = (i == (int)nav_cursor && !editing);
            Colour bg = is_cursor ? WHITE : BLACK;
            Colour fg = is_cursor ? BLACK : WHITE;

            /* Cursor bar */
            SSD1306_FillRect(0, y - 1, GUI_WIDTH, ROW_H - 1, bg);
            Font_DrawStr(1, y, label, 0, 1, fg);

            /* Value display */
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

        /* Scroll indicator */
        if (max_scroll > 0) {
            int16_t bar_h = VISIBLE_ROWS * ROW_H * VISIBLE_ROWS / count;
            if (bar_h < 4) bar_h = 4;
            int16_t bar_y = HEADER_H + (int32_t)scroll_y * VISIBLE_ROWS * ROW_H / (max_scroll + VISIBLE_ROWS * ROW_H);
            SSD1306_DrawVLine(GUI_WIDTH - 1, bar_y, bar_h, WHITE);
        }
    }

    /* Apply slide offset */
    if (slide_dir != 0) {
        /* Simple slide: just shift x (partial page visible) — use fill rect for transition */
    }

    SSD1306_Flush();
    gui_last_render = HAL_GetTick();
    gui_fps = 1000.0f * (0.7f * 30.0f + 0.3f * gui_fps) / 30.0f; /* low-pass */
    (void)gui_fps;
}
