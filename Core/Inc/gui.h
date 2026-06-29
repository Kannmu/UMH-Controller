#pragma once
#include "ssd1306.h"
#include "buttons.h"
#include <stddef.h>

typedef enum {
    MENU_FOLDER,
    MENU_ACTION,
    MENU_VALUE_INT,
    MENU_VALUE_ENUM,
    MENU_BACK,
    MENU_DATA_FLOAT
} MenuItemType;

struct MenuPage;
typedef struct MenuPage MenuPage;

typedef void (*MenuActionFn)(void);
typedef void (*MenuRenderFn)(const void* ctx);

typedef enum { EVENT_ENTER, EVENT_EXIT } PageEvent;

typedef struct {
    const char      *label;
    MenuItemType     type;
    union {
        const MenuPage *submenu;
        MenuActionFn    action;
        struct { int *value; int min; int max; int step; const char *fmt; } iv;
        struct { int *value; const char *const*names; int count; } ev;
        struct { uint8_t slot_idx; } df;
    };
    MenuRenderFn     custom_render;
} MenuItem;

struct MenuPage {
    const char    *title;
    const MenuItem *items;
    uint8_t        item_count;
    uint8_t        scroll_rows;               /* scrollable row count (0 = use item_count) */
    bool         (*on_input)(NavAction nav);  /* returns true if navigation was consumed */
    void         (*on_event)(PageEvent event);
};

void GUI_Init(void);
void GUI_Tick(void);
void GUI_Action_ToggleCalibBypass(void);
void GUI_Action_SetDemo(int idx);
void GUI_Action_StartSemiAutoCalib(void);

void GUI_PushPage(const MenuPage *page);
void GUI_PopPage(void);
const MenuPage* GUI_CurrentPage(void);

void GUI_FormatSmartUnits(char *buf, size_t buf_size, float value, const char *suffix, uint8_t decimals);

extern const MenuPage Page_Root;
extern const MenuPage Page_Demo;
extern const MenuPage Page_Calibration;
extern const MenuPage Page_About;
extern const MenuPage Page_SemiCalib;
extern const MenuPage Page_Refresh;
