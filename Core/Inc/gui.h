#pragma once
#include "ssd1306.h"
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
};

void GUI_Init(void);
void GUI_Tick(void);
void GUI_Action_ToggleCalibBypass(void);
void GUI_Action_SetDemo(int idx);
void GUI_Action_StartSemiAutoCalib(void);

/* Convenience: push a page onto navigation stack */
void GUI_PushPage(const MenuPage *page);
void GUI_PopPage(void);
const MenuPage* GUI_CurrentPage(void);

/* Smart unit formatting */
void GUI_FormatSmartUnits(char *buf, size_t buf_size, float value, const char *suffix, uint8_t decimals);

/* Semi-auto calibration page */
extern const MenuPage Page_SemiCalib;
extern const MenuPage Page_About;
