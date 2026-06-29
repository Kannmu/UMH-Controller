#pragma once
#include "main.h"

typedef enum { BTN_UP = 0, BTN_DOWN, BTN_CONFIRM, BTN_RETURN, BTN_COUNT } ButtonId;
typedef enum { EVT_NONE = 0, EVT_PRESS, EVT_RELEASE, EVT_HOLD, EVT_REPEAT } ButtonEvent;
typedef enum { NAV_NONE, NAV_UP, NAV_DOWN, NAV_CONFIRM, NAV_RETURN } NavAction;

void Buttons_Init(void);
void Buttons_Tick(void);
ButtonEvent Buttons_Get(ButtonId id);
uint8_t Buttons_IsPressed(ButtonId id);
NavAction Buttons_GetNav(void);
