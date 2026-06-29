#include "buttons.h"

#define DEBOUNCE_MS  20
#define HOLD_MS     500
#define REPEAT_MS   120
#define REPEAT_FAST_MS 60
#define FAST_AFTER_MS 2000

static const struct { GPIO_TypeDef *port; uint16_t pin; } btn_map[BTN_COUNT] = {
    [BTN_UP]      = {KEY0_GPIO_Port, KEY0_Pin},
    [BTN_DOWN]    = {KEY1_GPIO_Port, KEY1_Pin},
    [BTN_CONFIRM] = {KEY2_GPIO_Port, KEY2_Pin},
    [BTN_RETURN]  = {KEY3_GPIO_Port, KEY3_Pin},
};

static struct {
    GPIO_PinState raw;
    GPIO_PinState debounced;
    uint32_t      last_change;
    uint32_t      press_time;
    uint32_t      last_repeat;
    ButtonEvent   pending;
    uint8_t       held;
} btns[BTN_COUNT];

void Buttons_Init(void)
{
    for (int i = 0; i < BTN_COUNT; i++) {
        btns[i].debounced = GPIO_PIN_SET;
        btns[i].raw       = GPIO_PIN_SET;
        btns[i].pending   = EVT_NONE;
    }
}

void Buttons_Tick(void)
{
    uint32_t now = HAL_GetTick();
    for (int i = 0; i < BTN_COUNT; i++)
    {
        GPIO_PinState raw = HAL_GPIO_ReadPin(btn_map[i].port, btn_map[i].pin);
        if (raw != btns[i].raw)
        {
            btns[i].raw = raw;
            btns[i].last_change = now;
        }
        if ((now - btns[i].last_change) > DEBOUNCE_MS)
        {
            if (raw != btns[i].debounced)
            {
                btns[i].debounced = raw;
                if (raw == GPIO_PIN_RESET)
                {
                    btns[i].pending = EVT_PRESS;
                    btns[i].press_time = now;
                    btns[i].held = 0;
                }
                else
                {
                    btns[i].pending = EVT_RELEASE;
                }
            }
        }
        /* Hold / repeat */
        if (btns[i].debounced == GPIO_PIN_RESET && (now - btns[i].press_time) > HOLD_MS)
        {
            if (!btns[i].held)
            {
                btns[i].pending = EVT_HOLD;
                btns[i].held = 1;
                btns[i].last_repeat = now;
            }
            uint32_t repeat_ms = ((now - btns[i].press_time) > FAST_AFTER_MS) ? REPEAT_FAST_MS : REPEAT_MS;
            if ((now - btns[i].last_repeat) > repeat_ms)
            {
                btns[i].pending = EVT_REPEAT;
                btns[i].last_repeat = now;
            }
        }
    }
}

ButtonEvent Buttons_Get(ButtonId id)
{
    ButtonEvent e = btns[id].pending;
    btns[id].pending = EVT_NONE;
    return e;
}

uint8_t Buttons_IsPressed(ButtonId id)
{
    return btns[id].debounced == GPIO_PIN_RESET;
}

NavAction Buttons_GetNav(void)
{
    ButtonEvent ev;

    /* priority: CONFIRM / RETURN override directional repeats */
    if (Buttons_Get(BTN_CONFIRM) == EVT_PRESS) return NAV_CONFIRM;
    if (Buttons_Get(BTN_RETURN)  == EVT_PRESS) return NAV_RETURN;

    ev = Buttons_Get(BTN_UP);
    if (ev == EVT_PRESS || ev == EVT_REPEAT) return NAV_UP;

    ev = Buttons_Get(BTN_DOWN);
    if (ev == EVT_PRESS || ev == EVT_REPEAT) return NAV_DOWN;

    return NAV_NONE;
}
