#define _USE_MATH_DEFINES
#include "stimulation.h"
#include "calibration.h"
#include "utiles.h"
#include "custom_math.h"
#include "dma_manager.h"
#include "stim_types.h"
#include <string.h>
#include <stdlib.h>

int phase_set_mode = 0;
int is_stimulation_enabled = 1;

int demo_mode = -1;

Stimulation CurrentStimulation;
Stimulation EmptyStimulation;

/* ---- Registry boundary symbols (provided by linker script) ---- */
extern const StimTypeDescriptor *const _stim_type_registry_start[];
extern const StimTypeDescriptor *const _stim_type_registry_end[];
extern const StimDemoDescriptor *const _demo_registry_start[];
extern const StimDemoDescriptor *const _demo_registry_end[];

/* ---- Registry query functions ---- */

uint8_t Stim_Num_Types(void)
{
    return (uint8_t)(_stim_type_registry_end - _stim_type_registry_start);
}

const StimTypeDescriptor* Stim_Get_Type_By_Id(uint8_t type_id)
{
    uint8_t n = Stim_Num_Types();
    for (uint8_t i = 0; i < n; i++) {
        if (_stim_type_registry_start[i]->type_id == type_id)
            return _stim_type_registry_start[i];
    }
    return NULL;
}

const StimTypeDescriptor* Stim_Get_Type_By_Index(uint8_t index)
{
    if (index >= Stim_Num_Types()) return NULL;
    return _stim_type_registry_start[index];
}

uint8_t Stim_Get_Index_By_Type_Id(uint8_t type_id)
{
    uint8_t n = Stim_Num_Types();
    for (uint8_t i = 0; i < n; i++) {
        if (_stim_type_registry_start[i]->type_id == type_id)
            return i;
    }
    return 0xFF; /* sentinel */
}

const StimTypeDescriptor* Stim_Get_Type_By_Name(const char *name)
{
    uint8_t n = Stim_Num_Types();
    for (uint8_t i = 0; i < n; i++) {
        if (strncmp(_stim_type_registry_start[i]->name, name, 32) == 0)
            return _stim_type_registry_start[i];
    }
    return NULL;
}

uint8_t Stim_Num_Demos(void)
{
    return (uint8_t)(_demo_registry_end - _demo_registry_start);
}

const StimDemoDescriptor* Stim_Get_Demo_By_Index(uint8_t index)
{
    if (index >= Stim_Num_Demos()) return NULL;
    return _demo_registry_start[index];
}

const StimDemoDescriptor* Stim_Get_Demo_By_Name(const char *name, uint8_t name_len)
{
    uint8_t n = Stim_Num_Demos();
    for (uint8_t i = 0; i < n; i++) {
        if (strncmp(_demo_registry_start[i]->name, name, name_len) == 0
            && _demo_registry_start[i]->name[name_len] == '\0')
            return _demo_registry_start[i];
    }
    return NULL;
}

int Stim_Get_Demo_Index(const StimDemoDescriptor *demo)
{
    uint8_t n = Stim_Num_Demos();
    for (uint8_t i = 0; i < n; i++) {
        if (_demo_registry_start[i] == demo)
            return (int)i;
    }
    return -1;
}

/* ================================================================
 *  Core engine
 * ================================================================ */

void Stim_Init(void)
{
    const StimTypeDescriptor *point_type = Stim_Get_Type_By_Id(0);

    /* Allocate the per-type delta-time array — Stim_Num_Types() entries */
    {
        uint8_t n = Stim_Num_Types();
        updateDMABufferDeltaTimeByType = (double *)calloc(n, sizeof(double));
    }

    /* EmptyStimulation — safe fallback */
    memset(&EmptyStimulation, 0, sizeof(EmptyStimulation));
    strncpy(EmptyStimulation.name, "Empty", sizeof(EmptyStimulation.name) - 1);
    EmptyStimulation.type_id   = 0;
    EmptyStimulation.type_desc = point_type;
    EmptyStimulation.strength  = 100.0f;
    EmptyStimulation.frequency = 0.0f;
    EmptyStimulation.cached_period_us = 0;
    if (point_type && point_type->init)
        point_type->init(&EmptyStimulation);

    /* CurrentStimulation — starts as Point */
    CurrentStimulation = EmptyStimulation;
    strncpy(CurrentStimulation.name, "Current", sizeof(CurrentStimulation.name) - 1);
    CurrentStimulation.frequency = 200.0f;
    CurrentStimulation.cached_period_us = (uint32_t)(1e6f / 200.0f);
}

void Set_Stimulation(const Stimulation *stimulation)
{
    Stimulation sanitized = *stimulation;
    sanitized.strength = DMA_Clamp_Stimulation_Strength(sanitized.strength);

    /* Pre-calculate period */
    if (sanitized.frequency > 0.0f)
        sanitized.cached_period_us = (uint32_t)(1e6f / sanitized.frequency);
    else
        sanitized.cached_period_us = 0;

    /* Deinit old type */
    if (CurrentStimulation.type_desc && CurrentStimulation.type_desc->deinit)
        CurrentStimulation.type_desc->deinit(&CurrentStimulation);

    /* Copy in new stimulation */
    CurrentStimulation = sanitized;

    /* Init new type (precomputes cached vectors etc.) */
    if (CurrentStimulation.type_desc && CurrentStimulation.type_desc->init)
        CurrentStimulation.type_desc->init(&CurrentStimulation);

    Update_Full_Waveform_Buffer();
}

void Set_Stimulation_From_Demo(const StimDemoDescriptor *demo)
{
    if (!demo || !demo->type_desc || !demo->populate) return;

    Stimulation s;
    memset(&s, 0, sizeof(s));
    strncpy(s.name, demo->name, sizeof(s.name) - 1);
    s.type_id   = demo->type_id;
    s.type_desc = demo->type_desc;

    /* Populate type-specific context */
    demo->populate(&s);

    /* Set defaults for common fields if not set by populate */
    if (s.strength  == 0.0f) s.strength  = 100.0f;
    if (s.frequency == 0.0f) s.frequency = 200.0f;

    Set_Stimulation(&s);
}

void Update_Stimulation_State(float progress)
{
    if (Get_Calibration_Mode() == 1 || Get_Phase_Set_Mode() == 1)
        return;

    if (CurrentStimulation.type_desc && CurrentStimulation.type_desc->update)
        CurrentStimulation.type_desc->update(&CurrentStimulation, progress);
}

/* ================================================================
 *  Button / Demo switching
 * ================================================================ */

void Switch_Demo_Mode(void)
{
    if (Get_Calibration_Mode())
        return;

    static GPIO_PinState debouncedState = GPIO_PIN_SET;
    static GPIO_PinState lastRawState = GPIO_PIN_SET;
    static uint32_t lastDebounceTime = 0;
    const uint32_t debounceDelay = 50;

    GPIO_PinState currentRawState = HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);

    if (currentRawState != lastRawState)
        lastDebounceTime = HAL_GetTick();
    lastRawState = currentRawState;

    if ((HAL_GetTick() - lastDebounceTime) > debounceDelay)
    {
        if (currentRawState != debouncedState)
        {
            if (debouncedState == GPIO_PIN_SET && currentRawState == GPIO_PIN_RESET)
            {
                uint8_t num = Stim_Num_Demos();
                if (num > 0) {
                    demo_mode = (demo_mode + 1) % num;
                    const StimDemoDescriptor *d = Stim_Get_Demo_By_Index((uint8_t)demo_mode);
                    if (d) Set_Stimulation_From_Demo(d);
                }
            }
            debouncedState = currentRawState;
        }
    }
}

/* ================================================================
 *  Enable / Disable helpers
 * ================================================================ */

int Get_Stimulation_Enabled(void)
{
    return is_stimulation_enabled;
}

void Stimulation_Enable(void)
{
    if (is_stimulation_enabled) return;
    is_stimulation_enabled = 1;
    Update_Full_Waveform_Buffer();
}

void Stimulation_Disable(void)
{
    if (!is_stimulation_enabled) return;
    is_stimulation_enabled = 0;
    Update_Full_Waveform_Buffer();
}

/* ================================================================
 *  Passthrough accessors
 * ================================================================ */

int Get_Num_Demo_Stimulations(void)
{
    return (int)Stim_Num_Demos();
}

int Get_Demo_Mode(void)
{
    return demo_mode;
}

int Get_Phase_Set_Mode(void)
{
    return phase_set_mode;
}
