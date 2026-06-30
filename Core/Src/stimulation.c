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
    EmptyStimulation.strength  = DMA_STRENGTH_MAX;
    EmptyStimulation.frequency = 0.0f;
    EmptyStimulation.cached_period_us = 0;
    if (point_type && point_type->init)
        point_type->init(&EmptyStimulation);

    /* CurrentStimulation — starts as Point */
    CurrentStimulation = EmptyStimulation;
    strncpy(CurrentStimulation.name, "Current", sizeof(CurrentStimulation.name) - 1);
    CurrentStimulation.frequency = (float)STIMULATION_FREQ;
    CurrentStimulation.cached_period_us = (uint32_t)(US_PER_SEC_F / (float)STIMULATION_FREQ);
}

void Set_Stimulation(const Stimulation *stimulation)
{
    Stimulation sanitized = *stimulation;
    sanitized.strength = DMA_Clamp_Stimulation_Strength(sanitized.strength);

    /* Pre-calculate period */
    if (sanitized.frequency > 0.0f)
        sanitized.cached_period_us = (uint32_t)(US_PER_SEC_F / sanitized.frequency);
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
    if (s.strength  == 0.0f) s.strength  = DMA_STRENGTH_MAX;
    if (s.frequency == 0.0f) s.frequency = (float)STIMULATION_FREQ;

    Set_Stimulation(&s);
}

void Update_Stimulation_State(float progress)
{
    if (Get_Calibration_Mode() == 1 || Get_Phase_Set_Mode() == 1)
        return;

    if (CurrentStimulation.type_desc && CurrentStimulation.type_desc->update)
        CurrentStimulation.type_desc->update(&CurrentStimulation, progress);
}

/* ---- Enable / Disable helpers ---- */

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
