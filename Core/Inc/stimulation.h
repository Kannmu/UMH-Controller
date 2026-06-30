#pragma once
#include "main.h"
#include "transducer.h"
#include "stim_types.h"

#define STIMULATION_FREQ (200U)
#define STIMULATION_PERIOD (1.0 / STIMULATION_FREQ)
#define NUM_STIMULATION_SAMPLES (uint32_t)(TRANSDUCER_BASE_FREQ / STIMULATION_FREQ)

#define STIM_CTX_SIZE      64   /* inline context buffer for type-specific cached data */
#define DEFAULT_FOCUS_Z     0.1f

struct Stimulation {
    /* Common fields across all types */
    char         name[32];
    uint8_t      type_id;
    float        strength;
    float        frequency;
    uint32_t     cached_period_us;

    /* Vtable pointer to the type's descriptor */
    const StimTypeDescriptor *type_desc;

    /* Inline context buffer (type-specific cached data) */
    uint8_t      _ctx[STIM_CTX_SIZE];
};

/* ---- Extern globals ---- */
extern int                  phase_set_mode;
extern int                  demo_mode;
extern struct Stimulation   CurrentStimulation;
extern struct Stimulation   EmptyStimulation;

/* ---- Accessor inlines (zero-overhead, used by dma_manager.c hot path) ---- */
static inline uint8_t  Stim_Get_Type_Id(const struct Stimulation *s) {
    return s->type_id;
}
static inline float    Stim_Get_Strength(const struct Stimulation *s) {
    return s->strength;
}
static inline int      Stim_Is_Static(const struct Stimulation *s) {
    return s->type_desc && s->type_desc->is_static;
}

/* ---- Public API ---- */
void  Stim_Init(void);
void  Switch_Demo_Mode(void);
int   Get_Demo_Mode(void);
int   Get_Num_Demo_Stimulations(void);
int   Get_Phase_Set_Mode(void);
int   Get_Stimulation_Enabled(void);
void  Stimulation_Enable(void);
void  Stimulation_Disable(void);
void  Set_Stimulation(const struct Stimulation *stim);
void  Set_Stimulation_From_Demo(const StimDemoDescriptor *demo);
void  Update_Stimulation_State(float progress);
