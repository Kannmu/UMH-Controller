#ifndef STIM_TYPES_H
#define STIM_TYPES_H

#include <stdint.h>

/* Forward declaration */
struct Stimulation;

/* ---- Stimulation Type Descriptor (vtable) ---- */
typedef void (*Stim_Init_Fn)(struct Stimulation *stim);
typedef void (*Stim_Deinit_Fn)(struct Stimulation *stim);
typedef void (*Stim_Update_Fn)(struct Stimulation *stim, float progress);
typedef uint8_t (*Stim_Deserialize_Fn)(struct Stimulation *stim,
                                        const uint8_t *data, uint8_t len);

typedef struct {
    uint8_t             type_id;
    const char         *name;
    uint8_t             is_static;   /* waveform identical across all samples */
    Stim_Init_Fn        init;
    Stim_Deinit_Fn      deinit;
    Stim_Update_Fn      update;
    Stim_Deserialize_Fn deserialize;
} StimTypeDescriptor;

typedef void (*Stim_Demo_Populate_Fn)(struct Stimulation *stim);

/* ---- Demo Preset Descriptor ---- */
typedef struct {
    const char                *name;
    uint8_t                    type_id;
    const StimTypeDescriptor  *type_desc;
    Stim_Demo_Populate_Fn      populate;
} StimDemoDescriptor;

/* ---- Linker-Section Registration Macros ---- */
#define STIM_TYPE_REGISTER(desc_var) \
    __attribute__((section(".stim_type_registry"), used)) \
    static const StimTypeDescriptor *const _stim_type_##desc_var = &(desc_var)

#define STIM_DEMO_REGISTER(desc_var) \
    __attribute__((section(".demo_registry"), used)) \
    static const StimDemoDescriptor *const _demo_##desc_var = &(desc_var)

/* ---- Runtime Registry Access (implemented in stimulation.c) ---- */
uint8_t                   Stim_Num_Types(void);
const StimTypeDescriptor* Stim_Get_Type_By_Id(uint8_t type_id);
const StimTypeDescriptor* Stim_Get_Type_By_Index(uint8_t index);
uint8_t                   Stim_Get_Index_By_Type_Id(uint8_t type_id);
const StimTypeDescriptor* Stim_Get_Type_By_Name(const char *name);

uint8_t                   Stim_Num_Demos(void);
const StimDemoDescriptor* Stim_Get_Demo_By_Index(uint8_t index);
const StimDemoDescriptor* Stim_Get_Demo_By_Name(const char *name, uint8_t name_len);
int                       Stim_Get_Demo_Index(const StimDemoDescriptor *demo);

#endif /* STIM_TYPES_H */
