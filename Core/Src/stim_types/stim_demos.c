#include "stimulation.h"
#include "stim_types.h"
#include "dma_manager.h"
#include <string.h>

/* ================================================================
 * Each demo defines a populate() that fills in the Stimulation
 * (common fields + typed ctx). The caller then invokes the type's
 * init() for deferred computation (e.g. cached basis vectors).
 * ================================================================ */

/* --- DLM_2: Discrete, 2 segments, radius=6.25mm --- */
static void dlm2_populate(struct Stimulation *stim)
{
    stim->strength  = DMA_STRENGTH_MAX;
    stim->frequency = (float)STIMULATION_FREQ;
    struct { float position[3]; float normalVector[3]; float radius; int segments;
             float cached_u[3]; float cached_v[3]; } *ctx;
    ctx = (void *)stim->_ctx;
    memset(ctx, 0, sizeof(*ctx));
    ctx->position[0]      = 0.0f;
    ctx->position[1]      = 0.0f;
    ctx->position[2]      = DEFAULT_FOCUS_Z;
    ctx->normalVector[0]  = 0.0f;
    ctx->normalVector[1]  = 0.0f;
    ctx->normalVector[2]  = 1.0f;
    ctx->radius           = 6.25e-3f;
    ctx->segments         = 2;
}

/* --- DLM_3: Discrete, 3 segments, radius=4.81mm --- */
static void dlm3_populate(struct Stimulation *stim)
{
    stim->strength  = DMA_STRENGTH_MAX;
    stim->frequency = (float)STIMULATION_FREQ;
    struct { float position[3]; float normalVector[3]; float radius; int segments;
             float cached_u[3]; float cached_v[3]; } *ctx;
    ctx = (void *)stim->_ctx;
    memset(ctx, 0, sizeof(*ctx));
    ctx->position[0]      = 0.0f;
    ctx->position[1]      = 0.0f;
    ctx->position[2]      = DEFAULT_FOCUS_Z;
    ctx->normalVector[0]  = 0.0f;
    ctx->normalVector[1]  = 0.0f;
    ctx->normalVector[2]  = 1.0f;
    ctx->radius           = 4.81e-3f;
    ctx->segments         = 3;
}

/* --- ULM_L: Linear, 1 segment, Y range [-15mm, +15mm] --- */
static void ulm_l_populate(struct Stimulation *stim)
{
    stim->strength  = DMA_STRENGTH_MAX;
    stim->frequency = (float)STIMULATION_FREQ;
    struct { float startPoint[3]; float endPoint[3]; int segments; } *ctx;
    ctx = (void *)stim->_ctx;
    memset(ctx, 0, sizeof(*ctx));
    ctx->startPoint[0] = 0.0f;
    ctx->startPoint[1] = 0.015f;
    ctx->startPoint[2] = DEFAULT_FOCUS_Z;
    ctx->endPoint[0]   = 0.0f;
    ctx->endPoint[1]   = -0.015f;
    ctx->endPoint[2]   = DEFAULT_FOCUS_Z;
    ctx->segments      = 1;
}

/* --- LM_L: Linear, 2 segments, Y range [-7.5mm, +7.5mm] --- */
static void lm_l_populate(struct Stimulation *stim)
{
    stim->strength  = DMA_STRENGTH_MAX;
    stim->frequency = (float)STIMULATION_FREQ;
    struct { float startPoint[3]; float endPoint[3]; int segments; } *ctx;
    ctx = (void *)stim->_ctx;
    memset(ctx, 0, sizeof(*ctx));
    ctx->startPoint[0] = 0.0f;
    ctx->startPoint[1] = 7.5e-3f;
    ctx->startPoint[2] = DEFAULT_FOCUS_Z;
    ctx->endPoint[0]   = 0.0f;
    ctx->endPoint[1]   = -7.5e-3f;
    ctx->endPoint[2]   = DEFAULT_FOCUS_Z;
    ctx->segments      = 2;
}

/* --- LM_C: Circular, radius=4.77mm, normal Z --- */
static void lm_c_populate(struct Stimulation *stim)
{
    stim->strength  = DMA_STRENGTH_MAX;
    stim->frequency = (float)STIMULATION_FREQ;
    struct { float position[3]; float normalVector[3]; float radius;
             float cached_u[3]; float cached_v[3]; } *ctx;
    ctx = (void *)stim->_ctx;
    memset(ctx, 0, sizeof(*ctx));
    ctx->position[0]     = 0.0f;
    ctx->position[1]     = 0.0f;
    ctx->position[2]     = DEFAULT_FOCUS_Z;
    ctx->normalVector[0] = 0.0f;
    ctx->normalVector[1] = 0.0f;
    ctx->normalVector[2] = 1.0f;
    ctx->radius          = 4.77e-3f;
}

/* --- TwinTrap: position at (0, 0, 0.1) --- */
static void twintrap_populate(struct Stimulation *stim)
{
    stim->strength  = DMA_STRENGTH_MAX;
    stim->frequency = (float)STIMULATION_FREQ;
    struct { float position[3]; } *ctx;
    ctx = (void *)stim->_ctx;
    memset(ctx, 0, sizeof(*ctx));
    ctx->position[0] = 0.0f;
    ctx->position[1] = 0.0f;
    ctx->position[2] = DEFAULT_FOCUS_Z;
}

/* ---- Extern references to type descriptors ---- */
extern const StimTypeDescriptor discrete_desc;
extern const StimTypeDescriptor linear_desc;
extern const StimTypeDescriptor circular_desc;
extern const StimTypeDescriptor twintrap_desc;

/* ---- Demo descriptors with linker-section registration ---- */
static const StimDemoDescriptor demo_dlm2 = {
    .name      = "DLM_2",
    .type_id   = 1,  /* Discrete */
    .type_desc = &discrete_desc,
    .populate  = dlm2_populate,
};
STIM_DEMO_REGISTER(demo_dlm2);

static const StimDemoDescriptor demo_dlm3 = {
    .name      = "DLM_3",
    .type_id   = 1,  /* Discrete */
    .type_desc = &discrete_desc,
    .populate  = dlm3_populate,
};
STIM_DEMO_REGISTER(demo_dlm3);

static const StimDemoDescriptor demo_ulm_l = {
    .name      = "ULM_L",
    .type_id   = 2,  /* Linear */
    .type_desc = &linear_desc,
    .populate  = ulm_l_populate,
};
STIM_DEMO_REGISTER(demo_ulm_l);

static const StimDemoDescriptor demo_lm_l = {
    .name      = "LM_L",
    .type_id   = 2,  /* Linear */
    .type_desc = &linear_desc,
    .populate  = lm_l_populate,
};
STIM_DEMO_REGISTER(demo_lm_l);

static const StimDemoDescriptor demo_lm_c = {
    .name      = "LM_C",
    .type_id   = 3,  /* Circular */
    .type_desc = &circular_desc,
    .populate  = lm_c_populate,
};
STIM_DEMO_REGISTER(demo_lm_c);

static const StimDemoDescriptor demo_twintrap = {
    .name      = "TwinTrap",
    .type_id   = 4,  /* TwinTrap */
    .type_desc = &twintrap_desc,
    .populate  = twintrap_populate,
};
STIM_DEMO_REGISTER(demo_twintrap);
