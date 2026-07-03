#include "stimulation.h"
#include "stim_types.h"

typedef struct {
    float position[3];
} TwinTrapCtx;

static void twintrap_init(struct Stimulation *stim)
{
    TwinTrapCtx *ctx = (TwinTrapCtx *)stim->_ctx;
    ctx->position[0] = 0.0f;
    ctx->position[1] = 0.0f;
    ctx->position[2] = DEFAULT_FOCUS_Z;
}

static void twintrap_deinit(struct Stimulation *stim)
{
    (void)stim;
}

static void twintrap_update(struct Stimulation *stim, float progress)
{
    (void)progress;
    TwinTrapCtx *ctx = (TwinTrapCtx *)stim->_ctx;
    Set_Twin_Trap_Focus(ctx->position);
}

/*
 * TLV tags:
 *   0x01-0x03 position[0..2] (float)
 *   0xA0 strength  (float)
 *   0xA1 frequency (float)
 */
static uint8_t twintrap_deserialize(struct Stimulation *stim,
                                     const uint8_t *data, uint8_t len)
{
    TwinTrapCtx *ctx = (TwinTrapCtx *)stim->_ctx;
    uint8_t i = 0;
    while (i + 2 <= len) {
        uint8_t tag  = data[i]; i++;
        uint8_t flen = data[i]; i++;
        if (i + flen > len) break;
        switch (tag) {
        case STIM_TAG_FIELD_0: if (flen == 4) { float v; __builtin_memcpy(&v, &data[i], 4); ctx->position[0] = v; } break;
        case STIM_TAG_FIELD_1: if (flen == 4) { float v; __builtin_memcpy(&v, &data[i], 4); ctx->position[1] = v; } break;
        case STIM_TAG_FIELD_2: if (flen == 4) { float v; __builtin_memcpy(&v, &data[i], 4); ctx->position[2] = v; } break;
        case STIM_TAG_STRENGTH:  if (flen == 4) { float v; __builtin_memcpy(&v, &data[i], 4); stim->strength  = v; } break;
        case STIM_TAG_FREQUENCY: if (flen == 4) { float v; __builtin_memcpy(&v, &data[i], 4); stim->frequency = v; } break;
        default: break;
        }
        i += flen;
    }
    return 1;
}

const StimTypeDescriptor twintrap_desc = {
    .type_id     = 4,
    .name        = "TwinTrap",
    .is_static   = 1,
    .init        = twintrap_init,
    .deinit      = twintrap_deinit,
    .update      = twintrap_update,
    .deserialize = twintrap_deserialize,
};
STIM_TYPE_REGISTER(twintrap_desc);
