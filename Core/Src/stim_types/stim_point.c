#include "stimulation.h"
#include "stim_types.h"

/* ---- Point type context (fits in stim->_ctx[64]) ---- */
typedef struct {
    float position[3];
} PointCtx;

static void point_init(struct Stimulation *stim)
{
    PointCtx *ctx = (PointCtx *)stim->_ctx;
    ctx->position[0] = 0.0f;
    ctx->position[1] = 0.0f;
    ctx->position[2] = DEFAULT_FOCUS_Z;
}

static void point_deinit(struct Stimulation *stim)
{
    (void)stim;
}

static void point_update(struct Stimulation *stim, float progress)
{
    (void)progress;
    PointCtx *ctx = (PointCtx *)stim->_ctx;
    Set_Point_Focus(ctx->position);
}

/*
 * TLV tags for deserialize:
 *   0x01 position[0] (float, 4B)
 *   0x02 position[1] (float, 4B)
 *   0x03 position[2] (float, 4B)
 *   0xA0 strength     (float, 4B)
 *   0xA1 frequency    (float, 4B)
 */
static uint8_t point_deserialize(struct Stimulation *stim,
                                  const uint8_t *data, uint8_t len)
{
    PointCtx *ctx = (PointCtx *)stim->_ctx;
    uint8_t i = 0;
    while (i + 2 <= len) {
        uint8_t tag  = data[i]; i++;
        uint8_t flen = data[i]; i++;
        if (i + flen > len) break;
        switch (tag) {
        case 0x01: if (flen == 4) { float v; __builtin_memcpy(&v, &data[i], 4); ctx->position[0] = v; } break;
        case 0x02: if (flen == 4) { float v; __builtin_memcpy(&v, &data[i], 4); ctx->position[1] = v; } break;
        case 0x03: if (flen == 4) { float v; __builtin_memcpy(&v, &data[i], 4); ctx->position[2] = v; } break;
        case 0xA0: if (flen == 4) { float v; __builtin_memcpy(&v, &data[i], 4); stim->strength  = v; } break;
        case 0xA1: if (flen == 4) { float v; __builtin_memcpy(&v, &data[i], 4); stim->frequency = v; } break;
        default: break;
        }
        i += flen;
    }
    return 1;
}

const StimTypeDescriptor point_desc = {
    .type_id     = 0,
    .name        = "Point",
    .is_static   = 1,
    .init        = point_init,
    .deinit      = point_deinit,
    .update      = point_update,
    .deserialize = point_deserialize,
};
STIM_TYPE_REGISTER(point_desc);
