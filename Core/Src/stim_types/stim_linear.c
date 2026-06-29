#include "stimulation.h"
#include "custom_math.h"
#include "stim_types.h"

typedef struct {
    float startPoint[3];
    float endPoint[3];
    int   segments;
} LinearCtx;

static void linear_init(struct Stimulation *stim)
{
    (void)stim;
}

static void linear_deinit(struct Stimulation *stim)
{
    (void)stim;
}

static void linear_update(struct Stimulation *stim, float progress)
{
    LinearCtx *ctx = (LinearCtx *)stim->_ctx;
    int segs = ctx->segments;
    if (segs < 1) segs = 1;

    float total_segs = (float)segs;
    float p = progress * total_segs;
    int cur_seg = (int)p;
    if (cur_seg >= segs) cur_seg = segs - 1;

    float seg_progress = p - (float)cur_seg;
    float pos[3];

    /* Even segments: Start->End, Odd segments: End->Start */
    if (cur_seg % 2 == 0) {
        Vector3Lerp(pos, ctx->startPoint, ctx->endPoint, seg_progress);
    } else {
        Vector3Lerp(pos, ctx->endPoint, ctx->startPoint, seg_progress);
    }

    Set_Point_Focus(pos);
}

/*
 * TLV tags:
 *   0x01-0x03 startPoint[0..2] (float)
 *   0x04-0x06 endPoint[0..2]   (float)
 *   0x08 segments (int32)
 *   0xA0 strength  (float)
 *   0xA1 frequency (float)
 */
static uint8_t linear_deserialize(struct Stimulation *stim,
                                   const uint8_t *data, uint8_t len)
{
    LinearCtx *ctx = (LinearCtx *)stim->_ctx;
    uint8_t i = 0;
    while (i + 2 <= len) {
        uint8_t tag  = data[i]; i++;
        uint8_t flen = data[i]; i++;
        if (i + flen > len) break;
        switch (tag) {
        case 0x01: if (flen == 4) { float v; __builtin_memcpy(&v, &data[i], 4); ctx->startPoint[0] = v; } break;
        case 0x02: if (flen == 4) { float v; __builtin_memcpy(&v, &data[i], 4); ctx->startPoint[1] = v; } break;
        case 0x03: if (flen == 4) { float v; __builtin_memcpy(&v, &data[i], 4); ctx->startPoint[2] = v; } break;
        case 0x04: if (flen == 4) { float v; __builtin_memcpy(&v, &data[i], 4); ctx->endPoint[0] = v; } break;
        case 0x05: if (flen == 4) { float v; __builtin_memcpy(&v, &data[i], 4); ctx->endPoint[1] = v; } break;
        case 0x06: if (flen == 4) { float v; __builtin_memcpy(&v, &data[i], 4); ctx->endPoint[2] = v; } break;
        case 0x08: if (flen == 4) { int32_t v; __builtin_memcpy(&v, &data[i], 4); ctx->segments = (int)v; } break;
        case 0xA0: if (flen == 4) { float v; __builtin_memcpy(&v, &data[i], 4); stim->strength  = v; } break;
        case 0xA1: if (flen == 4) { float v; __builtin_memcpy(&v, &data[i], 4); stim->frequency = v; } break;
        default: break;
        }
        i += flen;
    }
    return 1;
}

const StimTypeDescriptor linear_desc = {
    .type_id     = 2,
    .name        = "Linear",
    .is_static   = 0,
    .init        = linear_init,
    .deinit      = linear_deinit,
    .update      = linear_update,
    .deserialize = linear_deserialize,
};
STIM_TYPE_REGISTER(linear_desc);
