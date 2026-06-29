#include "stimulation.h"
#include "custom_math.h"
#include "stim_types.h"
#include <math.h>

typedef struct {
    float position[3];
    float normalVector[3];
    float radius;
    float cached_u[3];
    float cached_v[3];
} CircularCtx;

static void circular_init(struct Stimulation *stim)
{
    CircularCtx *ctx = (CircularCtx *)stim->_ctx;

    /* Normalize normalVector and compute orthonormal basis */
    float n[3] = {ctx->normalVector[0], ctx->normalVector[1], ctx->normalVector[2]};
    Vector3Normalize(n);
    ctx->normalVector[0] = n[0];
    ctx->normalVector[1] = n[1];
    ctx->normalVector[2] = n[2];

    float t_vec[3];
    if (fabsf(n[0]) < 0.9f) {
        t_vec[0] = 1.0f; t_vec[1] = 0.0f; t_vec[2] = 0.0f;
    } else {
        t_vec[0] = 0.0f; t_vec[1] = 1.0f; t_vec[2] = 0.0f;
    }

    Vector3Cross(ctx->cached_u, t_vec, n);
    Vector3Normalize(ctx->cached_u);
    Vector3Cross(ctx->cached_v, n, ctx->cached_u);
}

static void circular_deinit(struct Stimulation *stim)
{
    (void)stim;
}

static void circular_update(struct Stimulation *stim, float progress)
{
    CircularCtx *ctx = (CircularCtx *)stim->_ctx;
    float angle = 2.0f * (float)M_PI * progress;
    float cos_a = cosf(angle);
    float sin_a = sinf(angle);

    float pos[3];
    pos[0] = ctx->position[0] + ctx->radius * (cos_a * ctx->cached_u[0] + sin_a * ctx->cached_v[0]);
    pos[1] = ctx->position[1] + ctx->radius * (cos_a * ctx->cached_u[1] + sin_a * ctx->cached_v[1]);
    pos[2] = ctx->position[2] + ctx->radius * (cos_a * ctx->cached_u[2] + sin_a * ctx->cached_v[2]);

    Set_Point_Focus(pos);
}

/*
 * TLV tags:
 *   0x01-0x03 position[0..2] (float)
 *   0x04-0x06 normalVector[0..2] (float)
 *   0x07 radius    (float)
 *   0xA0 strength  (float)
 *   0xA1 frequency (float)
 */
static uint8_t circular_deserialize(struct Stimulation *stim,
                                     const uint8_t *data, uint8_t len)
{
    CircularCtx *ctx = (CircularCtx *)stim->_ctx;
    uint8_t i = 0;
    while (i + 2 <= len) {
        uint8_t tag  = data[i]; i++;
        uint8_t flen = data[i]; i++;
        if (i + flen > len) break;
        switch (tag) {
        case 0x01: if (flen == 4) { float v; __builtin_memcpy(&v, &data[i], 4); ctx->position[0] = v; } break;
        case 0x02: if (flen == 4) { float v; __builtin_memcpy(&v, &data[i], 4); ctx->position[1] = v; } break;
        case 0x03: if (flen == 4) { float v; __builtin_memcpy(&v, &data[i], 4); ctx->position[2] = v; } break;
        case 0x04: if (flen == 4) { float v; __builtin_memcpy(&v, &data[i], 4); ctx->normalVector[0] = v; } break;
        case 0x05: if (flen == 4) { float v; __builtin_memcpy(&v, &data[i], 4); ctx->normalVector[1] = v; } break;
        case 0x06: if (flen == 4) { float v; __builtin_memcpy(&v, &data[i], 4); ctx->normalVector[2] = v; } break;
        case 0x07: if (flen == 4) { float v; __builtin_memcpy(&v, &data[i], 4); ctx->radius = v; } break;
        case 0xA0: if (flen == 4) { float v; __builtin_memcpy(&v, &data[i], 4); stim->strength  = v; } break;
        case 0xA1: if (flen == 4) { float v; __builtin_memcpy(&v, &data[i], 4); stim->frequency = v; } break;
        default: break;
        }
        i += flen;
    }
    return 1;
}

const StimTypeDescriptor circular_desc = {
    .type_id     = 3,
    .name        = "Circular",
    .is_static   = 0,
    .init        = circular_init,
    .deinit      = circular_deinit,
    .update      = circular_update,
    .deserialize = circular_deserialize,
};
STIM_TYPE_REGISTER(circular_desc);
