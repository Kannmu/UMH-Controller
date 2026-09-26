#include "motion_engine.h"
#include "umh_fast_math.h"
#include "main.h"
#include <math.h>
#include <string.h>

static int motion_emit_dark_vortex(umh_motion_engine_t *engine,
                                   umh_spatial_renderer_t *renderer,
                                   umh_output_frame_t *frame,
                                   float cx, float cy, float cz,
                                   float base_level);
static int motion_emit_vortex(umh_motion_engine_t *engine,
                              umh_spatial_renderer_t *renderer,
                              umh_output_frame_t *frame,
                              float cx, float cy, float cz,
                              float base_level);

#define UMH_TWO_PI 6.28318530717958647692f
#define UMH_SAFE_X_UM 100000
#define UMH_SAFE_Y_UM 100000
#define UMH_SAFE_Z_MIN_UM 10000
#define UMH_SAFE_Z_MAX_UM 300000

static uint16_t motion_read_u16(const uint8_t *p)
{
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int motion_lock(umh_motion_engine_t *engine)
{
  if (engine == NULL || engine->lock == NULL) return -1;
  return osMutexAcquire(engine->lock, osWaitForever) == osOK ? 0 : -1;
}

static void motion_unlock(umh_motion_engine_t *engine)
{
  if (engine != NULL && engine->lock != NULL) (void)osMutexRelease(engine->lock);
}

static void motion_record_service(umh_motion_engine_t *engine, uint32_t start_cycles)
{
  uint32_t cycles;
  if (engine == NULL) return;
  cycles = DWT->CYCCNT - start_cycles;
  engine->service_last_cycles = cycles;
  if (cycles > engine->service_max_cycles) engine->service_max_cycles = cycles;
  engine->service_sum_cycles += cycles;
  if (engine->service_count != 0xFFFFFFFFu) engine->service_count++;
}

static void motion_record_render(umh_motion_engine_t *engine, uint32_t cycles)
{
  if (cycles > engine->render_max_cycles) engine->render_max_cycles = cycles;
  engine->render_sum_cycles += cycles;
}

static void motion_record_submit(umh_motion_engine_t *engine, uint32_t cycles)
{
  if (cycles > engine->submit_max_cycles) engine->submit_max_cycles = cycles;
  engine->submit_sum_cycles += cycles;
}

static void motion_clamp_position(float *x, float *y, float *z)
{
  if (*x > (float)UMH_SAFE_X_UM) *x = (float)UMH_SAFE_X_UM;
  if (*x < -(float)UMH_SAFE_X_UM) *x = -(float)UMH_SAFE_X_UM;
  if (*y > (float)UMH_SAFE_Y_UM) *y = (float)UMH_SAFE_Y_UM;
  if (*y < -(float)UMH_SAFE_Y_UM) *y = -(float)UMH_SAFE_Y_UM;
  if (*z > (float)UMH_SAFE_Z_MAX_UM) *z = (float)UMH_SAFE_Z_MAX_UM;
  if (*z < (float)UMH_SAFE_Z_MIN_UM) *z = (float)UMH_SAFE_Z_MIN_UM;
}

static void motion_rebuild_pattern(umh_motion_engine_t *engine)
{
  uint8_t i;
  uint16_t span;
  float r = (float)engine->trap_radius_um;
  engine->pattern_count = 1u;
  memset(engine->pattern_offsets, 0, sizeof(engine->pattern_offsets));
  memset(engine->pattern_phases, 0, sizeof(engine->pattern_phases));
  if (engine->trap_mode == 0u) return;
  if (r < 500.0f) r = 500.0f;
  span = engine->trap_phase_span == 0u ? 256u : engine->trap_phase_span;
  switch (engine->trap_mode) {
    case 1u: /* axial twin with opposite phase: pressure node at target */
      engine->pattern_count = 2u;
      engine->pattern_offsets[0][2] = r;
      engine->pattern_offsets[1][2] = -r;
      engine->pattern_phases[0] = 0u;
      engine->pattern_phases[1] = 128u;
      break;
    case 2u: /* lateral twin with opposite phase: central pressure node */
      engine->pattern_count = 2u;
      engine->pattern_offsets[0][1] = r;
      engine->pattern_offsets[1][1] = -r;
      engine->pattern_phases[0] = 0u;
      engine->pattern_phases[1] = 128u;
      break;
    case 3u: /* ring with alternating phase: central node */
      engine->pattern_count = 8u;
      for (i = 0u; i < engine->pattern_count; ++i) {
        float angle = UMH_TWO_PI * (float)i / (float)engine->pattern_count;
        engine->pattern_offsets[i][0] = r * cosf(angle);
        engine->pattern_offsets[i][1] = r * sinf(angle);
        engine->pattern_offsets[i][2] = 0.0f;
        engine->pattern_phases[i] = (i & 1u) != 0u ? 128u : 0u;
      }
      break;
    case 4u: /* ring with orbital phase ramp: hollow / vortex-like core */
      engine->pattern_count = 8u;
      for (i = 0u; i < engine->pattern_count; ++i) {
        float angle = UMH_TWO_PI * (float)i / (float)engine->pattern_count;
        engine->pattern_offsets[i][0] = r * cosf(angle);
        engine->pattern_offsets[i][1] = r * sinf(angle);
        engine->pattern_offsets[i][2] = 0.0f;
        engine->pattern_phases[i] = (uint8_t)(((uint32_t)i * span) / engine->pattern_count);
      }
      break;
    case 5u: /* two coaxial rings with opposite carrier phase */
      engine->pattern_count = 8u;
      for (i = 0u; i < 4u; ++i) {
        float angle = UMH_TWO_PI * (float)i / 4.0f;
        engine->pattern_offsets[i][0] = r * cosf(angle);
        engine->pattern_offsets[i][1] = r * sinf(angle);
        engine->pattern_offsets[i][2] = 0.5f * r;
        engine->pattern_phases[i] = 0u;
        engine->pattern_offsets[i + 4u][0] = r * cosf(angle);
        engine->pattern_offsets[i + 4u][1] = r * sinf(angle);
        engine->pattern_offsets[i + 4u][2] = -0.5f * r;
        engine->pattern_phases[i + 4u] = 128u;
      }
      break;
    default:
      engine->pattern_count = 1u;
      break;
  }
}

static void motion_point_values(const umh_motion_engine_t *engine, uint16_t index,
                                float *x, float *y, float *z,
                                float *level, float *palette)
{
  const umh_motion_point_wire_t *p = &engine->points[index];
  *x = (float)p->x_10um * 10.0f;
  *y = (float)p->y_10um * 10.0f;
  *z = (float)p->z_10um * 10.0f;
  *level = (float)p->level;
  *palette = (float)(p->palette & 0x0Fu);
}

static float motion_catmull_rom(float p0, float p1, float p2, float p3, float t)
{
  float t2 = t * t;
  float t3 = t2 * t;
  return 0.5f * ((2.0f * p1) +
                 (-p0 + p2) * t +
                 (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
                 (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);
}

static void motion_sample_path(const umh_motion_engine_t *engine, float phase,
                               float *x, float *y, float *z,
                               float *level, float *palette)
{
  uint16_t count = engine->point_count;
  uint16_t segments;
  uint16_t i1, i2, i0, i3;
  float s, frac;
  float p0x, p0y, p0z, p1x, p1y, p1z, p2x, p2y, p2z, p3x, p3y, p3z;
  float level1, level2, pal1, pal2;
  float dummy0, dummy1;
  uint8_t looped = (engine->flags & UMH_MOTION_FLAG_LOOP) != 0u ? 1u : 0u;

  if (count == 0u) {
    *x = 0.0f; *y = 0.0f; *z = 0.0f; *level = 0.0f; *palette = 0.0f;
    return;
  }
  if (count == 1u) {
    motion_point_values(engine, 0u, x, y, z, level, palette);
    *z += (float)engine->z_offset_um;
    return;
  }
  segments = looped != 0u ? count : (uint16_t)(count - 1u);
  if (segments == 0u) segments = 1u;
  s = phase * (float)segments;
  if (s < 0.0f) s = 0.0f;
  i1 = (uint16_t)s;
  frac = s - (float)i1;
  if (i1 >= segments) {
    i1 = (uint16_t)(segments - 1u);
    frac = 1.0f;
  }
  if (looped != 0u) {
    i0 = (uint16_t)((i1 + count - 1u) % count);
    i2 = (uint16_t)((i1 + 1u) % count);
    i3 = (uint16_t)((i1 + 2u) % count);
  } else {
    i0 = i1 > 0u ? (uint16_t)(i1 - 1u) : i1;
    i2 = (uint16_t)(i1 + 1u < count ? i1 + 1u : i1);
    i3 = (uint16_t)(i1 + 2u < count ? i1 + 2u : i2);
  }
  motion_point_values(engine, i0, &p0x, &p0y, &p0z, &dummy0, &dummy1);
  motion_point_values(engine, i1, &p1x, &p1y, &p1z, &level1, &pal1);
  motion_point_values(engine, i2, &p2x, &p2y, &p2z, &level2, &pal2);
  motion_point_values(engine, i3, &p3x, &p3y, &p3z, &dummy0, &dummy1);
  if ((engine->points[i1].palette & UMH_MOTION_POINT_JUMP) != 0u) {
    /* Per-point zero-transit flag: jump directly to the next stroke/point
     * and hold it for this segment.  Only used by intentional haptics
     * teleports; levitation paths leave bit7 clear. */
    *x = p2x;
    *y = p2y;
    *z = p2z + (float)engine->z_offset_um;
    *level = level2;
    *palette = pal2;
    return;
  }
  if ((engine->flags & UMH_MOTION_FLAG_STEP) != 0u) {
    /* Zero-transit haptics: hold one control point for the whole segment and
     * jump exactly at the segment boundary.  Used by letter writing where
     * teleporting between strokes is intentional; levitation paths never set
     * this flag. */
    *x = p1x;
    *y = p1y;
    *z = p1z + (float)engine->z_offset_um;
    *level = level1;
    *palette = pal1;
    return;
  }
  if ((engine->flags & UMH_MOTION_FLAG_LINEAR) != 0u || count < 4u) {
    *x = p1x + (p2x - p1x) * frac;
    *y = p1y + (p2y - p1y) * frac;
    *z = p1z + (p2z - p1z) * frac;
  } else {
    *x = motion_catmull_rom(p0x, p1x, p2x, p3x, frac);
    *y = motion_catmull_rom(p0y, p1y, p2y, p3y, frac);
    *z = motion_catmull_rom(p0z, p1z, p2z, p3z, frac);
  }
  *level = level1 + (level2 - level1) * frac;
  *palette = pal1 + (pal2 - pal1) * frac;
  *z += (float)engine->z_offset_um;
}

static void motion_palette_color(const umh_motion_engine_t *engine, float palette_pos,
                                 uint8_t *r, uint8_t *g, uint8_t *b)
{
  uint8_t count = engine->palette_count;
  float pos;
  int32_t base;
  float frac;
  int32_t next;
  if (count == 0u) { *r = 255u; *g = 255u; *b = 255u; return; }
  if (count == 1u) { *r = engine->palette[0][0]; *g = engine->palette[0][1]; *b = engine->palette[0][2]; return; }
  pos = palette_pos;
  base = (int32_t)floorf(pos);
  frac = pos - (float)base;
  base %= (int32_t)count;
  if (base < 0) base += (int32_t)count;
  next = (base + 1) % (int32_t)count;
  *r = (uint8_t)((float)engine->palette[base][0] +
                 ((float)engine->palette[next][0] - (float)engine->palette[base][0]) * frac + 0.5f);
  *g = (uint8_t)((float)engine->palette[base][1] +
                 ((float)engine->palette[next][1] - (float)engine->palette[base][1]) * frac + 0.5f);
  *b = (uint8_t)((float)engine->palette[base][2] +
                 ((float)engine->palette[next][2] - (float)engine->palette[base][2]) * frac + 0.5f);
}

static int motion_emit_field(umh_motion_engine_t *engine, umh_spatial_renderer_t *renderer,
                             umh_output_frame_t *frame, float cx, float cy, float cz,
                             float base_level, float palette_pos)
{
  if (engine->vortex_program != UMH_VORTEX_OFF) {
    int emitted = motion_emit_vortex(engine, renderer, frame, cx, cy, cz, base_level);
    /* The alternating program counts its half periods in emitted frames, so
     * the counter advances with the field and not with the wall clock. */
    if (emitted == 0) engine->vortex_frames++;
    return emitted;
  }
  if (engine->trap_mode == UMH_MOTION_TRAP_DARK_VORTEX)
    return motion_emit_dark_vortex(engine, renderer, frame, cx, cy, cz, base_level);
  uint8_t count = engine->pattern_count != 0u ? engine->pattern_count : 1u;
  float per_source = base_level / sqrtf((float)count);
  uint8_t i;
  if (per_source > 255.0f) per_source = 255.0f;
  if (per_source < 0.0f) per_source = 0.0f;
  memset(frame, 0, sizeof(*frame));
  memset(engine->real_accum, 0, sizeof(engine->real_accum));
  memset(engine->imag_accum, 0, sizeof(engine->imag_accum));
  for (i = 0u; i < count; ++i) {
    umh_spatial_point_t point;
    point.x_um = (int32_t)lroundf(cx + engine->pattern_offsets[i][0]);
    point.y_um = (int32_t)lroundf(cy + engine->pattern_offsets[i][1]);
    point.z_um = (int32_t)lroundf(cz + engine->pattern_offsets[i][2]);
    point.level = (uint8_t)(per_source + 0.5f);
    point.phase = engine->pattern_phases[i];
    point.source_id = i;
    if (spatial_renderer_accumulate_point(renderer, &point,
                                          engine->real_accum,
                                          engine->imag_accum) != 0) return -1;
  }
  if (spatial_renderer_finalize(renderer, engine->real_accum,
                                engine->imag_accum, frame) != 0) return -2;
  if ((engine->flags & UMH_MOTION_FLAG_RGB) != 0u && engine->palette_count != 0u) {
    umh_rgb_value_t color[UMH_DEVICE_RGB_COUNT];
    uint8_t levels[UMH_DEVICE_RGB_COUNT];
    uint8_t r, g, b;
    motion_palette_color(engine, palette_pos, &r, &g, &b);
    for (i = 0u; i < UMH_DEVICE_RGB_COUNT; ++i) {
      color[i].red = r;
      color[i].green = g;
      color[i].blue = b;
      levels[i] = 255u;
    }
    spatial_renderer_merge_rgb(renderer, frame, color, levels, 0x0Fu);
  }
  return 0;
}

static void motion_track_axis(float desired, float *position, float *velocity,
                              float max_speed, float max_accel, float dt)
{
  float omega, accel;
  if (dt <= 0.0f || max_speed <= 0.0f || max_accel <= 0.0f) {
    *position = desired;
    *velocity = 0.0f;
    return;
  }
  omega = max_accel / max_speed;
  if (omega < 1.0f) omega = 1.0f;
  accel = omega * omega * (desired - *position) - 2.0f * omega * (*velocity);
  if (accel > max_accel) accel = max_accel;
  if (accel < -max_accel) accel = -max_accel;
  *velocity += accel * dt;
  if (*velocity > max_speed) *velocity = max_speed;
  if (*velocity < -max_speed) *velocity = -max_speed;
  *position += *velocity * dt;
}

static void motion_finish_stop(umh_motion_engine_t *engine, fpga_link_t *link)
{
  engine->state = UMH_MOTION_STATE_READY;
  engine->stop_requested = 0u;
  /* The run is over, so the vortex selection goes with it.  Leaving it set
   * would keep the engine producing the vortex field on the next start and
   * would ignore a live target the host programs in the meantime. */
  engine->vortex_program = UMH_VORTEX_OFF;
  engine->fade_scale = 0.0f;
  engine->fade_step = 0.0f;
  engine->last_service_us = 0u;
  engine->next_due_us = 0u;
  engine->vel_x_um_s = 0.0f;
  engine->vel_y_um_s = 0.0f;
  engine->vel_z_um_s = 0.0f;
  engine->current_valid = 1u;
  if (link != NULL) (void)fpga_link_safe_stop(link);
}

void motion_engine_init(umh_motion_engine_t *engine)
{
  osMutexAttr_t attributes;
  if (engine == NULL) return;
  memset(engine, 0, sizeof(*engine));
  engine->state = UMH_MOTION_STATE_OFF;
  engine->mode = UMH_MOTION_MODE_PATH;
  engine->flags = UMH_MOTION_FLAG_LOOP;
  engine->output_rate_hz = 500u;
  engine->period_us = 2000u;
  engine->loop_us = 2000000u;
  engine->max_speed_um_s = 500000u;      /* 500 mm/s */
  engine->max_accel_um_s2 = 20000000u;   /* 20,000 mm/s^2 */
  engine->level = 255u;
  engine->trap_mode = 0u;
  engine->trap_radius_um = 2000;
  engine->trap_phase_span = 0u;
  engine->ulm_wave_speed_mm_s = 5000u;
  engine->ulm_tangent_x = 1.0f;
  engine->pos_x_um = 0.0f;
  engine->pos_y_um = 0.0f;
  engine->pos_z_um = 100000.0f;
  engine->target_x_um = 0.0f;
  engine->target_y_um = 0.0f;
  engine->target_z_um = 100000.0f;
  engine->target_level = 255u;
  engine->target_palette = 0u;
  engine->current_valid = 0u;
  engine->phase = 0.0f;
  motion_rebuild_pattern(engine);
  memset(&attributes, 0, sizeof(attributes));
  attributes.name = "umh-motion";
  attributes.cb_mem = &engine->lock_memory;
  attributes.cb_size = sizeof(engine->lock_memory);
  engine->lock = osMutexNew(&attributes);
}

int motion_engine_upload(umh_motion_engine_t *engine, const uint8_t *payload,
                         uint16_t length)
{
  uint16_t count;
  if (engine == NULL || payload == NULL) return -1;
  if (length < sizeof(umh_motion_upload_wire_t)) return -2;
  if (payload[0] != 1u) return -3;
  count = motion_read_u16(&payload[2]);
  if (count > UMH_MOTION_MAX_POINTS) return -4;
  if ((uint32_t)count * sizeof(umh_motion_point_wire_t) +
      sizeof(umh_motion_upload_wire_t) != (uint32_t)length) return -5;
  if (motion_lock(engine) != 0) return -6;
  memcpy(engine->points, &payload[sizeof(umh_motion_upload_wire_t)],
         (size_t)count * sizeof(umh_motion_point_wire_t));
  engine->point_count = count;
  engine->phase = 0.0f;
  engine->configured = 1u;
  if (engine->state == UMH_MOTION_STATE_OFF) engine->state = UMH_MOTION_STATE_READY;
  if (engine->current_valid == 0u && count != 0u) {
    float point_level;
    float point_palette;
    motion_point_values(engine, 0u,
                        &engine->pos_x_um, &engine->pos_y_um, &engine->pos_z_um,
                        &point_level, &point_palette);
    engine->last_level = (uint8_t)(point_level < 0.0f ? 0.0f :
                                   (point_level > 255.0f ? 255.0f : point_level));
    engine->last_palette = (uint8_t)point_palette;
    engine->current_valid = 1u;
  }
  motion_unlock(engine);
  return 0;
}

int motion_engine_configure(umh_motion_engine_t *engine, const uint8_t *payload,
                            uint16_t length)
{
  const umh_motion_config_wire_v1_t *wire;
  const umh_motion_config_wire_t *ext;
  uint8_t i;
  if (engine == NULL || payload == NULL) return -1;
  if (length != sizeof(umh_motion_config_wire_v1_t) &&
      length != sizeof(umh_motion_config_wire_t)) return -2;
  wire = (const umh_motion_config_wire_v1_t *)payload;
  ext = length == sizeof(umh_motion_config_wire_t) ? (const umh_motion_config_wire_t *)payload : NULL;
  if (wire->mode > (uint8_t)UMH_MOTION_MODE_LIVE) return -3;
  if (wire->trap_mode > UMH_MOTION_TRAP_DARK_VORTEX || wire->palette_count > UMH_MOTION_PALETTE_SIZE) return -4;
  if (wire->output_rate_hz < UMH_MOTION_MIN_RATE_HZ ||
      wire->output_rate_hz > UMH_MOTION_MAX_RATE_HZ) return -5;
  if (wire->loop_ms < 1u || wire->loop_ms > 60000u) return -6;
  if (wire->max_speed_mm_s > 10000u || wire->max_accel_mm_s2 > 500000u) return -7;
  if (wire->z_offset_10um > UMH_MOTION_COORD_10UM_MAX ||
      wire->z_offset_10um < -UMH_MOTION_COORD_10UM_MAX) return -8;
  if (ext != NULL) {
    if (ext->ulm_frequency_hz > 1000u) return -10;
    if (ext->ulm_wave_speed_mm_s > 10000u) return -11;
    if (ext->ulm_axis > 3u) return -12;
  }
  if (motion_lock(engine) != 0) return -9;
  engine->mode = wire->mode;
  engine->flags = wire->flags;
  engine->output_rate_hz = wire->output_rate_hz;
  engine->period_us = 1000000u / (uint32_t)wire->output_rate_hz;
  if (engine->period_us == 0u) engine->period_us = 1u;
  engine->loop_us = (uint32_t)wire->loop_ms * 1000u;
  if (engine->loop_us == 0u) engine->loop_us = 1u;
  engine->max_speed_um_s = (uint32_t)wire->max_speed_mm_s * 1000u;
  engine->max_accel_um_s2 = (uint32_t)wire->max_accel_mm_s2 * 1000u;
  engine->z_offset_um = (int32_t)wire->z_offset_10um * 10;
  engine->level = wire->level;
  engine->trap_mode = wire->trap_mode;
  engine->trap_radius_um = (int32_t)wire->trap_radius_10um * 10;
  if (engine->trap_radius_um > 50000) engine->trap_radius_um = 50000;
  if (engine->trap_radius_um < 0) engine->trap_radius_um = 0;
  engine->trap_phase_span = wire->trap_phase_span;
  engine->palette_count = wire->palette_count;
  engine->palette_spin_x10 = wire->palette_spin_x10 > 250u ? 250u : wire->palette_spin_x10;
  engine->path_spin_mrad_s = wire->path_spin_mrad_s;
  if (ext != NULL) {
    engine->ulm_frequency_hz = ext->ulm_frequency_hz;
    engine->ulm_wave_speed_mm_s = ext->ulm_wave_speed_mm_s != 0u ? ext->ulm_wave_speed_mm_s : 5000u;
    engine->ulm_axis = ext->ulm_axis;
    if (ext->ulm_amplitude_10um != 0u) {
      engine->ulm_amplitude_um = (int32_t)ext->ulm_amplitude_10um * 10;
    } else if (engine->ulm_frequency_hz != 0u) {
      engine->ulm_amplitude_um = (int32_t)((float)engine->ulm_wave_speed_mm_s * 1000.0f / (2.0f * 3.14159265f * (float)engine->ulm_frequency_hz));
    }
    if (engine->ulm_amplitude_um > 50000) engine->ulm_amplitude_um = 50000;
    if (engine->ulm_amplitude_um < 0) engine->ulm_amplitude_um = 0;
  } else {
    engine->ulm_frequency_hz = 0u;
    engine->ulm_wave_speed_mm_s = 5000u;
    engine->ulm_amplitude_um = 0;
    engine->ulm_axis = 0u;
  }
  for (i = 0u; i < UMH_MOTION_PALETTE_SIZE; ++i) {
    engine->palette[i][0] = wire->palette[i][0];
    engine->palette[i][1] = wire->palette[i][1];
    engine->palette[i][2] = wire->palette[i][2];
  }
  engine->configured = 1u;
  engine->last_service_us = 0u;
  engine->next_due_us = 0u;
  /* A host config always takes the engine back to ordinary motion: leaving a
   * vortex program selected here would make the next frame ignore the path or
   * live target the host just programmed. */
  engine->vortex_program = UMH_VORTEX_OFF;
  motion_rebuild_pattern(engine);
  if (engine->state == UMH_MOTION_STATE_OFF) engine->state = UMH_MOTION_STATE_READY;
  motion_unlock(engine);
  return 0;
}

int motion_engine_target(umh_motion_engine_t *engine, const uint8_t *payload,
                         uint16_t length)
{
  const umh_motion_point_wire_t *wire;
  if (engine == NULL || payload == NULL) return -1;
  if (length != sizeof(umh_motion_point_wire_t)) return -2;
  wire = (const umh_motion_point_wire_t *)payload;
  if (motion_lock(engine) != 0) return -3;
  engine->target_x_um = (float)wire->x_10um * 10.0f;
  engine->target_y_um = (float)wire->y_10um * 10.0f;
  engine->target_z_um = (float)wire->z_10um * 10.0f;
  engine->target_level = wire->level;
  engine->target_palette = wire->palette;
  engine->target_valid = 1u;
  engine->configured = 1u;
  if (engine->current_valid == 0u) {
    engine->pos_x_um = engine->target_x_um;
    engine->pos_y_um = engine->target_y_um;
    engine->pos_z_um = engine->target_z_um;
    engine->current_valid = 1u;
  }
  if (engine->state == UMH_MOTION_STATE_OFF) engine->state = UMH_MOTION_STATE_READY;
  motion_unlock(engine);
  return 0;
}

int motion_engine_start(umh_motion_engine_t *engine)
{
  if (engine == NULL) return -1;
  if (motion_lock(engine) != 0) return -2;
  if (engine->configured == 0u) { motion_unlock(engine); return -3; }
  if (engine->mode == UMH_MOTION_MODE_PATH && engine->point_count == 0u) {
    motion_unlock(engine);
    return -4;
  }
  if (engine->mode == UMH_MOTION_MODE_LIVE && engine->target_valid == 0u) {
    if (engine->point_count != 0u) {
      float unused;
      motion_point_values(engine, 0u, &engine->target_x_um, &engine->target_y_um,
                          &engine->target_z_um, &unused, &unused);
    } else {
      engine->target_x_um = 0.0f;
      engine->target_y_um = 0.0f;
      engine->target_z_um = 100000.0f;
    }
    engine->target_level = engine->level;
    engine->target_palette = 0u;
    engine->target_valid = 1u;
  }
  if (engine->current_valid == 0u) {
    if (engine->mode == UMH_MOTION_MODE_PATH && engine->point_count != 0u) {
      float unused;
      motion_point_values(engine, 0u, &engine->pos_x_um, &engine->pos_y_um,
                          &engine->pos_z_um, &unused, &unused);
    } else {
      engine->pos_x_um = engine->target_x_um;
      engine->pos_y_um = engine->target_y_um;
      engine->pos_z_um = engine->target_z_um;
    }
    engine->current_valid = 1u;
  }
  engine->phase = 0.0f;
  engine->ulm_phase = 0.0f;
  engine->vel_x_um_s = 0.0f;
  engine->vel_y_um_s = 0.0f;
  engine->vel_z_um_s = 0.0f;
  engine->fade_scale = 1.0f;
  engine->fade_step = 0.0f;
  engine->last_level_scale = 0.0f;
  engine->stop_requested = 0u;
  engine->last_service_us = 0u;
  engine->next_due_us = 0u;
  engine->frames_submitted = 0u;
  engine->missed_deadlines = 0u;
  engine->frame_errors = 0u;
  engine->fps_frames = 0u;
  engine->fps_window_start_us = 0u;
  engine->service_max_cycles = 0u;
  engine->service_sum_cycles = 0u;
  engine->service_count = 0u;
  engine->service_last_cycles = 0u;
  engine->render_max_cycles = 0u;
  engine->render_sum_cycles = 0u;
  engine->submit_max_cycles = 0u;
  engine->submit_sum_cycles = 0u;
  engine->spin_angle = 0.0f;
  engine->palette_spin_phase = 0.0f;
  /* A run always opens on the +l half period. */
  engine->vortex_frames = 0u;
  engine->state = UMH_MOTION_STATE_RUNNING;
  motion_unlock(engine);
  return 0;
}

void motion_engine_request_stop(umh_motion_engine_t *engine)
{
  float ticks;
  if (engine == NULL) return;
  if (motion_lock(engine) != 0) return;
  if (engine->state == UMH_MOTION_STATE_RUNNING) {
    engine->state = UMH_MOTION_STATE_STOPPING;
    engine->stop_requested = 1u;
    engine->fade_scale = engine->last_level_scale > 0.01f ? engine->last_level_scale : 1.0f;
    ticks = (float)engine->output_rate_hz * (float)UMH_MOTION_STOP_FADE_MS / 1000.0f;
    if (ticks < 1.0f) ticks = 1.0f;
    engine->fade_step = engine->fade_scale / ticks;
    if (engine->fade_step <= 0.0f) engine->fade_step = 1.0f;
    engine->last_service_us = 0u;
    engine->next_due_us = 0u;
  }
  motion_unlock(engine);
}

void motion_engine_abort(umh_motion_engine_t *engine, fpga_link_t *link)
{
  if (engine == NULL) return;
  if (motion_lock(engine) != 0) return;
  engine->state = engine->configured != 0u ? UMH_MOTION_STATE_READY : UMH_MOTION_STATE_OFF;
  engine->stop_requested = 0u;
  engine->fade_scale = 0.0f;
  engine->fade_step = 0.0f;
  engine->vortex_program = UMH_VORTEX_OFF;
  engine->last_service_us = 0u;
  engine->next_due_us = 0u;
  engine->vel_x_um_s = 0.0f;
  engine->vel_y_um_s = 0.0f;
  engine->vel_z_um_s = 0.0f;
  engine->current_valid = 1u;
  motion_unlock(engine);
  if (link != NULL) (void)fpga_link_safe_stop(link);
}

uint8_t motion_engine_owns_output(const umh_motion_engine_t *engine)
{
  if (engine == NULL) return 0u;
  return (engine->state == UMH_MOTION_STATE_RUNNING ||
          engine->state == UMH_MOTION_STATE_STOPPING) ? 1u : 0u;
}

uint8_t motion_engine_uses_rgb(const umh_motion_engine_t *engine)
{
  if (engine == NULL) return 0u;
  return ((engine->flags & UMH_MOTION_FLAG_RGB) != 0u &&
          engine->palette_count != 0u) ? 1u : 0u;
}

uint8_t motion_engine_is_active(const umh_motion_engine_t *engine)
{
  return motion_engine_owns_output(engine);
}

uint8_t motion_engine_trap_mode(const umh_motion_engine_t *engine)
{
  if (engine == NULL) return 0u;
  return engine->trap_mode;
}

uint8_t motion_engine_vortex_program(const umh_motion_engine_t *engine)
{
  if (engine == NULL) return UMH_VORTEX_OFF;
  return engine->vortex_program;
}

uint32_t motion_engine_service(umh_motion_engine_t *engine,
                               umh_spatial_renderer_t *renderer,
                               fpga_link_t *link, uint64_t now_us,
                               uint32_t *wait_us)
{
  umh_output_frame_t frame;
  float dt = 0.0f;
  float desired_x, desired_y, desired_z;
  float desired_level, desired_palette;
  float level_scale;
  int result;
  if (wait_us == NULL) return 0u;
  *wait_us = 1000u;
  if (engine == NULL || renderer == NULL || link == NULL) return 0u;
  if (motion_lock(engine) != 0) return 0u;
  if (engine->state != UMH_MOTION_STATE_RUNNING &&
      engine->state != UMH_MOTION_STATE_STOPPING) {
    motion_unlock(engine);
    return 0u;
  }
  if (now_us < engine->next_due_us) {
    uint64_t delta = engine->next_due_us - now_us;
    *wait_us = delta > 100000u ? 100000u : (uint32_t)delta;
    motion_unlock(engine);
    return 0u;
  }
  if (engine->last_service_us != 0u && now_us > engine->last_service_us) {
    dt = (float)(now_us - engine->last_service_us) / 1000000.0f;
    if (dt > 0.05f) dt = 0.05f;
  }
  engine->last_service_us = now_us;
  if (dt > 0.0f && engine->ulm_frequency_hz != 0u) {
    engine->ulm_phase += (float)engine->ulm_frequency_hz * dt;
    while (engine->ulm_phase >= 1.0f) engine->ulm_phase -= 1.0f;
  }

  desired_x = engine->pos_x_um;
  desired_y = engine->pos_y_um;
  desired_z = engine->pos_z_um;
  desired_level = (float)engine->last_level;
  desired_palette = (float)engine->last_palette;

  if (engine->state == UMH_MOTION_STATE_STOPPING) {
    /* hold the last field while the amplitude fades out */
    engine->fade_scale -= engine->fade_step;
    if (engine->fade_scale < 0.0f) engine->fade_scale = 0.0f;
  } else if ((engine->flags & UMH_MOTION_FLAG_PAUSED) != 0u) {
    /* hold the last field */
  } else if (engine->mode == UMH_MOTION_MODE_PATH) {
    if (engine->loop_us != 0u && dt > 0.0f) {
      engine->phase += dt * 1000000.0f / (float)engine->loop_us;
      if ((engine->flags & UMH_MOTION_FLAG_LOOP) != 0u) {
        while (engine->phase >= 1.0f) engine->phase -= 1.0f;
      } else if (engine->phase > 1.0f) {
        engine->phase = 1.0f;
      }
    }
    motion_sample_path(engine, engine->phase,
                       &desired_x, &desired_y, &desired_z,
                       &desired_level, &desired_palette);
  } else { /* LIVE target */
    if (engine->target_valid != 0u) {
      desired_x = engine->target_x_um;
      desired_y = engine->target_y_um;
      desired_z = engine->target_z_um;
      desired_level = (float)engine->target_level;
      desired_palette = (float)engine->target_palette;
    }
  }

  motion_clamp_position(&desired_x, &desired_y, &desired_z);
  if ((engine->flags & UMH_MOTION_FLAG_DIRECT) != 0u) {
    engine->pos_x_um = desired_x;
    engine->pos_y_um = desired_y;
    engine->pos_z_um = desired_z;
    engine->vel_x_um_s = 0.0f;
    engine->vel_y_um_s = 0.0f;
    engine->vel_z_um_s = 0.0f;
  } else if (dt <= 0.0f) {
    /* First generated frame: hold the physical position the ball is already
     * in.  The tracker starts moving on the next frame, so entering path or
     * live mode never commands an instantaneous focus jump. */
    engine->vel_x_um_s = 0.0f;
    engine->vel_y_um_s = 0.0f;
    engine->vel_z_um_s = 0.0f;
  } else {
    motion_track_axis(desired_x, &engine->pos_x_um, &engine->vel_x_um_s,
                      (float)engine->max_speed_um_s,
                      (float)engine->max_accel_um_s2, dt);
    motion_track_axis(desired_y, &engine->pos_y_um, &engine->vel_y_um_s,
                      (float)engine->max_speed_um_s,
                      (float)engine->max_accel_um_s2, dt);
    motion_track_axis(desired_z, &engine->pos_z_um, &engine->vel_z_um_s,
                      (float)engine->max_speed_um_s,
                      (float)engine->max_accel_um_s2, dt);
  }
  motion_clamp_position(&engine->pos_x_um, &engine->pos_y_um, &engine->pos_z_um);

  /* Time-varying colour gradient and turntable rotation.  Both are pure
   * motion-engine state updates: no host traffic, no extra FPGA commands. */
  if (dt > 0.0f && engine->palette_spin_x10 != 0u) {
    engine->palette_spin_phase += (float)engine->palette_spin_x10 * 0.1f * dt;
    while (engine->palette_spin_phase >= 1024.0f)
      engine->palette_spin_phase -= 1024.0f;
  }
  if (dt > 0.0f && engine->path_spin_mrad_s != 0) {
    engine->spin_angle += (float)engine->path_spin_mrad_s * 0.001f * dt;
    while (engine->spin_angle >= UMH_TWO_PI) engine->spin_angle -= UMH_TWO_PI;
    while (engine->spin_angle < 0.0f) engine->spin_angle += UMH_TWO_PI;
  }

  engine->last_level = (uint8_t)(desired_level > 255.0f ? 255.0f :
                                 (desired_level < 0.0f ? 0.0f : desired_level));
  engine->last_palette = (uint8_t)(desired_palette < 0.0f ? 0.0f :
                                   (desired_palette > 255.0f ? 255.0f : desired_palette));

  level_scale = (float)engine->last_level * ((float)engine->level / 255.0f);
  if (engine->state == UMH_MOTION_STATE_STOPPING) level_scale *= engine->fade_scale;
  if (level_scale > 255.0f) level_scale = 255.0f;
  if (level_scale < 0.0f) level_scale = 0.0f;
  engine->last_level_scale = engine->fade_scale;

  if (link->status.fifo_credit == 0u) {
    (void)fpga_link_poll_status(link);
    if (link->status.fifo_credit == 0u) {
      *wait_us = 250u;
      motion_unlock(engine);
      return 0u;
    }
  }

  uint32_t service_start_cycles = DWT->CYCCNT;
  uint32_t render_start_cycles = DWT->CYCCNT;
  {
    float emit_x = engine->pos_x_um;
    float emit_y = engine->pos_y_um;
    if (engine->path_spin_mrad_s != 0) {
      int32_t angle_q10 = (int32_t)(engine->spin_angle * (1024.0f / UMH_TWO_PI));
      float spin_cos = umh_fast_cos_q10(angle_q10);
      float spin_sin = umh_fast_sin_q10(angle_q10);
      emit_x = engine->pos_x_um * spin_cos - engine->pos_y_um * spin_sin;
      emit_y = engine->pos_x_um * spin_sin + engine->pos_y_um * spin_cos;
    }
    if ((engine->flags & UMH_MOTION_FLAG_ULM) != 0u &&
        engine->ulm_frequency_hz != 0u && engine->ulm_amplitude_um > 0) {
      float ulm_off = sinf(UMH_TWO_PI * engine->ulm_phase) * (float)engine->ulm_amplitude_um;
      float ulm_off_q = cosf(UMH_TWO_PI * engine->ulm_phase) * (float)engine->ulm_amplitude_um;
      if (engine->ulm_axis == 1u) emit_x += ulm_off;
      else if (engine->ulm_axis == 2u) emit_y += ulm_off;
      else { emit_x += ulm_off; emit_y += ulm_off_q; }
    }
    if (motion_emit_field(engine, renderer, &frame,
                          emit_x, emit_y, engine->pos_z_um,
                          level_scale,
                          (float)engine->last_palette + engine->palette_spin_phase) != 0) {
      motion_record_service(engine, service_start_cycles);
      engine->frame_errors++;
      engine->next_due_us = now_us + 1000u;
      *wait_us = 1000u;
      motion_record_render(engine, DWT->CYCCNT - render_start_cycles);
      motion_unlock(engine);
      return 0u;
    }
  }
  motion_record_render(engine, DWT->CYCCNT - render_start_cycles);
  frame.sequence = engine->frame_sequence++;
  frame.deadline = now_us;
  {
    uint32_t submit_start_cycles = DWT->CYCCNT;
    result = fpga_link_submit_allow_hold(link, &frame, 1u);
    motion_record_submit(engine, DWT->CYCCNT - submit_start_cycles);
  }
  if (result != 0) {
    motion_record_service(engine, service_start_cycles);
    engine->frame_errors++;
    engine->next_due_us = now_us + 1000u;
    *wait_us = 1000u;
    motion_unlock(engine);
    return 0u;
  }

  engine->frames_submitted++;
  engine->fps_frames++;
  motion_record_service(engine, service_start_cycles);
  if (engine->fps_window_start_us == 0u) engine->fps_window_start_us = now_us;
  if (now_us - engine->fps_window_start_us >= 1000000u) {
    uint64_t elapsed = now_us - engine->fps_window_start_us;
    engine->fps = (uint32_t)(((uint64_t)engine->fps_frames * 100000000u) / elapsed);
    engine->fps_frames = 0u;
    engine->fps_window_start_us = now_us;
  }

  if (engine->state == UMH_MOTION_STATE_STOPPING && engine->fade_scale <= 0.0f) {
    motion_finish_stop(engine, link);
    *wait_us = 1000u;
    motion_unlock(engine);
    return 0u;
  }

  if (engine->next_due_us == 0u) {
    /* First frame of a run: anchor the absolute deadline to now.  Without
     * this the initial += period produced a deadline of "period microseconds
     * after boot", which the service loop then counted as millions of missed
     * frames before re-synchronising. */
    engine->next_due_us = now_us + (uint64_t)engine->period_us;
  } else {
    engine->next_due_us += (uint64_t)engine->period_us;
    if (engine->next_due_us <= now_us) {
      uint64_t behind = now_us - engine->next_due_us;
      uint32_t missed = (uint32_t)(behind / (uint64_t)engine->period_us) + 1u;
      engine->missed_deadlines += missed;
      engine->next_due_us = now_us + (uint64_t)engine->period_us;
    }
  }
  {
    uint64_t wait = engine->next_due_us > now_us ? engine->next_due_us - now_us : 250u;
    *wait_us = wait > 100000u ? 100000u : (uint32_t)wait;
    if (*wait_us == 0u) *wait_us = 250u;
  }
  motion_unlock(engine);
  return 0u;
}

void motion_engine_get_status(const umh_motion_engine_t *engine,
                              umh_motion_status_wire_t *status)
{
  if (status == NULL) return;
  memset(status, 0, sizeof(*status));
  if (engine == NULL) return;
  if (motion_lock((umh_motion_engine_t *)engine) != 0) return;
  status->state = engine->state;
  status->flags = engine->flags;
  status->output_rate_hz = engine->output_rate_hz;
  status->path_points = engine->point_count;
  status->loop_ms = (uint16_t)(engine->loop_us / 1000u);
  status->x_um = (int32_t)lroundf(engine->pos_x_um);
  status->y_um = (int32_t)lroundf(engine->pos_y_um);
  status->z_um = (int32_t)lroundf(engine->pos_z_um);
  status->level = engine->last_level;
  status->trap_mode = engine->trap_mode;
  {
    uint32_t cycles_per_us = SystemCoreClock / 1000000u;
    uint32_t max_us;
    uint32_t avg_us = 0u;
    if (cycles_per_us == 0u) cycles_per_us = 1u;
    max_us = engine->service_max_cycles / cycles_per_us;
    status->service_max_us = max_us > 65535u ? 65535u : (uint16_t)max_us;
    if (engine->service_count != 0u) {
      avg_us = (engine->service_sum_cycles / engine->service_count) / cycles_per_us;
      if (avg_us > 65535u) avg_us = 65535u;
    }
    status->service_avg_us = (uint16_t)avg_us;
    max_us = engine->render_max_cycles / cycles_per_us;
    status->render_max_us = max_us > 65535u ? 65535u : (uint16_t)max_us;
    status->render_avg_us = engine->service_count != 0u ?
        (uint16_t)((engine->render_sum_cycles / engine->service_count) / cycles_per_us) : 0u;
    max_us = engine->submit_max_cycles / cycles_per_us;
    status->submit_max_us = max_us > 65535u ? 65535u : (uint16_t)max_us;
    status->submit_avg_us = engine->service_count != 0u ?
        (uint16_t)((engine->submit_sum_cycles / engine->service_count) / cycles_per_us) : 0u;
  }
  status->frames = engine->frames_submitted;
  status->missed_deadlines = engine->missed_deadlines;
  status->frame_errors = engine->frame_errors;
  status->fps_x100 = engine->fps;
  motion_unlock((umh_motion_engine_t *)engine);
}


/* 100 mm geometric focus places the dark vortex core near 80 mm. */
#define UMH_LEVITATION_FOCUS_Z_UM 100000
#define UMH_LEVITATION_SPIRAL_TURNS 1.0f

static int motion_emit_dark_vortex(umh_motion_engine_t *engine,
                                   umh_spatial_renderer_t *renderer,
                                   umh_output_frame_t *frame,
                                   float cx, float cy, float cz,
                                   float base_level)
{
  const float two_pi = 6.28318530717958647692f;
  uint16_t i;
  float wavelength_um;
  float phase_scale;
  float magnitude;
  float focus_z_um;
  (void)cz;
  if (engine == NULL || renderer == NULL || renderer->profile == NULL || frame == NULL) return -1;
  if ((renderer->profile->capability_flags & UMH_PROFILE_CAP_GEOMETRY_VALID) == 0u) return -2;
  if (renderer->carrier_hz == 0u || renderer->sound_speed_um_per_s == 0u) return -3;
  if (base_level < 0.0f) base_level = 0.0f;
  if (base_level > 255.0f) base_level = 255.0f;
  wavelength_um = (float)renderer->sound_speed_um_per_s / (float)renderer->carrier_hz;
  phase_scale = two_pi / wavelength_um;
  focus_z_um = (float)UMH_LEVITATION_FOCUS_Z_UM;
  magnitude = base_level * (1.0f / 255.0f);
  memset(frame, 0, sizeof(*frame));
  memset(engine->real_accum, 0, sizeof(engine->real_accum));
  memset(engine->imag_accum, 0, sizeof(engine->imag_accum));
  for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) {
    const umh_element_coordinate_t *c = &renderer->profile->coordinates[i];
    float ex = (float)c->x_um - cx;
    float ey = (float)c->y_um - cy;
    float ez = (float)c->z_um - focus_z_um;
    float distance = sqrtf(ex * ex + ey * ey + ez * ez);
    float theta = UMH_LEVITATION_SPIRAL_TURNS * atan2f(ey, ex);
    float phase_q10 = (theta - phase_scale * distance) * (1024.0f / two_pi) +
                      (float)renderer->phase_offset_q10[i];
    float mag = magnitude * renderer->gain_scale[i];
    int32_t q10 = (int32_t)lroundf(phase_q10);
    engine->real_accum[i] = mag * umh_fast_cos_q10(q10);
    engine->imag_accum[i] = mag * umh_fast_sin_q10(q10);
  }
  return spatial_renderer_finalize(renderer, engine->real_accum,
                                   engine->imag_accum, frame);
}

/* Acoustic vortex: the same complex accumulation as the dark vortex, but the
 * spiral is wound around the foam-plane focus instead of the trap, and the
 * sign of the winding is the whole program.
 *
 * The field is otherwise constant -- same focus, same amplitude, same level
 * every frame -- so STEADY produces a frame that is bit-identical on every
 * iteration and fpga_link_submit_allow_hold() drops all but the first SPI
 * write.  ALT changes the sign every UMH_VORTEX_ALT_HALF_FRAMES frames, so it
 * costs one write per half period (the first frame of each half) and nothing
 * for the frames inside it.  The level is full on every channel, i.e. the
 * maximum output power this hardware has. */
static int motion_emit_vortex(umh_motion_engine_t *engine,
                              umh_spatial_renderer_t *renderer,
                              umh_output_frame_t *frame,
                              float cx, float cy, float cz,
                              float base_level)
{
  const float two_pi = 6.28318530717958647692f;
  uint16_t i;
  float wavelength_um;
  float phase_scale;
  float magnitude;
  float turns;
  if (engine == NULL || renderer == NULL || renderer->profile == NULL || frame == NULL) return -1;
  if ((renderer->profile->capability_flags & UMH_PROFILE_CAP_GEOMETRY_VALID) == 0u) return -2;
  if (renderer->carrier_hz == 0u || renderer->sound_speed_um_per_s == 0u) return -3;
  if (base_level < 0.0f) base_level = 0.0f;
  if (base_level > 255.0f) base_level = 255.0f;
  wavelength_um = (float)renderer->sound_speed_um_per_s / (float)renderer->carrier_hz;
  phase_scale = two_pi / wavelength_um;
  turns = (float)UMH_VORTEX_TURNS;
  /* Time reversal: every other half period the spiral is wound the other way,
   * so the ring reverses its orbital motion and shears the film in the
   * opposite direction. */
  if (engine->vortex_program == UMH_VORTEX_ALT &&
      ((engine->vortex_frames / UMH_VORTEX_ALT_HALF_FRAMES) & 1u) != 0u)
    turns = -turns;
  /* Full amplitude everywhere: a spatial magnitude of 1 becomes wire level
   * 128 in spatial_renderer_finalize(), which is this hardware's full power. */
  magnitude = base_level * (1.0f / 255.0f);
  memset(frame, 0, sizeof(*frame));
  memset(engine->real_accum, 0, sizeof(engine->real_accum));
  memset(engine->imag_accum, 0, sizeof(engine->imag_accum));
  for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) {
    const umh_element_coordinate_t *c = &renderer->profile->coordinates[i];
    float ex = (float)c->x_um - cx;
    float ey = (float)c->y_um - cy;
    float ez = (float)c->z_um - cz;
    float distance = sqrtf(ex * ex + ey * ey + ez * ez);
    /* l turns of phase per turn of azimuth, plus the ordinary focusing delay,
     * which is what pulls the spiral wavefront down onto the foam.  The
     * azimuth is taken about the commanded axis, so the whole field follows
     * the engine position the way every other program does. */
    float phase_q10 = (turns * atan2f(ey, ex) - phase_scale * distance) *
                      (1024.0f / two_pi) +
                      (float)renderer->phase_offset_q10[i];
    float mag = magnitude * renderer->gain_scale[i];
    int32_t q10 = (int32_t)lroundf(phase_q10);
    engine->real_accum[i] = mag * umh_fast_cos_q10(q10);
    engine->imag_accum[i] = mag * umh_fast_sin_q10(q10);
  }
  return spatial_renderer_finalize(renderer, engine->real_accum,
                                   engine->imag_accum, frame);
}

int motion_engine_configure_levitation(umh_motion_engine_t *engine,
                                       uint8_t level, int32_t trap_z_um)
{
  if (engine == NULL) return -1;
  if (trap_z_um < (int32_t)UMH_SAFE_Z_MIN_UM || trap_z_um > (int32_t)UMH_SAFE_Z_MAX_UM) return -2;
  if (motion_lock(engine) != 0) return -3;
  engine->mode = UMH_MOTION_MODE_LIVE;
  engine->flags = 0u;
  engine->output_rate_hz = 50u;
  engine->period_us = 20000u;
  engine->level = 255u;
  engine->trap_mode = UMH_MOTION_TRAP_DARK_VORTEX;
  engine->trap_radius_um = 0;
  engine->trap_phase_span = 0u;
  engine->z_offset_um = 0;
  engine->palette_count = 0u;
  engine->palette_spin_x10 = 0u;
  engine->path_spin_mrad_s = 0;
  engine->ulm_frequency_hz = 0u;
  engine->ulm_amplitude_um = 0;
  engine->ulm_axis = 0u;
  engine->target_valid = 1u;
  engine->target_x_um = 0.0f;
  engine->target_y_um = 0.0f;
  engine->target_z_um = (float)trap_z_um;
  engine->target_level = level;
  engine->target_palette = 0u;
  engine->pos_x_um = 0.0f;
  engine->pos_y_um = 0.0f;
  engine->pos_z_um = (float)trap_z_um;
  engine->vel_x_um_s = 0.0f;
  engine->vel_y_um_s = 0.0f;
  engine->vel_z_um_s = 0.0f;
  engine->current_valid = 1u;
  engine->last_level = level;
  engine->last_level_scale = 0.0f;
  engine->last_palette = 0u;
  engine->configured = 1u;
  engine->stop_requested = 0u;
  engine->last_service_us = 0u;
  engine->next_due_us = 0u;
  /* Levitation and the vortex programs are different users of the same engine
   * and must never be selected at once. */
  engine->vortex_program = UMH_VORTEX_OFF;
  motion_rebuild_pattern(engine);
  engine->state = UMH_MOTION_STATE_READY;
  motion_unlock(engine);
  return 0;
}

int motion_engine_configure_vortex(umh_motion_engine_t *engine, uint8_t program)
{
  uint32_t period;
  float z_um;
  if (engine == NULL) return -1;
  if (program < UMH_VORTEX_STEADY || program > UMH_VORTEX_ALT) return -2;
  if (motion_lock(engine) != 0) return -3;

  /* Both programs radiate a static focus at full amplitude; only the winding
   * sense of the phase differs, and ALT reverses that twice per period.  The
   * frame cadence therefore only has to place those edges and feed the link
   * watchdog, and every other frame is suppressed by the hold optimisation. */
  period = 1000000u / (uint32_t)UMH_VORTEX_RATE_HZ;
  if (period == 0u) period = 1u;
  z_um = (float)UMH_VORTEX_FOCUS_Z_UM;

  /* A plain LIVE program: the target never moves, so the ordinary tracker
   * holds the focus and the emitter owns the whole phase field. */
  engine->mode = UMH_MOTION_MODE_LIVE;
  engine->flags = 0u;
  engine->output_rate_hz = UMH_VORTEX_RATE_HZ;
  engine->period_us = period;
  engine->level = UMH_VORTEX_LEVEL;
  engine->trap_mode = 0u;
  engine->trap_radius_um = 0;
  engine->trap_phase_span = 0u;
  engine->z_offset_um = 0;
  engine->palette_count = 0u;
  engine->palette_spin_x10 = 0u;
  engine->path_spin_mrad_s = 0;
  engine->ulm_frequency_hz = 0u;
  engine->ulm_amplitude_um = 0;
  engine->ulm_axis = 0u;
  engine->point_count = 0u;
  engine->target_valid = 1u;
  engine->target_x_um = 0.0f;
  engine->target_y_um = 0.0f;
  engine->target_z_um = z_um;
  engine->target_level = UMH_VORTEX_LEVEL;
  engine->target_palette = 0u;
  engine->pos_x_um = 0.0f;
  engine->pos_y_um = 0.0f;
  engine->pos_z_um = z_um;
  engine->vel_x_um_s = 0.0f;
  engine->vel_y_um_s = 0.0f;
  engine->vel_z_um_s = 0.0f;
  engine->current_valid = 1u;
  engine->last_level = UMH_VORTEX_LEVEL;
  engine->last_level_scale = 0.0f;
  engine->last_palette = 0u;
  engine->configured = 1u;
  engine->stop_requested = 0u;
  engine->last_service_us = 0u;
  engine->next_due_us = 0u;
  engine->fade_scale = 1.0f;
  engine->fade_step = 0.0f;
  /* The program is the only thing the emitter reads back; the charge, the
   * focus and the level are constants of both programs. */
  engine->vortex_program = program;
  engine->vortex_frames = 0u;
  motion_rebuild_pattern(engine);
  engine->state = UMH_MOTION_STATE_READY;
  motion_unlock(engine);
  return 0;
}
