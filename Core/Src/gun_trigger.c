#include "gun_trigger.h"
#include "main.h"
#include "system_status.h"
#include "umh_fast_math.h"
#include "umh_utils.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
/* ---------------------------------------------------------------------------
 * The six gun modes.
 *
 * Mode 1 is the historical single focus and goes through the ordinary point
 * producer, spatial_renderer_accumulate_point(), exactly as before.  Modes 2..5
 * are phase-only fields that no other producer needs, so they are synthesised
 * here and handed to the same spatial_renderer_finalize() and the same
 * fpga_link FRAME transaction as everything else.  There is no second phase
 * table, no second carrier and no second SPI path: this file only computes the
 * per-channel carrier phase and hands it to the renderer, in the 1/1024-turn
 * (q10) domain the renderer's own trig table uses.
 *
 * Every mode drives all calibrated channels at the same amplitude, so the
 * emitted power is identical across modes and equals a plane wave's: the phase
 * decides where the energy goes, not how much of it there is. */
/* Phase-domain constants. */
#define UMH_GUN_Q10_PER_TURN 1024.0f
#define UMH_GUN_DEG_TO_RAD   0.01745329252f
#define UMH_GUN_BUILD_ID 0x474E0023u
volatile uint32_t gun_trigger_build_id(void)
{
  return UMH_GUN_BUILD_ID;
}
volatile uint32_t gun_trigger_build_id_seen;
/* The SWD bench probe (Utiles/umh_gun_probe.py) reads this struct by offset.
 * Pinning the offsets here makes a layout change a compile error instead of a
 * silently wrong dump.  They cover every field the probe decodes; the arrays
 * at the end are not part of the wire image. */
_Static_assert(offsetof(umh_gun_trigger_t, level) == 4u, "probe offset level");
_Static_assert(offsetof(umh_gun_trigger_t, mode) == 5u, "probe offset mode");
_Static_assert(offsetof(umh_gun_trigger_t, collapse_t0_ms) == 8u,
               "probe offset collapse_t0");
_Static_assert(offsetof(umh_gun_trigger_t, render_ms) == 12u, "probe offset render_ms");
_Static_assert(offsetof(umh_gun_trigger_t, last_poll_ms) == 16u,
               "probe offset last_poll");
_Static_assert(offsetof(umh_gun_trigger_t, base_level) == 20u,
               "probe offset base_level");
_Static_assert(offsetof(umh_gun_trigger_t, base_ready) == 104u,
               "probe offset base_ready");
_Static_assert(offsetof(umh_gun_trigger_t, level_sent) == 105u,
               "probe offset level_sent");
_Static_assert(offsetof(umh_gun_trigger_t, base_prev) == 106u,
               "probe offset base_prev");
_Static_assert(offsetof(umh_gun_trigger_t, fires) == 276u, "probe offset fires");
_Static_assert(offsetof(umh_gun_trigger_t, deferred) == 280u,
               "probe offset deferred");
_Static_assert(offsetof(umh_gun_trigger_t, last_submit_ms) == 284u,
               "probe offset last_submit");
_Static_assert(offsetof(umh_gun_trigger_t, sequence) == 288u,
               "probe offset sequence");
_Static_assert(offsetof(umh_gun_trigger_t, frame) == 292u, "probe offset frame");
_Static_assert(offsetof(umh_gun_trigger_t, frame.channels[0].level) == 313u,
               "probe offset channel level");
_Static_assert(offsetof(umh_gun_trigger_t, frame.rgb[0].blue) == 482u,
               "probe offset LED blue");
const char *const umh_gun_mode_names[UMH_GUN_MODE_COUNT] = {
  "SINGLE FOCUS", "PLANE WAVE", "VORTEX L1", "BESSEL L1", "BOTTLE DYN",
  "VOICE SPEAKER"
};
/* Symmetric Q15 low-pass: unity DC gain, <= 0.11 dB ripple through 3 kHz,
 * >= 42 dB rejection from 4.4 kHz.  The capture loop uses fifteen Cortex-M4
 * dual MACs plus the centre tap; no sample-history shifting is required. */
const int16_t umh_gun_voice_fir[UMH_GUN_VOICE_FIR_TAPS] = {
     97, 131, -81, -266, 25, 486, 150, -748,
   -543, 1030, 1301, -1274, -2960, 1448, 10248, 14680,
  10248, 1448, -2960, -1274, 1301, 1030, -543, -748,
    150, 486, 25, -266, -81, 131, 97
};
/* The SWD bench probe (Utiles/umh_gun_probe.py) reads this struct by offset.
 * Pinning the offsets here makes a layout change a compile error instead of a
 * silently wrong dump. */
/* PA9 reads low while the shot switch is closed.  The pin keeps the internal
 * pull-up that gpio.c configures, so an open switch (or an unplugged SMA) idles
 * high and only a real short to ground starts a shot. */
static uint8_t gun_pin_pressed(void)
{
  return HAL_GPIO_ReadPin(TRIGGER_GPIO_Port, TRIGGER_Pin) == GPIO_PIN_RESET ? 1u : 0u;
}
/* One channel of a phase-only mode.  phase_q10 is the absolute carrier phase in
 * 1/1024 turn and already contains the calibration phase offset, which is how
 * spatial_renderer_accumulate_point() spells the same quantity; the per-channel
 * calibration gain is applied inside the helper.  The source amplitude is 1.0
 * (full scale) for every channel, so the mode is a pure phase field. */
static void gun_accumulate(umh_gun_trigger_t *trigger,
                           const umh_spatial_renderer_t *renderer, uint16_t index,
                           int32_t phase_q10)
{
  spatial_renderer_accumulate_q10(renderer, index, phase_q10, 1.0f,
                                  trigger->real_accum, trigger->imag_accum);
}
/* Fill trigger->frame with the selected mode at full source amplitude.
 *
 * Returns 0, or a negative value with the frame left as the caller had it.
 * now_ms anchors the mode 5 collapse and is ignored by the static modes. */
static int gun_trigger_build(umh_gun_trigger_t *trigger,
                             umh_spatial_renderer_t *renderer, uint32_t now_ms)
{
  uint16_t i;
  uint16_t active;
  const uint8_t *active_index;
  const umh_element_coordinate_t *coords;
  float phase_scale;
  float beta_sin = 0.0f;
  float focus_z = 0.0f;
  uint8_t mode;
  if (trigger == NULL || renderer == NULL) return -1;
  if (renderer->profile == NULL) return -3;
  mode = trigger->mode;
  if (mode >= UMH_GUN_MODE_COUNT) mode = (uint8_t)UMH_GUN_MODE_FOCUS;
  if (mode == UMH_GUN_MODE_FOCUS) {
    umh_spatial_point_t focus;
    /* Single focus: the commanded point sits on the array axis, UMH_GUN_FOCUS_UM
     * in front of the transducer faces.  Every element then carries the
     * propagation delay of its own distance to that point on top of its
     * calibration phase, so the wavefront converges instead of staying flat.
     * This is the phase law spatial_renderer_point() already uses for every
     * other producer, so no second phase implementation exists here. */
    focus.x_um = 0;
    focus.y_um = 0;
    /* The profile frame puts z = 0 on the PCB microphone-port plane and the
     * piezo face at +z, and UMH_GUN_FOCUS_UM is measured from that face. */
    focus.z_um = (int32_t)UMH_GUN_FOCUS_UM + renderer->profile->coordinates[0].z_um;
    focus.level = 255u;
    focus.phase = 0u;
    focus.source_id = 0u;
    memset(&trigger->frame, 0, sizeof(trigger->frame));
    memset(trigger->real_accum, 0, sizeof(trigger->real_accum));
    memset(trigger->imag_accum, 0, sizeof(trigger->imag_accum));
    if (spatial_renderer_accumulate_point(renderer, &focus, trigger->real_accum,
                                          trigger->imag_accum) != 0) return -3;
  } else {
    if ((renderer->profile->capability_flags & UMH_PROFILE_CAP_GEOMETRY_VALID) == 0u)
      return -2;
    if (renderer->carrier_hz == 0u || renderer->sound_speed_um_per_s == 0u) return -3;
    /* distance_um * phase_scale is the propagation phase in q10, the same
     * product spatial_renderer_accumulate_point() forms. */
    phase_scale = UMH_GUN_Q10_PER_TURN /
                  ((float)renderer->sound_speed_um_per_s / (float)renderer->carrier_hz);
    beta_sin = sinf(UMH_GUN_BESSEL_BETA_DEG * UMH_GUN_DEG_TO_RAD);
    if (mode == UMH_GUN_MODE_VORTEX) {
      focus_z = (float)UMH_GUN_VORTEX_FOCUS_UM +
                (float)renderer->profile->coordinates[0].z_um;
    } else if (mode == UMH_GUN_MODE_BOTTLE) {
      /* Collapse the waist from the far depth to the near one over
       * UMH_GUN_BOTTLE_COLLAPSE_MS and hold it there for the rest of the shot.
       * Smoothstep rather than a straight line: the ring closes fastest in the
       * middle of the collapse and eases into the trap, which is what makes the
       * last millimetres of the approach gentle on a captured particle. */
      float t = (float)(uint32_t)(now_ms - trigger->collapse_t0_ms) /
                (float)UMH_GUN_BOTTLE_COLLAPSE_MS;
      if (t > 1.0f) t = 1.0f;
      t = t * t * (3.0f - 2.0f * t);
      focus_z = (float)UMH_GUN_BOTTLE_FAR_UM +
                ((float)UMH_GUN_BOTTLE_NEAR_UM - (float)UMH_GUN_BOTTLE_FAR_UM) * t +
                (float)renderer->profile->coordinates[0].z_um;
    } else {
      focus_z = 0.0f;   /* plane wave: no distance term at all */
    }
    memset(&trigger->frame, 0, sizeof(trigger->frame));
    memset(trigger->real_accum, 0, sizeof(trigger->real_accum));
    memset(trigger->imag_accum, 0, sizeof(trigger->imag_accum));
    active = renderer->active_count;
    active_index = renderer->active_index;
    coords = renderer->profile->coordinates;
    for (i = 0u; i < active; ++i) {
      uint16_t c = active_index[i];
      float x = (float)coords[c].x_um;
      float y = (float)coords[c].y_um;
      float rho = umh_sqrtf(fmaf(x, x, y * y));
      int32_t phase_q10 = renderer->phase_offset_q10[c];
      if (mode == UMH_GUN_MODE_PLANE) {
        /* Flat wavefront: the calibration offset alone.  All 84 elements leave
         * in phase, so the aperture radiates straight ahead and stays coherent
         * on the axis all the way out -- this is the strongest mode on axis and
         * the only one with no wavefront shape at all.  It deliberately carries
         * no azimuthal winding: a plane wave with a phase singularity is a
         * vortex beam, not a plane wave. */
        gun_accumulate(trigger, renderer, c, phase_q10);
        continue;
      }
      /* One azimuthal turn of the carrier around the axis: the l = 1 winding
       * that forces the on-axis null every hollow mode in this file relies on.
       * It is one full turn for every channel, whatever the radius, so the
       * aperture's ring integral cancels exactly on the axis. */
      phase_q10 += (int32_t)trigger->azimuth_q10[c];
      if (mode == UMH_GUN_MODE_BESSEL) {
        /* Conical wavefront: every element radiates at the same angle beta, so
         * the contributions interfere into a Bessel beam whose hollow core holds
         * its radius over the Bessel zone instead of converging to one plane. */
        phase_q10 -= (int32_t)(rho * beta_sin * phase_scale);
      } else {
        /* Spherical wavefront converging on the axis at focus_z, optionally
         * combined with the cone of the bottle mode. */
        float z_off = focus_z - (float)coords[c].z_um;
        float distance = umh_sqrtf(fmaf(z_off, z_off, rho * rho));
        phase_q10 -= (int32_t)(distance * phase_scale);
        if (mode == UMH_GUN_MODE_BOTTLE)
          phase_q10 -= (int32_t)(rho * beta_sin * phase_scale);
      }
      gun_accumulate(trigger, renderer, c, phase_q10);
    }
  }
  if (spatial_renderer_finalize(renderer, trigger->real_accum, trigger->imag_accum,
                                &trigger->frame) != 0) return -3;
  /* Snapshot the untouched image, then emit it at the current level.  The
   * snapshot is what every later refresh scales from. */
  for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i)
    trigger->base_level[i] = trigger->frame.channels[i].level;
  trigger->base_ready = 1u;
  return 0;
}
/* Write the current level into the frame.
 *
 * base_level[] holds the image levels produced by the renderer and is never
 * modified; every submission copies it through the level instead of scaling
 * frame.channels[] in place.  Applying the level to a pristine copy keeps
 * envelope = 128 * level / 128 exact for any number of refreshes.
 *
 * The WS2812 word comes from the same level and is blue, so the four LEDs are a
 * true indicator of the emitted intensity: dark when the output is muted and
 * 3/4 brightness at full power. */
static void gun_trigger_apply_level(umh_gun_trigger_t *trigger)
{
  uint16_t i;
  uint8_t level = trigger->level;
  uint8_t brightness;
  if (level > UMH_GUN_LEVEL) level = (uint8_t)UMH_GUN_LEVEL;
  for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) {
    uint32_t scaled = trigger->base_ready != 0u
                          ? ((uint32_t)trigger->base_level[i] * level) / UMH_GUN_LEVEL
                          : 0u;
    trigger->frame.channels[i].level = (uint8_t)scaled;
  }
  brightness = (uint8_t)((3u * level) / 2u);
  for (i = 0u; i < UMH_DEVICE_RGB_COUNT; ++i) {
    trigger->frame.rgb[i].red = 0u;
    trigger->frame.rgb[i].green = 0u;
    trigger->frame.rgb[i].blue = brightness;
  }
  trigger->frame.update_flags |= UMH_FRAME_FLAG_RGB;
}
/* Mute the array now and hand the channels back to the normal producer order.
 * The frame is written at level 0 first, so the STOP cannot be followed by a
 * focus the render task packed from the old level. */
static int gun_trigger_mute(umh_gun_trigger_t *trigger, fpga_link_t *link)
{
  trigger->engaged = 0u;
  trigger->level = 0u;
  trigger->base_ready = 0u;
  trigger->level_sent = 0u;
  /* The next press replays the bottle collapse from the wide end. */
  trigger->collapse_t0_ms = 0u;
  trigger->render_ms = 0u;
  return fpga_link_safe_stop(link);
}
void gun_trigger_init(umh_gun_trigger_t *trigger)
{
  const umh_device_profile_t *profile;
  uint16_t i;
  if (trigger == NULL) return;
  memset(trigger, 0, sizeof(*trigger));
  /* Cache the build id in RAM for the bench probe; see gun_trigger_build_id(). */
  gun_trigger_build_id_seen = gun_trigger_build_id();
  profile = device_profile_get();
  if (profile != NULL) {
    for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) {
      float x = (float)profile->coordinates[i].x_um;
      float y = (float)profile->coordinates[i].y_um;
      trigger->azimuth_q10[i] = (int16_t)((atan2f(y, x) * UMH_GUN_VORTEX_TURNS /
                                           UMH_TWO_PI) * UMH_GUN_Q10_PER_TURN);
    }
  }
  trigger->raw = gun_pin_pressed();
  /* Start released even if the pin is already low: the debounce then has to see
   * the level UMH_GUN_DEBOUNCE_SAMPLES times before a shot can start, exactly
   * like any other press. */
  trigger->pressed = 0u;
  trigger->level = 0u;
  trigger->mode = (uint8_t)UMH_GUN_MODE_FOCUS;
}
void gun_trigger_sample(umh_gun_trigger_t *trigger)
{
  uint8_t raw;
  if (trigger == NULL) return;
  raw = gun_pin_pressed();
  trigger->raw = raw;
  if (raw == trigger->pressed) {
    trigger->stable_samples = 0u;
    return;
  }
  if (trigger->stable_samples < UMH_GUN_DEBOUNCE_SAMPLES) ++trigger->stable_samples;
  if (trigger->stable_samples >= UMH_GUN_DEBOUNCE_SAMPLES) {
    trigger->stable_samples = 0u;
    trigger->pressed = raw;
  }
}
uint8_t gun_trigger_pressed(const umh_gun_trigger_t *trigger)
{
  return trigger != NULL ? trigger->pressed : 0u;
}
uint8_t gun_trigger_owns_output(const umh_gun_trigger_t *trigger)
{
  return trigger != NULL ? trigger->engaged : 0u;
}
uint8_t gun_trigger_holds_image(const umh_gun_trigger_t *trigger)
{
  return trigger != NULL && trigger->engaged != 0u && trigger->base_ready != 0u ? 1u : 0u;
}
uint8_t gun_trigger_mode(const umh_gun_trigger_t *trigger)
{
  return trigger != NULL ? trigger->mode : (uint8_t)UMH_GUN_MODE_FOCUS;
}
void gun_trigger_set_mode(umh_gun_trigger_t *trigger, uint8_t mode)
{
  if (trigger == NULL) return;
  trigger->mode = mode < UMH_GUN_MODE_COUNT ? mode : (uint8_t)UMH_GUN_MODE_FOCUS;
}
int gun_trigger_select_mode(umh_gun_trigger_t *trigger, uint8_t mode)
{
  if (trigger == NULL || mode >= UMH_GUN_MODE_COUNT) return -1;
  /* A held shot owns the array: changing the mode under it would rewrite the
   * field the render task is refreshing.  The UI keeps the previous mode and the
   * next click works. */
  if (trigger->engaged != 0u) return -1;
  trigger->mode = mode;
  return 0;
}
void gun_trigger_claim_output(umh_gun_trigger_t *trigger)
{
  if (trigger == NULL) return;
  trigger->engaged = 1u;
  /* The image must be rebuilt: the frame may still hold the previous producer's
   * channels until emit() renders the mode.  Clearing level_sent as well makes
   * the render task re-assert the image once it is built, so a claim that is
   * followed by one render iteration before emit() cannot be served with a
   * frame the FPGA has not seen. */
  trigger->base_ready = 0u;
  trigger->level = 0u;
  trigger->level_sent = 0u;
}
void gun_trigger_drop_output(umh_gun_trigger_t *trigger)
{
  if (trigger == NULL) return;
  trigger->engaged = 0u;
  trigger->base_ready = 0u;
  trigger->level = 0u;
}
void gun_trigger_note_deferred(umh_gun_trigger_t *trigger)
{
  if (trigger != NULL) ++trigger->deferred;
}
int gun_trigger_emit(umh_gun_trigger_t *trigger, umh_spatial_renderer_t *renderer,
                     fpga_link_t *link)
{
  if (trigger == NULL || renderer == NULL || link == NULL) return -1;
  /* A tap shorter than the debounce window never produces a press edge, and a
   * tap released while the takeover is still running must not leave the output
   * armed either. */
  if (trigger->pressed == 0u) return -2;
  trigger->level = (uint8_t)UMH_GUN_LEVEL;
  /* Anchor the mode 5 collapse before the field is built, so the first frame of
   * a shot is the widest bottle rather than a partially collapsed one. */
  trigger->collapse_t0_ms = HAL_GetTick();
  trigger->render_ms = trigger->collapse_t0_ms;
  if (gun_trigger_build(trigger, renderer, trigger->collapse_t0_ms) != 0) return -3;
  gun_trigger_apply_level(trigger);
  trigger->frame.sequence = ++trigger->sequence;
  trigger->frame.deadline = system_time_us();
  /* A frame the FPGA is still building returns zero credit for one round trip;
   * one status poll is enough to see the builder idle again. */
  if (fpga_link_submit(link, &trigger->frame) != 0) {
    (void)fpga_link_poll_status(link);
    if (fpga_link_submit(link, &trigger->frame) != 0) {
      system_status_fault(UMH_FAULT_FPGA_OUTPUT, 1u, UMH_FAULT_WARNING);
      return -4;
    }
  }
  trigger->engaged = 1u;
  trigger->level_sent = trigger->level;
  trigger->last_submit_ms = HAL_GetTick();
  ++trigger->fires;
  return 0;
}
int gun_trigger_release(umh_gun_trigger_t *trigger, fpga_link_t *link)
{
  if (trigger == NULL || link == NULL) return -1;
  if (trigger->engaged == 0u) return 0;
  /* Disown first: from here the render task refuses to send anything, so the
   * frame below cannot be raced by a focus packed from the old level. */
  return gun_trigger_mute(trigger, link);
}
int gun_trigger_refresh(umh_gun_trigger_t *trigger, umh_spatial_renderer_t *renderer,
                        fpga_link_t *link)
{
  const fpga_status_wire_t *status;
  uint32_t now;
  uint8_t wanted;
  uint8_t rebuild = 0u;
  int result = 0;
  if (trigger == NULL || link == NULL || trigger->engaged == 0u ||
      trigger->base_ready == 0u) return 0;
  now = HAL_GetTick();
  /* The FPGA link watchdog clears a running output after ~30 ms without chip
   * select, so the link needs a transaction at least every 30 ms; this polls the
   * status at 5 ms and at every keep-alive, which covers it whether or not the
   * image changed.  Polling on every ~1 ms render iteration would only load the
   * SPI bus the dynamic bottle is already using. */
  if ((uint32_t)(now - trigger->last_poll_ms) >= UMH_GUN_POLL_MS ||
      (uint32_t)(now - trigger->last_submit_ms) >= (UMH_GUN_REFRESH_MS / 2u)) {
    result = fpga_link_poll_status(link);
    trigger->last_poll_ms = now;
  }
  status = fpga_link_status(link);
  /* The dynamic bottle is the only mode whose image changes while it is held.
   * Rebuild it once per UMH_GUN_BOTTLE_PERIOD_MS; between rebuilds the field is
   * bit-identical, so the link's hold optimisation costs one SPI frame per step
   * instead of one per millisecond.  The build works on the frame in place, so
   * the previous channel image is saved first and restored if the build fails:
   * a half-built frame must never reach the array. */
  if (trigger->mode == (uint8_t)UMH_GUN_MODE_BOTTLE &&
      (uint32_t)(now - trigger->render_ms) >= UMH_GUN_BOTTLE_PERIOD_MS) {
    memcpy(trigger->base_prev, trigger->frame.channels, sizeof(trigger->base_prev));
    if (renderer != NULL && gun_trigger_build(trigger, renderer, now) == 0) {
      rebuild = memcmp(trigger->base_prev, trigger->frame.channels,
                       sizeof(trigger->base_prev)) != 0u ? 1u : 0u;
    } else {
      memcpy(trigger->frame.channels, trigger->base_prev, sizeof(trigger->base_prev));
    }
    trigger->render_ms = now;
  }
  /* Re-send when the bottle moved, when the takeover asked for the first word,
   * when the FPGA reports the output stopped, or as the slow keep-alive.  A
   * static image costs no SPI traffic between keep-alives. */
  wanted = (rebuild != 0u || trigger->level_sent != trigger->level ||
            (status->status_flags & FPGA_STATUS_RUNNING) == 0u ||
            (uint32_t)(now - trigger->last_submit_ms) >= UMH_GUN_REFRESH_MS) ? 1u : 0u;
  if (wanted != 0u) {
    gun_trigger_apply_level(trigger);
    trigger->frame.sequence = ++trigger->sequence;
    trigger->frame.deadline = system_time_us();
    /* allow_hold: an unchanged channel image must not cost an SPI frame, which
     * is what keeps a 10 ms bottle step cheap. */
    if (fpga_link_submit_allow_hold(link, &trigger->frame, 1u) == 0) {
      trigger->last_submit_ms = now;
      trigger->level_sent = trigger->level;
    }
    /* The release path runs in the protocol task and can land while this submit
     * waits for the SPI mutex.  Its STOP would then be followed by this frame,
     * which would leave the array firing with the switch open. */
    if (trigger->engaged == 0u) result = fpga_link_safe_stop(link);
  }
  return result;
}
