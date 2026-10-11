#include "gun_trigger.h"
#include "main.h"
#include "system_status.h"
#include <stddef.h>
#include <string.h>

/* The SWD bench tools hard-code these offsets (Utiles/gun_probe.tcl and
 * Utiles/umh_gun_trigger_test.py); keep them pinned to the compiler's layout. */
_Static_assert(offsetof(umh_gun_trigger_t, level) == 4u, "probe offset level");
_Static_assert(offsetof(umh_gun_trigger_t, base_ready) == 89u, "probe offset base_ready");
_Static_assert(offsetof(umh_gun_trigger_t, last_submit_ms) == 100u, "probe offset last_submit");
_Static_assert(offsetof(umh_gun_trigger_t, frame.channels[0].level) == 129u,
               "probe offset channel level");
_Static_assert(offsetof(umh_gun_trigger_t, frame.rgb[0].blue) == 298u,
               "probe offset LED blue");

/* PA9 reads low while the shot switch is closed.  The pin keeps the internal
 * pull-up that gpio.c configures, so an open switch (or an unplugged SMA) idles
 * high and only a real short to ground starts a shot. */
static uint8_t gun_pin_pressed(void)
{
  return HAL_GPIO_ReadPin(TRIGGER_GPIO_Port, TRIGGER_Pin) == GPIO_PIN_RESET ? 1u : 0u;
}

/* Write the current level into the frame.
 *
 * base_level[] holds the plane-wave levels produced by the renderer and is never
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
 * plane wave the render task packed from the old level. */
static int gun_trigger_mute(umh_gun_trigger_t *trigger, fpga_link_t *link)
{
  trigger->engaged = 0u;
  trigger->level = 0u;
  trigger->base_ready = 0u;
  trigger->level_sent = 0u;
  return fpga_link_safe_stop(link);
}

void gun_trigger_init(umh_gun_trigger_t *trigger)
{
  if (trigger == NULL) return;
  memset(trigger, 0, sizeof(*trigger));
  trigger->raw = gun_pin_pressed();
  /* Start released even if the pin is already low: the debounce then has to see
   * the level UMH_GUN_DEBOUNCE_SAMPLES times before a shot can start, exactly
   * like any other press. */
  trigger->pressed = 0u;
  trigger->level = 0u;
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

void gun_trigger_claim_output(umh_gun_trigger_t *trigger)
{
  if (trigger == NULL) return;
  trigger->engaged = 1u;
  /* The image must be rebuilt: the frame may still hold the previous producer's
   * channels until emit() renders the plane wave.  Clearing level_sent as well
   * makes the render task re-assert the image once it is built, so a claim that
   * is followed by one render iteration before emit() cannot be served with a
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
  uint16_t i;
  float magnitude;
  if (trigger == NULL || renderer == NULL || link == NULL) return -1;
  /* A tap shorter than the debounce window never produces a press edge, and a
   * tap released while the takeover is still running must not leave the output
   * armed either. */
  if (trigger->pressed == 0u) return -2;
  trigger->level = (uint8_t)UMH_GUN_LEVEL;

  /* Plane wave: every element emits the same acoustic phase, so there is no
   * source phase and no propagation-delay term.  Commanding exactly
   * phase_offset_q10[i] makes the emitted wavefront flat across the 84 elements;
   * the helper applies the per-channel calibration gain, and finalize maps a
   * magnitude of 1.0 to the full-power level. */
  magnitude = 1.0f;
  memset(&trigger->frame, 0, sizeof(trigger->frame));
  memset(trigger->real_accum, 0, sizeof(trigger->real_accum));
  memset(trigger->imag_accum, 0, sizeof(trigger->imag_accum));
  for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i)
    spatial_renderer_accumulate_q10(renderer, i, renderer->phase_offset_q10[i],
                                    magnitude, trigger->real_accum,
                                    trigger->imag_accum);
  if (spatial_renderer_finalize(renderer, trigger->real_accum,
                                trigger->imag_accum, &trigger->frame) != 0) return -3;

  /* Snapshot the untouched plane wave, then emit it at the current level.  The
   * snapshot is what every later refresh scales from. */
  for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i)
    trigger->base_level[i] = trigger->frame.channels[i].level;
  trigger->base_ready = 1u;
  gun_trigger_apply_level(trigger);

  trigger->frame.sequence = ++trigger->sequence;
  trigger->frame.deadline = system_time_us();
  if (fpga_link_submit(link, &trigger->frame) != 0) {
    /* A frame the FPGA is still building returns zero credit for one round trip;
     * one status poll is enough to see the builder idle again. */
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
   * frame below cannot be raced by a plane wave packed from the old level. */
  return gun_trigger_mute(trigger, link);
}

int gun_trigger_refresh(umh_gun_trigger_t *trigger, fpga_link_t *link)
{
  const fpga_status_wire_t *status;
  uint32_t now;
  uint8_t wanted;
  int result;
  if (trigger == NULL || link == NULL || trigger->engaged == 0u ||
      trigger->base_ready == 0u) return 0;

  /* One status transaction per render iteration: the FPGA link watchdog clears a
   * running output after ~30 ms without chip select. */
  result = fpga_link_poll_status(link);
  status = fpga_link_status(link);
  now = HAL_GetTick();

  /* Re-send when the takeover asked for the first word, when the FPGA reports the
   * output stopped, or as the slow keep-alive.  A held plane wave is static, so a
   * stable shot costs no SPI traffic between keep-alives. */
  wanted = (trigger->level_sent != trigger->level ||
            (status->status_flags & FPGA_STATUS_RUNNING) == 0u ||
            (uint32_t)(now - trigger->last_submit_ms) >= UMH_GUN_REFRESH_MS) ? 1u : 0u;

  if (wanted != 0u) {
    gun_trigger_apply_level(trigger);
    trigger->frame.sequence = ++trigger->sequence;
    trigger->frame.deadline = system_time_us();
    if (fpga_link_submit(link, &trigger->frame) == 0) {
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
