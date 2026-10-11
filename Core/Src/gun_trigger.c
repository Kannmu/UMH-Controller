#include "gun_trigger.h"
#include "main.h"
#include "system_status.h"
#include <string.h>

/* PA9 reads low while the shot switch is closed.  The pin keeps the internal
 * pull-up that gpio.c configures, so an open switch (or an unplugged SMA) idles
 * high and only a real short to ground starts a shot. */
static uint8_t gun_pin_pressed(void)
{
  return HAL_GPIO_ReadPin(TRIGGER_GPIO_Port, TRIGGER_Pin) == GPIO_PIN_RESET ? 1u : 0u;
}

/* Compute current output level based on envelope state and elapsed time.
 * Returns level value (0-128) to be applied. */
static uint8_t gun_trigger_compute_level(const umh_gun_trigger_t *trigger,
                                         uint32_t now_ms)
{
  uint32_t elapsed_ms;
  uint32_t total_ramp_ms;
  float progress;

  if (trigger->envelope_state == 0u) {
    /* Idle: no sound output */
    return 0u;
  } else if (trigger->envelope_state == 2u) {
    /* Full power: maximum level (128 maps to full amplitude) */
    return 128u;
  } else if (trigger->envelope_state == 1u) {
    /* Ramping up: linearly increase from 0 to 128 */
    elapsed_ms = now_ms - trigger->ramp_start_ms;
    total_ramp_ms = UMH_GUN_RAMP_UP_MS;
    if (elapsed_ms >= total_ramp_ms) {
      return 128u;
    }
    progress = (float)elapsed_ms / (float)total_ramp_ms;
    return (uint8_t)(128u * progress);
  } else if (trigger->envelope_state == 3u) {
    /* Ramping down: linearly decrease from 128 to 0 */
    elapsed_ms = now_ms - trigger->ramp_start_ms;
    total_ramp_ms = UMH_GUN_RAMP_DOWN_MS;
    if (elapsed_ms >= total_ramp_ms) {
      return 0u;
    }
    progress = 1.0f - ((float)elapsed_ms / (float)total_ramp_ms);
    return (uint8_t)(128u * progress);
  }

  return 0u;
}

void gun_trigger_init(umh_gun_trigger_t *trigger)
{
  if (trigger == NULL) return;
  memset(trigger, 0, sizeof(*trigger));
  trigger->raw = gun_pin_pressed();
  /* Start released even if the pin is already low: the debounce then has to see
   * the level twice before a shot can start, exactly like any other press. */
  trigger->pressed = 0u;
  trigger->envelope_state = 0u;
  trigger->current_level = 0u;
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

void gun_trigger_claim_output(umh_gun_trigger_t *trigger)
{
  if (trigger == NULL) return;
  trigger->engaged = 1u;
  trigger->refresh_pending = 1u;
  /* Initialize envelope: start ramping up */
  trigger->envelope_state = 1u;
  trigger->ramp_start_ms = HAL_GetTick();
  trigger->current_level = 0u;
}

void gun_trigger_drop_output(umh_gun_trigger_t *trigger)
{
  if (trigger == NULL) return;
  trigger->engaged = 0u;
  trigger->refresh_pending = 0u;
  trigger->envelope_state = 0u;
  trigger->current_level = 0u;
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
  uint32_t now_ms;
  if (trigger == NULL || renderer == NULL || link == NULL) return -1;
  /* A tap shorter than the debounce window never produces a press edge, and a
   * tap released while the takeover is still running must not leave the output
   * armed either. */
  if (trigger->pressed == 0u) return -2;

  now_ms = HAL_GetTick();
  /* Compute the current level based on the envelope state */
  trigger->current_level = gun_trigger_compute_level(trigger, now_ms);

  /* Plane wave: every element emits the same acoustic phase, so there is no
   * source phase and no propagation-delay term.  spatial_renderer_accumulate_q10
   * takes the commanded channel phase and adds the channel's calibration
   * correction through spatial_renderer_finalize, so commanding exactly
   * phase_offset_q10[i] makes the emitted wavefront flat across the 84
   * elements; the same helper applies the per-channel gain, and finalize maps a
   * magnitude of 1.0 to the 50% duty (level 128) that is this hardware's full
   * power.  This is the far-field limit of the spatial point source, i.e. the
   * same code path as any other producer, only with the distance term dropped. */
  magnitude = (float)UMH_GUN_SOURCE_LEVEL * (1.0f / 255.0f);
  memset(&trigger->frame, 0, sizeof(trigger->frame));
  memset(trigger->real_accum, 0, sizeof(trigger->real_accum));
  memset(trigger->imag_accum, 0, sizeof(trigger->imag_accum));
  for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i)
    spatial_renderer_accumulate_q10(renderer, i, renderer->phase_offset_q10[i],
                                    magnitude, trigger->real_accum,
                                    trigger->imag_accum);
  if (spatial_renderer_finalize(renderer, trigger->real_accum,
                                trigger->imag_accum, &trigger->frame) != 0) return -3;

  /* Apply envelope: scale all channel levels by the current envelope value */
  for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) {
    uint32_t scaled = ((uint32_t)trigger->frame.channels[i].level * trigger->current_level) / 128u;
    trigger->frame.channels[i].level = (uint8_t)(scaled & 0xFFu);
  }

  trigger->frame.sequence = ++trigger->sequence;
  trigger->frame.deadline = system_time_us();
  /* Submit straight to the FPGA.  The shot is a one-shot state change, not a
   * timeline, so it must not wait behind frames still queued in the ring. */
  if (fpga_link_submit(link, &trigger->frame) != 0) {
    /* A frame the FPGA is still building returns zero credit for one round
     * trip; one status poll is enough to see the builder idle again. */
    (void)fpga_link_poll_status(link);
    if (fpga_link_submit(link, &trigger->frame) != 0) {
      system_status_fault(UMH_FAULT_FPGA_OUTPUT, 1u, UMH_FAULT_WARNING);
      return -4;
    }
  }

  /* Update envelope state progression */
  if (trigger->envelope_state == 1u) {
    /* Check if ramp-up is complete */
    if (gun_trigger_compute_level(trigger, now_ms) >= 128u) {
      trigger->envelope_state = 2u; /* Move to full power state */
    }
  }

  trigger->engaged = 1u;
  trigger->refresh_pending = 1u;
  trigger->last_submit_ms = now_ms;
  ++trigger->fires;
  return 0;
}

int gun_trigger_release(umh_gun_trigger_t *trigger, fpga_link_t *link)
{
  if (trigger == NULL || link == NULL) return -1;

  /* Start fade-out instead of immediately stopping */
  trigger->envelope_state = 3u;
  trigger->ramp_start_ms = HAL_GetTick();
  trigger->refresh_pending = 1u;

  return 0;
}

int gun_trigger_refresh(umh_gun_trigger_t *trigger, fpga_link_t *link)
{
  const fpga_status_wire_t *status;
  uint32_t now;
  uint8_t wanted;
  int result;
  if (trigger == NULL || link == NULL || trigger->engaged == 0u) return 0;

  /* One status transaction per render iteration.  The FPGA link watchdog clears
   * a running output after ~30 ms without chip select, so this poll is what
   * keeps a held shot emitting while nothing else is being submitted. */
  result = fpga_link_poll_status(link);
  status = fpga_link_status(link);
  now = HAL_GetTick();

  /* Update current level based on envelope state */
  trigger->current_level = gun_trigger_compute_level(trigger, now);

  /* Check if we should transition states or fully stop */
  if (trigger->envelope_state == 3u) {
    /* Ramping down: check if complete */
    if (trigger->current_level == 0u && (now - trigger->ramp_start_ms) >= UMH_GUN_RAMP_DOWN_MS) {
      /* Ramp-down complete: stop output and release */
      trigger->engaged = 0u;
      trigger->refresh_pending = 0u;
      trigger->envelope_state = 0u;
      return fpga_link_safe_stop(link);
    }
  }

  /* Re-send when the takeover asked for one last word (refresh_pending), when
   * the FPGA reports the output stopped, or as a slow keep-alive refresh. */
  wanted = (trigger->refresh_pending != 0u ||
            (status->status_flags & FPGA_STATUS_RUNNING) == 0u ||
            (uint32_t)(now - trigger->last_submit_ms) >= UMH_GUN_REFRESH_MS ||
            trigger->envelope_state != 2u) ? 1u : 0u;

  if (wanted != 0u) {
    /* Apply envelope scaling to frame levels */
    uint16_t i;
    for (i = 0u; i < UMH_DEVICE_CHANNEL_COUNT; ++i) {
      uint32_t scaled = ((uint32_t)trigger->frame.channels[i].level * trigger->current_level) / 128u;
      trigger->frame.channels[i].level = (uint8_t)(scaled & 0xFFu);
    }

    trigger->frame.sequence = ++trigger->sequence;
    trigger->frame.deadline = system_time_us();
    if (fpga_link_submit(link, &trigger->frame) == 0) {
      trigger->last_submit_ms = now;
      trigger->refresh_pending = 0u;
      /* The release path runs in the protocol task and can land while this
       * submit waits for the SPI mutex.  Its STOP would then be followed by
       * this frame, which would leave the array firing with the switch open;
       * stop again whenever ownership disappeared under us. */
      if (trigger->engaged == 0u) result = fpga_link_safe_stop(link);
    }
  }
  return result;
}
