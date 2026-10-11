#ifndef GUN_TRIGGER_H
#define GUN_TRIGGER_H

#include <stdint.h>
#include "fpga_link.h"
#include "frame_ring.h"
#include "spatial_renderer.h"

/* PA9 fire trigger -- the "ultrasound gun" input.
 *
 * PA9 is a keyed digital input, not an output: the internal pull-up holds it
 * high while the switch is open and the switch pulls it to ground when it is
 * pressed.  R176 (22 ohm, 0402) sits between the pin and U174.5, the centre of
 * the SMA footprint, so the shot button is a plug-in interface instead of a
 * solder joint.
 *
 * A press is a hardware override that works in every device mode.  The trigger
 * claims the 84-channel output, takes it away from whichever producer held it
 * (block plan, demo, motion, modulated aperture) and emits one static plane
 * wave at full source amplitude: every element drives the same acoustic phase,
 * with no propagation-delay term, which is the far-field limit of the spatial
 * point source.  The image is built by spatial_renderer and submitted through
 * fpga_link, so the shot shares the element geometry, calibration phases, level
 * semantics and SPI path with every other producer; there is no second
 * rendering or link path.  Releasing the switch stops the output and leaves the
 * device in its default silent state.
 *
 * Only the input task samples the pin, only the protocol task starts or stops a
 * shot, and only the render task keeps it alive.  Those are the same three
 * contexts the rest of the output producers already use.
 */

/* Full source amplitude.  spatial_renderer_finalize maps a magnitude of 1.0 to
 * level 128, the 50% carrier duty that actually launches a 40 kHz wave; 255
 * would be 99.6% DC and would leave no carrier to emit. */
#define UMH_GUN_SOURCE_LEVEL 255u

/* === PA9 Debounce Configuration ===
 * Debounce samples at 10 ms cadence to ignore brief disconnections (e.g., 0.3 ms).
 * Default: 1 sample = ~10 ms, sufficient for most mechanical switches. Increase for noisy environments. */
#define UMH_GUN_DEBOUNCE_SAMPLES 5u

/* === Envelope Ramp Configuration (Fade In/Out) ===
 * Controls ultrasound power rise and fall time for smooth operation.
 *
 * Power rise time (fade-in) from 0 to full power in milliseconds.
 * Recommended range: 50-500 ms. Lower values = faster rise. */
#define UMH_GUN_RAMP_UP_MS 100u

/* Power fall time (fade-out) from full power to 0 in milliseconds.
 * Recommended range: 50-500 ms. Lower values = faster fall. */
#define UMH_GUN_RAMP_DOWN_MS 100u

/* While a shot is held the render task re-sends the same image at this rate
 * even if the FPGA still reports RUNNING.  The shot is a state, not a timeline,
 * so this is a watchdog refresh: it also repairs the output after a host STOP
 * or an FPGA link dropout cleared it mid-shot. */
#define UMH_GUN_REFRESH_MS 200u

typedef struct {
  volatile uint8_t pressed;   /* debounced pin: 1 while the switch is closed  */
  volatile uint8_t engaged;   /* the trigger currently owns the 84 channels   */
  volatile uint8_t refresh_pending; /* render task must re-assert the image   */
  uint8_t raw;                /* latest raw sample of PA9 (1 = low = pressed) */
  uint8_t stable_samples;     /* debounce progress towards the raw sample     */
  uint32_t fires;             /* press edges that started a shot              */
  uint32_t deferred;          /* press edges refused, e.g. during calibration */
  uint32_t last_submit_ms;    /* HAL tick of the last FRAME transaction       */
  uint32_t ramp_start_ms;     /* HAL tick when envelope ramp started          */
  uint32_t sequence;          /* last frame sequence handed to the FPGA       */
  uint8_t envelope_state;     /* 0=idle, 1=ramping up, 2=full, 3=ramping down */
  uint8_t current_level;      /* current ultrasound power level (0-128)       */
  uint16_t reserved;          /* padding for alignment                        */
  umh_output_frame_t frame;   /* the held plane-wave image                    */
  /* Plane-wave accumulation, shared with spatial_renderer_finalize.  Keeping
   * them here instead of on the protocol task stack holds that task's worst
   * chain below its 3 KiB grant (see the stack table in app_freertos.c). */
  float real_accum[UMH_DEVICE_CHANNEL_COUNT];
  float imag_accum[UMH_DEVICE_CHANNEL_COUNT];
} umh_gun_trigger_t;

void gun_trigger_init(umh_gun_trigger_t *trigger);
/* Input task, one call per 10 ms scan.  Cheap enough for the 512 B input task
 * stack: it only reads the pin and advances one debounce counter. */
void gun_trigger_sample(umh_gun_trigger_t *trigger);
uint8_t gun_trigger_pressed(const umh_gun_trigger_t *trigger);
uint8_t gun_trigger_owns_output(const umh_gun_trigger_t *trigger);
/* Claim the output before the producer takeover, so the render task stops
 * pulling frames out of the ring while the takeover resets it.  The claim also
 * marks the image for one forced re-assert: a producer that was already inside
 * an SPI transaction when the claim was set can still push one stale frame
 * afterwards, and the render task runs after that producer has returned. */
void gun_trigger_claim_output(umh_gun_trigger_t *trigger);
/* Undo a claim whose takeover failed; the render task resumes its normal path. */
void gun_trigger_drop_output(umh_gun_trigger_t *trigger);
void gun_trigger_note_deferred(umh_gun_trigger_t *trigger);
/* Build and submit the plane-wave image.  Fails, without owning anything, when
 * the trigger was already released or the FPGA did not accept the frame. */
int gun_trigger_emit(umh_gun_trigger_t *trigger, umh_spatial_renderer_t *renderer,
                     fpga_link_t *link);
/* STOP the output and hand the channels back to the normal producer order. */
int gun_trigger_release(umh_gun_trigger_t *trigger, fpga_link_t *link);
/* Render task service: keep the held image emitting and self-heal a cleared
 * output.  Returns the status poll result, or 0 when no shot is held. */
int gun_trigger_refresh(umh_gun_trigger_t *trigger, fpga_link_t *link);

#endif
