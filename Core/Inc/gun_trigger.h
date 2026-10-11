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
#define UMH_GUN_RAMP_UP_MS 300u

/* Power fall time (fade-out) from full power to 0 in milliseconds.
 * Recommended range: 50-500 ms. Lower values = faster fall. */
#define UMH_GUN_RAMP_DOWN_MS 500u

/* Hard deadline for the fade-out.  The ramp itself and the closing STOP are
 * advanced by the render task, the only context that can submit at frame rate;
 * the protocol task arms that ramp and re-checks this deadline on every poll,
 * so a fade the render task cannot finish (SPI mutex held, task starved, a link
 * that refuses every frame) is muted from the protocol task anyway.  An open
 * switch must never leave the array firing, so this is a safety net and not a
 * second fade path: in normal operation the STOP happens at
 * UMH_GUN_RAMP_DOWN_MS and this timeout never expires. */
#define UMH_GUN_RELEASE_TIMEOUT_MS (UMH_GUN_RAMP_DOWN_MS + 100u)

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
  uint8_t current_level;      /* current envelope value (0-128), LED intensity */
  /* Levels the envelope is written into.  These are the plane-wave levels and
   * must never be scaled in place: the envelope is applied to this pristine
   * copy on every submit, so a repeated refresh cannot compound it. */
  uint8_t base_level[UMH_DEVICE_CHANNEL_COUNT];
  uint8_t base_ready;         /* base_level[] holds a built plane wave        */
  uint8_t envelope_sent;      /* envelope value the last accepted frame had   */
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
/* 1 from the moment the takeover's emit() has built the plane wave until the
 * release.  The render task gates the shot branch on this instead of on
 * gun_trigger_owns_output(), because between the claim and the emit the frame
 * still belongs to the previous producer. */
uint8_t gun_trigger_holds_image(const umh_gun_trigger_t *trigger);
/* 1 from the release edge until the fade-out has been muted: the switch is open
 * and the array is still winding down.  The protocol task uses it to tell a held
 * shot from a shot that is already fading, so a press that lands inside the
 * 100 ms fade re-arms the ramp instead of waiting for the STOP. */
uint8_t gun_trigger_releasing(const umh_gun_trigger_t *trigger);
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
 * the trigger was already released or the FPGA did not accept the frame.  The
 * frame is always rebuilt from the plane-wave levels before it is packed, so
 * the fade envelope scales it instead of accumulating on it. */
int gun_trigger_emit(umh_gun_trigger_t *trigger, umh_spatial_renderer_t *renderer,
                     fpga_link_t *link);
/* Release edge: latch the fade-out and hand the channels back afterwards.
 *
 * Idempotent, because it is called once per protocol-task iteration while the
 * switch is open: the first call arms the fade, and the calls after it are the
 * watchdog that force-mutes a fade the render task did not finish within
 * UMH_GUN_RELEASE_TIMEOUT_MS.  Re-arming the ramp on every call is exactly the
 * bug this latch exists to prevent -- it held the envelope at ~90 % forever, so
 * the LEDs stayed at full red and the array kept firing with the switch open. */
int gun_trigger_release(umh_gun_trigger_t *trigger, fpga_link_t *link);
/* Render task service: advance the fade envelope and keep the held image
 * emitting, self-healing a cleared output.  The image is re-sent only while
 * the envelope is moving (or on the keep-alive / after a host STOP), and the
 * WS2812 brightness always follows the level actually being emitted.
 * Returns the status poll result, or 0 when no shot is held. */
int gun_trigger_refresh(umh_gun_trigger_t *trigger, fpga_link_t *link);

#endif
