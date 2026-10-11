#ifndef GUN_TRIGGER_H
#define GUN_TRIGGER_H

#include <stdint.h>
#include "fpga_link.h"
#include "frame_ring.h"
#include "spatial_renderer.h"

/* PA9 fire trigger -- the "ultrasound gun" input.
 *
 * The switch is a hardware override: while it is held the trigger owns the 84
 * channels and emits one static plane wave (every element on the same acoustic
 * phase, i.e. the far-field limit of the spatial point source).  The transition
 * is a hard edge in both directions -- 0 to full power on the press, full power
 * to 0 on the release -- with no ramp, no envelope state and no release
 * watchdog to reason about.
 *
 * A hard edge in a 40 kHz envelope is audible: the air self-demodulates the
 * second derivative of the envelope, so switching the array on and off is heard
 * as a click.  That is the intended behaviour here.
 *
 * Only the input task samples the pin and only the protocol task handles the
 * edges.  The render task trafficks the link while a shot is held; it never
 * changes the intensity. */

/* Envelope value of a shot, and the only intensity this module writes: it scales
 * the plane wave into the frame and it scales the WS2812 brightness, so the four
 * LEDs cannot disagree with the ultrasound.  128 is the full source amplitude,
 * because spatial_renderer_finalize maps a magnitude of 1.0 to level 128, the
 * 50% carrier duty that actually launches a 40 kHz wave (255 would be 99.6% DC
 * and would leave no carrier to emit). */
#define UMH_GUN_LEVEL 128u

/* Debounce samples at the input task's 10 ms cadence, to keep mechanical
 * contact bounce from writing a burst of edges into the output. */
#define UMH_GUN_DEBOUNCE_SAMPLES 3u

/* While a shot is held the render task re-sends the same image at this rate.
 * The FPGA link watchdog clears a running output after ~30 ms without chip
 * select; a held plane wave is static, so this keep-alive is also the repair
 * path after a host STOP cleared the output underneath the shot. */
#define UMH_GUN_REFRESH_MS 200u

typedef struct {
  uint8_t pressed;            /* debounced pin: 1 while the switch is closed  */
  volatile uint8_t engaged;   /* the trigger currently owns the 84 channels   */
  uint8_t raw;                /* latest raw sample of PA9 (1 = low = pressed) */
  uint8_t stable_samples;     /* debounce progress towards the raw sample     */
  uint8_t level;              /* 0 while idle, UMH_GUN_LEVEL while firing     */
  /* Levels the envelope is written into.  These are the plane-wave levels and
   * must never be scaled in place: the envelope is applied to this copy on every
   * submit, so a repeated refresh cannot compound it. */
  uint8_t base_level[UMH_DEVICE_CHANNEL_COUNT];
  uint8_t base_ready;         /* base_level[] holds a built plane wave        */
  uint8_t level_sent;         /* level the last accepted frame carried        */
  uint32_t fires;             /* press edges that started a shot              */
  uint32_t deferred;          /* press edges refused, e.g. during calibration */
  uint32_t last_submit_ms;    /* HAL tick of the last FRAME transaction       */
  uint32_t sequence;          /* last frame sequence handed to the FPGA       */
  umh_output_frame_t frame;   /* the held plane-wave image                    */
  /* Plane-wave accumulation, shared with spatial_renderer_finalize.  Keeping
   * them here instead of on the protocol task stack holds that task's worst
   * chain below its 3 KiB grant (see the stack table in app_freertos.c). */
  float real_accum[UMH_DEVICE_CHANNEL_COUNT];
  float imag_accum[UMH_DEVICE_CHANNEL_COUNT];
} umh_gun_trigger_t;

void gun_trigger_init(umh_gun_trigger_t *trigger);
/* Input task, one call per 10 ms scan.  Only reads the pin and advances one
 * debounce counter, so the 512 B input task stack is enough. */
void gun_trigger_sample(umh_gun_trigger_t *trigger);
uint8_t gun_trigger_pressed(const umh_gun_trigger_t *trigger);
uint8_t gun_trigger_owns_output(const umh_gun_trigger_t *trigger);
/* 1 from the moment the takeover's emit() has built the plane wave until the
 * release.  The render task gates the shot branch on this rather than on
 * owns_output(), because between the claim and the emit the frame still holds
 * the previous producer's channels. */
uint8_t gun_trigger_holds_image(const umh_gun_trigger_t *trigger);
/* Claim the output before the producer takeover, so the render task stops
 * pulling frames out of the ring while the takeover resets it. */
void gun_trigger_claim_output(umh_gun_trigger_t *trigger);
/* Undo a claim whose takeover failed; the render task resumes its normal path. */
void gun_trigger_drop_output(umh_gun_trigger_t *trigger);
void gun_trigger_note_deferred(umh_gun_trigger_t *trigger);
/* Build and submit the plane wave at UMH_GUN_LEVEL.  Fails, without owning
 * anything, when the switch was already released or the FPGA did not accept the
 * frame. */
int gun_trigger_emit(umh_gun_trigger_t *trigger, umh_spatial_renderer_t *renderer,
                     fpga_link_t *link);
/* Release edge: full power to 0 and STOP, in one call. */
int gun_trigger_release(umh_gun_trigger_t *trigger, fpga_link_t *link);
/* Render task service: keep the held image alive, self-healing a cleared output.
 * Returns the status poll result, or 0 when no shot is held. */
int gun_trigger_refresh(umh_gun_trigger_t *trigger, fpga_link_t *link);

#endif
