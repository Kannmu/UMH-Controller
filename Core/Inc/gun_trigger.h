#ifndef GUN_TRIGGER_H
#define GUN_TRIGGER_H

#include <stdint.h>
#include "fpga_link.h"
#include "frame_ring.h"
#include "spatial_renderer.h"

/* PA9 fire trigger -- the "ultrasound gun" input.
 *
 * The switch is a hardware override: while it is held the trigger owns the 84
 * channels and emits one of the gun modes below, selected by the gun menu.
 * Modes 1–5 are hard edges in both directions -- 0 to full power on press and
 * full power to 0 on release -- with no ramp or release watchdog.  Voice mode is
 * the explicit exception: it records silently while held, then plays on release
 * through the separate AM aperture-modulation path.
 *
 * A hard edge in a 40 kHz envelope is audible: the air self-demodulates the
 * second derivative of the envelope, so switching the array on and off is heard
 * as a click.  That is the intended behaviour here.
 *
 * Default mode.  Out of the box (before a gun-menu selection) a press fires
 * UMH_GUN_MODE_FOCUS: the single focus on the array axis at UMH_GUN_FOCUS_UM.
 * A selected mode remains active after leaving the menu.
 *
 * Only the input task samples the pin and only the protocol task handles the
 * edges.  The render task trafficks the link while a shot is held and advances
 * the mode 5 collapse, which is a time evolution of the field and therefore
 * belongs on the frame clock; it never changes the intensity. */

/* ---------------------------------------------------------------- modes ---- */
typedef enum {
  UMH_GUN_MODE_FOCUS = 0u,   /* single focus on the axis at UMH_GUN_FOCUS_UM   */
  UMH_GUN_MODE_PLANE = 1u,   /* every element in phase: a plane wave           */
  UMH_GUN_MODE_VORTEX = 2u,  /* focused l = 1 vortex: hollow ring at the focus */
  UMH_GUN_MODE_BESSEL = 3u,  /* l = 1 Bessel vortex: a hollow, non-diffracting
                              * tube from a conical (axicon) wavefront         */
  UMH_GUN_MODE_BOTTLE = 4u,  /* dynamic bottle: the l = 1 tube collapses and
                              * then holds (see UMH_GUN_BOTTLE_*)             */
  UMH_GUN_MODE_VOICE = 5u,   /* record from four MEMS microphones, then AM     */
  UMH_GUN_MODE_COUNT = 6u
} umh_gun_mode_t;

/* Menu labels, in mode order.  The OLED shows 21 columns of 5x7 text. */
extern const char *const umh_gun_mode_names[UMH_GUN_MODE_COUNT];

/* Build identity of the gun implementation.  gun_trigger_init() reads it once
 * into gun_trigger_build_id_seen, and the bench probe reads that symbol back over
 * SWD: "is the new firmware really flashed?" is otherwise unanswerable, because an
 * image without the gun menu reads as a perfectly plausible idle gun state, all
 * zeroes.  The value is "GN" plus a version, bumped whenever the gun
 * implementation changes. */
uint32_t gun_trigger_build_id(void);
extern volatile uint32_t gun_trigger_build_id_seen;

/* Record 8-bit fixed mu-law at 8 kHz; decode/denoise at release before
 * converting in place to the 0..128 AM envelope.  There is no record AGC. */
#define UMH_GUN_VOICE_INPUT_RATE 16000u
#define UMH_GUN_VOICE_SAMPLE_RATE 8000u
#define UMH_GUN_VOICE_MAX_MS 2000u
#define UMH_GUN_VOICE_MAX_SAMPLES ((UMH_GUN_VOICE_SAMPLE_RATE * UMH_GUN_VOICE_MAX_MS) / 1000u)
#define UMH_GUN_VOICE_OUTPUT_RATE UMH_GUN_VOICE_SAMPLE_RATE

/* AM envelope is centred at 64 and swings by 64 slots.  That is the full
 * 0..128 carrier range (off to 50% duty); higher values would reduce the
 * 40 kHz carrier rather than increase audible output. */
#define UMH_GUN_VOICE_BIAS 64u
#define UMH_GUN_VOICE_SWING 64u
#define UMH_GUN_VOICE_PREBUFFER 128u

/* Symmetric Q15 filter before 2:1 decimation.  The duplicated history ring
 * exposes one contiguous window without per-tap wrap checks. */
#define UMH_GUN_VOICE_FIR_TAPS 31u
#define UMH_GUN_VOICE_FIR_HISTORY 32u
_Static_assert((UMH_GUN_VOICE_FIR_HISTORY & (UMH_GUN_VOICE_FIR_HISTORY - 1u)) == 0u &&
               UMH_GUN_VOICE_FIR_TAPS <= UMH_GUN_VOICE_FIR_HISTORY,
               "voice FIR history must cover all taps and be a power of two");
extern const int16_t umh_gun_voice_fir[UMH_GUN_VOICE_FIR_TAPS];

/* Envelope value of a shot, and the only intensity this module writes: it scales
 * the focused image into the frame and it scales the WS2812 brightness, so the
 * four LEDs cannot disagree with the ultrasound.  128 is the full source
 * amplitude, because spatial_renderer_finalize maps a magnitude of 1.0 to level
 * 128, the 50% carrier duty that actually launches a 40 kHz wave (255 would be
 * 99.6% DC and would leave no carrier to emit). */
#define UMH_GUN_LEVEL 128u

/* Focus distance of a shot, measured from the transducer faces.
 *
 * The aperture is the 84-element hexagonal deck (10 mm pitch, x -43..+43 mm,
 * y -45..+45 mm) and lambda is 8.575 mm at 40 kHz, so the array's own natural
 * focus -- the last axial maximum of the unfocused field -- sits at 279 mm
 * (Utiles/umh_focus_plan.py, Rayleigh integral over the measured element
 * positions).  Inside that distance every extra millimetre is bought with
 * pressure one-for-one (the focal pressure is the coherent sum, ~1/z), outside
 * it the focusing phase law buys nothing at all: at 1 m the whole array is in
 * its far field and a focus there is worth +0.3 dB over a plane wave.
 *
 * 250 mm is the value that keeps a usable working range: the on-axis pressure
 * is within 0.3 dB of the best achievable for any target between 200 and
 * 350 mm, the -6 dB focal spot is ~20 mm across and the -6 dB depth of focus is
 * ~300 mm, so the shot does not have to be aimed at an exact standoff.  The
 * price against a 150 mm focus is 2.8 dB (in exchange for 1.7x the distance),
 * and the price against the plane wave at 1 m is only 2.2 dB.
 *
 * A shot still drives all 84 channels at UMH_GUN_LEVEL, i.e. at exactly the same
 * 50 % duty a plane wave used: the emitted power is unchanged, only the
 * wavefront converges. */
#define UMH_GUN_FOCUS_UM 95000u

/* --------------------------------------------------- hollow beam modes ------
 *
 * Modes 3, 4 and 5 all radiate one helical phase field: the carrier phase is
 * wound by one full turn around the beam axis (topological charge l = 1) on top
 * of whatever wavefront the mode asks for.  The winding guarantees a pressure
 * null exactly on the axis -- the azimuthal integral of exp(j*phi) is zero for
 * any radial amplitude, whatever the array does -- so the beam is a hollow tube
 * with the particle sitting in its dark core instead of a bright axial focus.
 *
 * The null is the part the hardware guarantees.  The bright wall around it
 * lands where diffraction puts it: the aperture is 92 mm across and lambda is
 * 8.6 mm, so at the 250 mm working plane the transverse resolution is 20..90 mm
 * no matter what phase the array commands.  The numbers quoted below are the
 * Utiles/umh_gun_modes.py predictions for these 84 measured elements, not a
 * continuous-aperture idealisation. */

/* l = 1 focused vortex: the ordinary point-focus delay plus one azimuthal turn,
 * so the wavefront converges to a focal ring on the axis at
 * UMH_GUN_VORTEX_FOCUS_UM.  Focusing to a ring instead of a point is what keeps
 * the core dark; a point focus would put all the pressure back on the axis.
 * Predicted field at 250 mm: a 40 mm dark core inside a -10 dB ring at 80 mm,
 * with a -55 dB on-axis null.  The core is the diffraction limit of this
 * aperture there (the Airy radius is ~23 mm), so it is about twice the 20 mm a
 * tighter focus would give. */
#define UMH_GUN_VORTEX_FOCUS_UM 95000u
#define UMH_GUN_VORTEX_TURNS    1.0f

/* l = 1 Bessel vortex: a conical (axicon) wavefront instead of a spherical one.
 * The source phase is -k * rho * sin(beta) with rho the element's distance from
 * the axis, so every element launches its beam at the same angle beta and the
 * contributions interfere into a Bessel beam: a hollow tube whose radius is set
 * by beta and stays constant over the Bessel zone (aperture radius / tan beta)
 * instead of converging to one plane and diverging again.
 *
 * beta comes from the J1 first zero, r_core = 3.8317 / (k sin beta), the
 * relation a real axicon obeys:
 *   beta 1.35 deg -> core radius 3.8317/(0.7327*0.02356) = 222 mm and a Bessel
 *   zone of 45.9/tan(1.35 deg) = 1946 mm, i.e. non-diffracting well past the
 *   working range.
 * One wavelength of aperture phase is 0.13 rad/mm * 8.6 mm = 1.1 rad, still
 * under the pi a 10 mm element pitch can represent, so the cone is not clipped
 * by the sampling.  Predicted field at 250 mm: a -7 dB wall with the -3 dB lobe
 * out to ~25 mm and a -65 dB on-axis null. */
#define UMH_GUN_BESSEL_BETA_DEG 1.35f

/* Dynamic bottle.  The same l = 1 winding, but the convergence depth collapses
 * from UMH_GUN_BOTTLE_FAR_UM to UMH_GUN_BOTTLE_NEAR_UM over
 * UMH_GUN_BOTTLE_COLLAPSE_MS while a shot is held and then holds at the near
 * depth for as long as the switch stays down.
 *
 * The collapse starts with a long, wide tube: the waist is far away, so the
 * hollow core is broad and a particle anywhere inside it is pushed towards the
 * axis by the bright wall as the waist comes in.  The trap is that dark core,
 * and Utiles/umh_gun_modes.py measures it in each waist plane for this aperture:
 *
 *   waist depth   600 mm -> 100 mm     dark core diameter 300 mm -> 27 mm
 *   peak          -3.7 dB -> -9.5 dB below the plane wave
 *   on-axis null  -63 dB -> -59 dB at every step of the sweep
 *
 * One press is therefore a ten-to-one funnel, and both ends are the same law
 * with a different waist depth, so the null never goes away and the collapse has
 * no step a particle can fall out of.  The bright ring around the core is where
 * diffraction puts it, 80..130 mm across; unlike the core it is not a controlled
 * quantity.
 *
 * The reach is set by the aperture, not by the phase law: a 20 mm feature at a
 * 250 mm standoff is inside this array's resolution limit, which is why the
 * small end of the bottle is reached by pulling the waist in rather than by
 * squeezing the wavefront at a fixed standoff.
 *
 * The field is rebuilt every UMH_GUN_BOTTLE_PERIOD_MS and is bit-identical
 * between rebuilds, so the link's hold optimisation costs one SPI frame per step
 * instead of one per render iteration.  One press is one collapse: the envelope
 * stays at full power from the press to the release, so the trap holds at the
 * near depth until the switch opens and the particle drops.
 *
 * The collapse is advanced by the render task, on the frame clock, not by the
 * protocol task that serviced the press: the shot is one continuous emission and
 * its field has to follow wall-clock time even while the protocol task is busy
 * with USB traffic. */
#define UMH_GUN_BOTTLE_FAR_UM       600000u
#define UMH_GUN_BOTTLE_NEAR_UM      100000u
#define UMH_GUN_BOTTLE_COLLAPSE_MS  2000u
#define UMH_GUN_BOTTLE_PERIOD_MS    10u

/* Debounce samples at the input task's 10 ms cadence, to keep mechanical
 * contact bounce from writing a burst of edges into the output. */
#define UMH_GUN_DEBOUNCE_SAMPLES 3u

/* While a shot is held the render task re-sends the same image at this rate.
 * The FPGA link watchdog clears a running output after ~30 ms without chip
 * select; a held focus is static, so this keep-alive is also the repair
 * path after a host STOP cleared the output underneath the shot. */
#define UMH_GUN_REFRESH_MS 200u

/* Status poll cadence while a shot is held.  The FPGA link watchdog clears a
 * running output after ~30 ms without chip select, so the link needs one
 * transaction in that window; polling faster than this only loads the SPI bus
 * that a dynamic-bottle step is already using. */
#define UMH_GUN_POLL_MS 5u

typedef struct {
  uint8_t pressed;            /* debounced pin: 1 while the switch is closed  */
  volatile uint8_t engaged;   /* the trigger currently owns the 84 channels   */
  uint8_t raw;                /* latest raw sample of PA9 (1 = low = pressed) */
  uint8_t stable_samples;     /* debounce progress towards the raw sample     */
  uint8_t level;              /* 0 while idle, UMH_GUN_LEVEL while firing     */
  /* Mode the next press fires.  Written by the UI task, read by the protocol
   * task when it builds a shot and by the render task when it refreshes one.
   * A byte load/store is atomic on this part, and select_mode() refuses to
   * change it while a shot owns the output, so a rebuild never races a mode
   * change. */
  volatile uint8_t mode;
  /* Dynamic-bottle state.  collapse_t0_ms is the HAL tick of the shot that
   * started the current collapse and render_ms is when the field was last
   * rebuilt from it.  Both restart on every press, so each press replays the
   * collapse from the wide end. */
  uint32_t collapse_t0_ms;
  uint32_t render_ms;
  uint32_t last_poll_ms;      /* HAL tick of the last status transaction      */
  /* Levels the envelope is written into.  These are the focused levels and
   * must never be scaled in place: the envelope is applied to this copy on every
   * submit, so a repeated refresh cannot compound it. */
  uint8_t base_level[UMH_DEVICE_CHANNEL_COUNT];
  uint8_t base_ready;         /* base_level[] holds a built gun image          */
  uint8_t level_sent;         /* level the last accepted frame carried        */
  /* Previous channel image, kept so a failed dynamic-bottle rebuild can be
   * rolled back instead of driving the array with a half-built frame.  It is a
   * struct member rather than a local because the render task's grant is 4 KiB
   * and this is 168 B of it. */
  umh_output_channel_t base_prev[UMH_DEVICE_CHANNEL_COUNT];
  uint32_t fires;             /* press edges that started a shot              */
  uint32_t deferred;          /* press edges refused, e.g. during calibration */
  uint32_t last_submit_ms;    /* HAL tick of the last FRAME transaction       */
  uint32_t sequence;          /* last frame sequence handed to the FPGA       */
  umh_output_frame_t frame;   /* the held gun image                           */
  /* Per-channel complex accumulation, shared with spatial_renderer_finalize.
   * Keeping them here instead of on the protocol task stack holds that task's
   * worst chain below its 3 KiB grant (see the stack table in app_freertos.c). */
  float real_accum[UMH_DEVICE_CHANNEL_COUNT];
  float imag_accum[UMH_DEVICE_CHANNEL_COUNT];
  int16_t azimuth_q10[UMH_DEVICE_CHANNEL_COUNT];
} umh_gun_trigger_t;

void gun_trigger_init(umh_gun_trigger_t *trigger);
/* Input task, one call per 10 ms scan.  Only reads the pin and advances one
 * debounce counter, so the 512 B input task stack is enough. */
void gun_trigger_sample(umh_gun_trigger_t *trigger);
uint8_t gun_trigger_pressed(const umh_gun_trigger_t *trigger);
uint8_t gun_trigger_owns_output(const umh_gun_trigger_t *trigger);
/* 1 from the moment the takeover's emit() has built the gun image until the
 * release.  The render task gates the shot branch on this rather than on
 * owns_output(), because between the claim and the emit the frame still holds
 * the previous producer's channels. */
uint8_t gun_trigger_holds_image(const umh_gun_trigger_t *trigger);
/* Gun menu mode selection.  select_mode() is the row-cycling path: it raises
 * mode_busy, refuses while a shot owns the output (returns -1 and the UI keeps
 * the previous mode), and clears the flag once the mode is stored.  set_mode()
 * is the unconditional write the UI uses when it enters or leaves the menu. */
int gun_trigger_select_mode(umh_gun_trigger_t *trigger, uint8_t mode);
void gun_trigger_set_mode(umh_gun_trigger_t *trigger, uint8_t mode);
uint8_t gun_trigger_mode(const umh_gun_trigger_t *trigger);
/* Claim the output before the producer takeover, so the render task stops
 * pulling frames out of the ring while the takeover resets it. */
void gun_trigger_claim_output(umh_gun_trigger_t *trigger);
/* Undo a claim whose takeover failed; the render task resumes its normal path. */
void gun_trigger_drop_output(umh_gun_trigger_t *trigger);
void gun_trigger_note_deferred(umh_gun_trigger_t *trigger);
/* Build and submit the selected gun mode at UMH_GUN_LEVEL.  Fails, without
 * owning anything, when the switch was already released or the FPGA did not
 * accept the frame. */
int gun_trigger_emit(umh_gun_trigger_t *trigger, umh_spatial_renderer_t *renderer,
                     fpga_link_t *link);
/* Release edge: full power to 0 and STOP, in one call. */
int gun_trigger_release(umh_gun_trigger_t *trigger, fpga_link_t *link);
/* Render task service: keep the held image alive, advance the dynamic-bottle
 * collapse, self-heal a cleared output.  Returns the status poll result, or 0
 * when no shot is held. */
int gun_trigger_refresh(umh_gun_trigger_t *trigger, umh_spatial_renderer_t *renderer,
                        fpga_link_t *link);

#endif
