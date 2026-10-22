#ifndef UMH_USB_AUDIO_DSP_H
#define UMH_USB_AUDIO_DSP_H

#include <stdint.h>

#define USB_AUDIO_DSP_TAPS 48u
#define USB_AUDIO_DSP_OUTPUT_MAX 20u
#define USB_AUDIO_DSP_PEAK_WINDOW 100u /* 5 ms at 20 kHz, matches the host */
#define USB_AUDIO_DSP_LOOKAHEAD 96u    /* 4.8 ms look-ahead limiter */

typedef struct {
  /* Mirrored history makes every FIR dot product contiguous (no modulo in the
   * inner loop).  Coefficients live in Flash; history is 192 bytes. */
  int16_t history[2u * USB_AUDIO_DSP_TAPS];
  /* The dynamic stages below are a direct float port of UMH Speaker's
   * AudioSignalProcessor limiter and ParametricAmModulator AGC.  They run on
   * the Cortex-M4F FPU; the previous Q16/uint16 version had a different
   * release curve and partial-window gain target, which only showed up on
   * real programme material, never on a steady tone. */
  float peak_window[USB_AUDIO_DSP_PEAK_WINDOW];
  float dc_state;
  float previous_input;
  float limiter_gain;
  float agc_gain;
  float quant_error;   /* first-order error feedback on the 8-bit level */
  uint16_t silent_samples;
  uint16_t peak_window_pos;
  uint16_t peak_window_count;
  uint16_t lookahead_pos;
  uint8_t head;
  uint8_t phase;
  uint8_t gate;
} usb_audio_dsp_t;

void usb_audio_dsp_reset(usb_audio_dsp_t *dsp);
/* The output capacity must be >= ceil(input_count * 5/12).  This routine
 * preserves fractional phase across arbitrary USB packets and never allocates.
 * gain_q15 is retained for API compatibility; the USB endpoint volume is not
 * applied here because it would scale the 8-bit envelope after the AGC. */
uint16_t usb_audio_dsp_process(usb_audio_dsp_t *dsp, const int16_t *pcm,
                               uint16_t input_count, uint16_t gain_q15,
                               uint8_t *envelope);

#endif
