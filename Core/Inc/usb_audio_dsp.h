#ifndef UMH_USB_AUDIO_DSP_H
#define UMH_USB_AUDIO_DSP_H

#include <stdint.h>

#define USB_AUDIO_DSP_TAPS 48u
#define USB_AUDIO_DSP_OUTPUT_MAX 20u
#define USB_AUDIO_DSP_PEAK_WINDOW 100u /* 5 ms at 20 kHz */

typedef struct {
  /* Mirrored history makes every FIR dot product contiguous (no modulo in the
   * inner loop).  Coefficients live in Flash; history is 192 bytes. */
  int16_t history[2u * USB_AUDIO_DSP_TAPS];
  uint16_t peak_window[USB_AUDIO_DSP_PEAK_WINDOW];
  int32_t dc_q8;
  uint32_t normalization_gain_q16;
  uint16_t silent_samples;
  uint16_t peak_window_pos;
  uint16_t peak_window_count;
  uint8_t head;
  uint8_t phase;
  uint8_t gate;
} usb_audio_dsp_t;

void usb_audio_dsp_reset(usb_audio_dsp_t *dsp);
/* The output capacity must be >= ceil(input_count * 5/12).  This routine
 * preserves fractional phase across arbitrary USB packets and never allocates.
 * gain_q15 scales the entire AM envelope, including its carrier bias. */
uint16_t usb_audio_dsp_process(usb_audio_dsp_t *dsp, const int16_t *pcm,
                               uint16_t input_count, uint16_t gain_q15,
                               uint8_t *envelope);

#endif
