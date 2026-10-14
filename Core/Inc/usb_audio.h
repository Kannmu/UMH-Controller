#ifndef UMH_USB_AUDIO_H
#define UMH_USB_AUDIO_H

#include <stdint.h>

/* UAC1 adaptive playback sink.  The host mixes to mono at 48 kHz; the existing
 * envelope clock servo adapts the 20 kHz stream to the independent FPGA clock. */
#define UMH_USB_AUDIO_RATE 48000u
#define UMH_USB_AUDIO_PACKET_MAX 98u /* allow 49 samples in an adaptive frame */
#define UMH_USB_AUDIO_RING_SAMPLES 256u
#define UMH_USB_AUDIO_BLOCK_SAMPLES 48u
#define UMH_USB_AUDIO_TIMEOUT_MS 12u
#define UMH_USB_AUDIO_BUILD_ID 0x55414301u

typedef struct {
  uint32_t build_id;
  uint32_t packets;
  uint32_t samples;
  uint32_t dropped_samples;
  uint32_t bad_packets;
  uint32_t processed_samples;
  uint32_t envelope_samples;
  uint32_t starts;
  uint32_t stops;
  uint32_t max_isr_cycles;
  uint32_t max_process_cycles;
  uint16_t queued_samples;
  uint8_t owns_output;
  uint8_t inhibited;
  uint8_t level_min;
  uint8_t level_max;
} umh_usb_audio_stats_t;

extern volatile umh_usb_audio_stats_t umh_usb_audio_stats;
/* SWD bench interlock: setting this to 1 prevents only UAC from driving output.
 * It defaults to 0, needs no host application, and never gates legacy modes. */
extern volatile uint8_t umh_usb_audio_debug_inhibit;

/* USB interrupt context, bounded copies only; no allocation, DSP or SPI. */
void umh_usb_audio_init(void);
void umh_usb_audio_set_stream(uint8_t enabled);
void umh_usb_audio_receive(const uint8_t *data, uint16_t length);
void umh_usb_audio_set_mute(uint8_t mute);
void umh_usb_audio_set_volume(int16_t volume_db256);
uint8_t umh_usb_audio_mute(void);
int16_t umh_usb_audio_volume(void);
uint16_t umh_usb_audio_gain_q15(void);
uint8_t umh_usb_audio_streaming(void);
uint32_t umh_usb_audio_generation(void);
uint32_t umh_usb_audio_last_packet_ms(void);

/* Protocol task context. */
uint16_t umh_usb_audio_read(int16_t *samples, uint16_t capacity);
void umh_usb_audio_discard(void);

#endif
