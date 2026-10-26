#ifndef UMH_USB_AUDIO_H
#define UMH_USB_AUDIO_H

#include <stdint.h>

/* UAC1 adaptive playback sink.  The host mixes to mono at 48 kHz; the existing
 * envelope clock servo adapts the 20 kHz stream to the independent FPGA clock.
 *
 * The endpoint advertises exactly one nominal frame (48 samples / 96 bytes).
 * The G4 isochronous OUT PMA buffer is sized from wMaxPacketSize and the core
 * does not reprogram the RX count on every re-arm, so advertising 98 for a
 * 49-sample adaptive frame left the hardware buffer one 32-byte block short
 * of the measured length; a host that ever sends the extra sample loses two
 * bytes in the middle of a frame.  Clock recovery belongs to the envelope
 * servo, so the nominal 96-byte frame is the safe contract. */
#define UMH_USB_AUDIO_RATE 48000u
#define UMH_USB_AUDIO_PACKET_MAX 96u /* one 48-sample frame, exactly */
#define UMH_USB_AUDIO_RING_SAMPLES 128u /* 2.7 ms, power of two */
#define UMH_USB_AUDIO_BLOCK_SAMPLES 48u
/* A 1 ms host packet and a 5.3 ms input ring must survive far more than the
 * old 12 ms before the aperture engine is told the stream died; a brief USB
 * scheduling gap must not release and restart the output. */
#define UMH_USB_AUDIO_TIMEOUT_MS 40u
#define UMH_USB_AUDIO_BUILD_ID 0x5541431Du

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
  uint16_t short_packets;   /* isoc frames whose length != 96 */
  uint16_t zero_packets;    /* isoc frames with length 0 */
  uint32_t max_packet_delta; /* max |s[i]-s[i-1]| inside one frame */
  uint32_t bad_delta_packets; /* frames with a >4000 LSB intra-frame step */
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



