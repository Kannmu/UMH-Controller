#ifndef GUN_VOICE_DSP_H
#define GUN_VOICE_DSP_H

#include <stdint.h>

/* Voice recording storage and the release-time denoiser.
 *
 * 8 kHz throughout.  The recording lives in SRAM1 so the 16 KiB CCM block is
 * free for the render path's hot data.  A fixed differential code keeps the
 * storage in 8..10 KiB for the full two-second shot; the waveform is decoded
 * on playback, so nothing else in the AM envelope path changes.
 *
 * GUN_VOICE_CODEC_VERSION is the storage-format tag.  Bump it (and re-check
 * the buffer size) whenever the bit layout or the quantiser changes, so a
 * recording or a host-side decoder from an older build cannot be mistaken for
 * the current format. */

#define GUN_VOICE_SAMPLE_RATE_HZ 8000u
#define GUN_VOICE_MAX_SAMPLES    16000u   /* 2 s at 8 kHz */
#define GUN_VOICE_HOP            64u

#define GUN_VOICE_CODEC_MULAW8   1u
#define GUN_VOICE_CODEC_ADPCM4   2u
#define GUN_VOICE_CODEC_ADPCM5   3u

/* Default: 5-bit IMA ADPCM, 10 000 B for 2 s (3.2:1).  Override with
 * -DGUN_VOICE_CODEC=GUN_VOICE_CODEC_ADPCM4 for the 8 000 B 4:1 form, or
 * GUN_VOICE_CODEC_MULAW8 for the uncompressed 1 s fallback. */
#ifndef GUN_VOICE_CODEC
#define GUN_VOICE_CODEC GUN_VOICE_CODEC_ADPCM5
#endif

#if GUN_VOICE_CODEC == GUN_VOICE_CODEC_ADPCM5
#define GUN_VOICE_BITS_PER_SAMPLE 5u
#define GUN_VOICE_CODEC_VERSION   3u
#define GUN_VOICE_STORAGE_SAMPLES GUN_VOICE_MAX_SAMPLES
#elif GUN_VOICE_CODEC == GUN_VOICE_CODEC_ADPCM4
#define GUN_VOICE_BITS_PER_SAMPLE 4u
#define GUN_VOICE_CODEC_VERSION   2u
#define GUN_VOICE_STORAGE_SAMPLES GUN_VOICE_MAX_SAMPLES
#elif GUN_VOICE_CODEC == GUN_VOICE_CODEC_MULAW8
#define GUN_VOICE_BITS_PER_SAMPLE 8u
#define GUN_VOICE_CODEC_VERSION   1u
#define GUN_VOICE_STORAGE_SAMPLES (GUN_VOICE_MAX_SAMPLES / 2u)
#else
#error "unknown GUN_VOICE_CODEC"
#endif

#define GUN_VOICE_BUFFER_BYTES \
  ((GUN_VOICE_STORAGE_SAMPLES * GUN_VOICE_BITS_PER_SAMPLE) / 8u)

#define GUN_VOICE_WORK_BYTES 2048u
#define GUN_VOICE_PROCESS_STEPS 16u

/* 8-bit mu-law companding; the fallback codec and the host tools use it. */
uint8_t gun_voice_encode(int32_t pcm);
int16_t gun_voice_decode(uint8_t code);

/* One packing + quantiser instance.  The same struct serves the capture
 * encoder, the streaming reader inside the denoiser and the player; only the
 * direction of the code path differs.  acc/bits hold the partially written
 * byte, low bits first. */
typedef struct {
  int16_t predict;
  uint16_t byte_pos;
  int8_t index;
  uint8_t bits;
  uint32_t acc;
} gun_voice_codec_t;

void gun_voice_codec_reset(gun_voice_codec_t *codec);
void gun_voice_codec_push(gun_voice_codec_t *codec, uint8_t *buffer, int32_t pcm);
void gun_voice_codec_flush(gun_voice_codec_t *codec, uint8_t *buffer);
int16_t gun_voice_codec_pull(gun_voice_codec_t *codec, const uint8_t *buffer,
                             uint16_t byte_limit);

typedef struct {
  gun_voice_codec_t codec;
  const uint8_t *data;
  uint16_t byte_limit;
  uint16_t total;
  uint16_t position;
  float gain;
} gun_voice_player_t;

/* The render task lends the inactive aperture envelope ring to this processor.
 * It must keep ownership of the output until processing finishes.  No heap,
 * second recording, DMA, hardware peripheral, or protocol state is required. */
typedef struct {
  uint8_t *samples;              /* packed capture stream in SRAM1 */
  void *workspace;
  gun_voice_codec_t dec;         /* current read pass */
  gun_voice_codec_t enc;         /* in-place rewrite during denoise */
  int16_t window[GUN_VOICE_HOP]; /* previous hop for the 50 % overlapped FFT */
  int16_t fresh[GUN_VOICE_HOP];
  uint16_t count;
  uint16_t byte_limit;
  uint16_t speech_frames;
  uint16_t noise_frames;
  uint16_t active_samples;
  int32_t position;
  uint32_t noise_rms;
  uint32_t noise_ceiling;
  float noise_power;
  float fallback_power;
  float speech_square_sum;
  float speech_rms;
  float playback_gain;
  float gate;
  uint8_t stage;
  uint8_t speech_run;
  uint8_t speech_run_max;
  uint8_t voice_hold;
  uint8_t codec_version;
} gun_voice_processor_t;

/* 0 = still working, 1 = envelope ready, -1 = no sufficiently clear speech.
 * Every service call has a fixed bound; only this mode uses the workspace. */
void gun_voice_process_begin(gun_voice_processor_t *processor, uint8_t *samples,
                             uint16_t count, void *workspace);
int gun_voice_process_service(gun_voice_processor_t *processor);

/* Decode the finished recording into 8-bit AM envelope values at 8 kHz.  The
 * gain is the processor's playback_gain. */
void gun_voice_player_open(gun_voice_player_t *player, const uint8_t *data,
                           uint16_t samples, float gain);
uint16_t gun_voice_player_read(gun_voice_player_t *player, uint8_t *out,
                               uint16_t capacity);

#endif
