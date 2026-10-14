#ifndef GUN_VOICE_DSP_H
#define GUN_VOICE_DSP_H

#include <stdint.h>

/* Fixed companding keeps the two-second recording in 16 KiB without changing
 * the gain with time.  Code 0xff is zero; codes are not unsigned linear PCM. */
uint8_t gun_voice_encode(int32_t pcm);
int16_t gun_voice_decode(uint8_t code);

/* The render task lends the inactive aperture envelope ring to this processor.
 * It must keep ownership of the output until processing finishes.  No heap,
 * second recording, DMA, hardware peripheral, or protocol state is required. */
#define GUN_VOICE_WORK_BYTES 2048u
#define GUN_VOICE_PROCESS_STEPS 16u

typedef struct {
  uint8_t *samples;
  void *workspace;
  uint16_t count;
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
} gun_voice_processor_t;

/* 0 = still working, 1 = envelope ready, -1 = no sufficiently clear speech.
 * Every service call has a fixed bound; only this mode uses the workspace. */
void gun_voice_process_begin(gun_voice_processor_t *processor, uint8_t *samples,
                             uint16_t count, void *workspace);
int gun_voice_process_service(gun_voice_processor_t *processor);

#endif
