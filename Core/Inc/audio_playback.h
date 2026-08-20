#pragma once

#include <stdint.h>

#define AUDIO_SAMPLE_RATE       48000U
#define AUDIO_OUTPUT_RATE       40000U
#define AUDIO_CHANNELS          1U
#define AUDIO_BITS_PER_SAMPLE   16U
#define AUDIO_RING_CAPACITY     4096U
#define AUDIO_PREBUFFER_SAMPLES 960U

typedef enum
{
    AUDIO_STATE_LEGACY = 0,
    AUDIO_STATE_PREPARE,
    AUDIO_STATE_RUNNING,
    AUDIO_STATE_DRAINING,
    AUDIO_STATE_FAULT
} AudioState;

typedef struct __attribute__((packed))
{
    uint8_t state;
    uint8_t muted;
    uint16_t ring_fill;
    uint32_t underrun_count;
    uint32_t overrun_count;
    uint32_t usb_reset_count;
    uint32_t packet_loss_count;
    int32_t clock_correction_ppm;
} AudioStatus;

void Audio_Init(void);
void Audio_Task(void);
int Audio_Enter(void);
void Audio_Exit(void);
int Audio_Is_Active(void);
AudioState Audio_Get_State(void);
int Audio_Set_Focus(const float position[3]);
void Audio_Set_Level(float level);
void Audio_Set_Mute(int mute);
void Audio_Push_PCM(const uint8_t *data, uint32_t length);
void Audio_Get_Status(AudioStatus *status);
