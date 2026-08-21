#pragma once

#include <stdint.h>

#define SEQUENCE_GET_CAPS       0x20U
#define SEQUENCE_BEGIN          0x21U
#define SEQUENCE_UPLOAD         0x22U
#define SEQUENCE_COMMIT         0x23U
#define SEQUENCE_DATA           0x24U
#define SEQUENCE_GET_STATUS     0x25U

#define SEQUENCE_INPUT_RATE             40000U
#define SEQUENCE_OUTPUT_RATE            40000U
#define SEQUENCE_BLOCK_SAMPLES          200U
#define SEQUENCE_PREBUFFER_SAMPLES      2400U
#define SEQUENCE_RING_CAPACITY          4096U
#define SEQUENCE_MAX_STATES             100U
#define SEQUENCE_RENDER_STATES          SEQUENCE_MAX_STATES
#define SEQUENCE_PHASE_TICKS            100U
#define SEQUENCE_OUTPUT_CHANNELS        60U
#define SEQUENCE_PACKET_SAMPLES         120U
#define SEQUENCE_MAX_UPLOAD_STATES      4U

typedef enum
{
    SEQUENCE_MAPPING_ABSOLUTE_CLAMPED = 0,
    SEQUENCE_MAPPING_CYCLIC_INCREMENT = 1,
} SequenceMapping;

typedef enum
{
    SEQUENCE_STATE_IDLE = 0,
    SEQUENCE_STATE_CONFIGURING = 1,
    SEQUENCE_STATE_BUFFERING = 2,
    SEQUENCE_STATE_RUNNING = 3,
    SEQUENCE_STATE_HOLD = 4,
    SEQUENCE_STATE_FAULT = 5,
} SequenceState;

typedef struct __attribute__((packed))
{
    uint32_t input_rate;
    uint32_t output_rate;
    uint16_t block_samples;
    uint16_t prebuffer_samples;
    uint16_t max_states;
    uint8_t phase_ticks;
    uint8_t output_channels;
} SequenceCapabilities;

typedef struct __attribute__((packed))
{
    uint16_t state_count;
    uint8_t mapping;
    uint8_t neutral_state;
    int32_t control_scale_q16;
    float focus[3];
} SequenceDescriptor;

typedef struct __attribute__((packed))
{
    uint8_t state;
    uint8_t mapping;
    uint16_t ring_fill;
    uint32_t underrun_count;
    uint32_t overrun_count;
    uint32_t packet_loss_count;
    uint32_t comm_rx_dropped_bytes;
    uint32_t render_timeout_count;
    int32_t clock_correction_ppm;
    uint32_t current_render_us;
    uint32_t maximum_render_us;
    /* Optional diagnostics appended after the original 36-byte status. */
    uint32_t rendered_block_count;
    uint32_t render_over_budget_count;
    uint32_t dma_deadline_miss_count;
    uint32_t render_mode;
} SequenceStatus;

_Static_assert(sizeof(SequenceCapabilities) == 16U, "sequence capabilities wire size");
_Static_assert(sizeof(SequenceDescriptor) == 20U, "sequence descriptor wire size");
_Static_assert(sizeof(SequenceStatus) == 52U, "sequence status wire size");
