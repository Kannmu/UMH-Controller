#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define WAVEFORM_TICKS 100
#define MAX_CHANNEL_PINS 16
#define TEST_CASES 100000

typedef struct
{
    uint16_t pin;
    uint16_t start;
    uint16_t duty;
} Pulse;

static uint32_t random_state = 0x7505A17U;

static uint32_t Next_Random(void)
{
    uint32_t x = random_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    random_state = x;
    return x;
}

static void Render_Legacy(const Pulse *pulses, int count, uint16_t initial_state,
                          uint16_t port_mask, uint16_t output[WAVEFORM_TICKS])
{
    uint16_t turn_on[WAVEFORM_TICKS] = {0};
    uint16_t turn_off[WAVEFORM_TICKS] = {0};
    uint16_t event_indices[MAX_CHANNEL_PINS * 2 + 2];
    int event_count = 0;
    uint16_t current_state = initial_state;

    event_indices[event_count++] = 0;
    for (int i = 0; i < count; i++)
    {
        if (pulses[i].duty == 0U)
        {
            continue;
        }

        uint16_t start = pulses[i].start;
        uint16_t end = (uint16_t)((start + pulses[i].duty) % WAVEFORM_TICKS);
        if (turn_on[start] == 0U && turn_off[start] == 0U)
        {
            event_indices[event_count++] = start;
        }
        turn_on[start] |= pulses[i].pin;

        if (turn_on[end] == 0U && turn_off[end] == 0U)
        {
            event_indices[event_count++] = end;
        }
        turn_off[end] |= pulses[i].pin;

        if (start >= end)
        {
            current_state |= pulses[i].pin;
        }
    }

    for (int i = 1; i < event_count; i++)
    {
        uint16_t key = event_indices[i];
        int j = i - 1;
        while (j >= 0 && event_indices[j] > key)
        {
            event_indices[j + 1] = event_indices[j];
            j--;
        }
        event_indices[j + 1] = key;
    }

    uint16_t running_state = current_state | port_mask;
    int previous = 0;
    for (int i = 0; i < event_count; i++)
    {
        int index = event_indices[i];
        if (i > 0 && index == event_indices[i - 1])
        {
            continue;
        }

        for (int tick = previous; tick < index; tick++)
        {
            output[tick] = running_state;
        }

        running_state |= turn_on[index];
        running_state &= (uint16_t)~turn_off[index];
        running_state |= port_mask;
        previous = index;
    }

    for (int tick = previous; tick < WAVEFORM_TICKS; tick++)
    {
        output[tick] = running_state;
    }
}

static void Render_Optimized(const Pulse *pulses, int count, uint16_t initial_state,
                             uint16_t port_mask, uint16_t output[WAVEFORM_TICKS])
{
    uint16_t turn_on[WAVEFORM_TICKS] = {0};
    uint16_t turn_off[WAVEFORM_TICKS] = {0};
    uint16_t running_state = initial_state;

    for (int i = 0; i < count; i++)
    {
        if (pulses[i].duty == 0U)
        {
            continue;
        }

        uint16_t start = pulses[i].start;
        uint16_t end = (uint16_t)((start + pulses[i].duty) % WAVEFORM_TICKS);
        turn_on[start] |= pulses[i].pin;
        turn_off[end] |= pulses[i].pin;
        if (start >= end)
        {
            running_state |= pulses[i].pin;
        }
    }

    for (int tick = 0; tick < WAVEFORM_TICKS; tick++)
    {
        running_state |= turn_on[tick];
        running_state &= (uint16_t)~turn_off[tick];
        output[tick] = running_state | port_mask;
    }
}

int main(void)
{
    Pulse pulses[MAX_CHANNEL_PINS];
    uint16_t legacy[WAVEFORM_TICKS];
    uint16_t optimized[WAVEFORM_TICKS];

    for (int test = 0; test < TEST_CASES; test++)
    {
        int count = (int)(Next_Random() % (MAX_CHANNEL_PINS + 1U));
        for (int i = 0; i < count; i++)
        {
            pulses[i].pin = (uint16_t)(1U << i);
            pulses[i].start = (uint16_t)(Next_Random() % WAVEFORM_TICKS);
            pulses[i].duty = (uint16_t)(Next_Random() % 51U);
        }

        uint16_t initial_state = (uint16_t)(Next_Random() & 0x8000U);
        uint16_t port_mask = (uint16_t)(Next_Random() & 0x0700U);
        Render_Legacy(pulses, count, initial_state, port_mask, legacy);
        Render_Optimized(pulses, count, initial_state, port_mask, optimized);

        if (memcmp(legacy, optimized, sizeof(legacy)) != 0)
        {
            for (int tick = 0; tick < WAVEFORM_TICKS; tick++)
            {
                if (legacy[tick] != optimized[tick])
                {
                    fprintf(stderr,
                            "Mismatch in case %d at tick %d: legacy=%04x optimized=%04x\n",
                            test, tick, legacy[tick], optimized[tick]);
                    return 1;
                }
            }
        }
    }

    const float pi = 3.14159265358979323846f;
    double maximum_tick_error = 0.0;
    for (uint32_t raw_phase = 0; raw_phase <= 65535U; raw_phase++)
    {
        float phase = ((float)raw_phase / 65535.0f) * (2.0f * pi);
        float optimized_ticks = phase * (100.0f / (2.0f * pi));
        double reference_ticks = (double)phase * 100.0 /
                                 (2.0 * 3.14159265358979323846);
        double error = fabs((double)optimized_ticks - reference_ticks);
        if (error > maximum_tick_error)
        {
            maximum_tick_error = error;
        }
    }

    if (maximum_tick_error >= 1.0e-4)
    {
        fprintf(stderr, "phase conversion error too large: %.9f ticks\n",
                maximum_tick_error);
        return 1;
    }

    printf("waveform equivalence: %d randomized cases passed; max phase error %.9f ticks\n",
           TEST_CASES, maximum_tick_error);
    return 0;
}
