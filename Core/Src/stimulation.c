#define _USE_MATH_DEFINES
// #include <math.h>
#include "stimulation.h"
#include "calibration.h"
#include "utiles.h"
#include "custom_math.h"
#include "dma_manager.h"
#include "indexed_linear.h"
#include "audio_playback.h"

_Static_assert(NUM_STIMULATION_SAMPLES == INDEXED_LINEAR_SAMPLE_COUNT,
               "IndexedLinear protocol requires exactly 200 DMA samples");

int phase_set_mode = 0;
int is_stimulation_enabled = 1;

int demo_mode = -1;

Stimulation CurrentStimulation = {
    .name = "Current",
    .type = Point,
    .position = {0.0f, 0.0f, 0.1f},
    .strength = 100,
    .startPoint = {0.0f, 0.0f, 0.1f},
    .endPoint = {0.0f, 0.0f, 0.1f},
    .segments = 1,
    .normalVector = {0.0f, 0.0f, 0.0f},
    .radius = 0.0f,
    .frequency = 200.0f,
    .cached_period_us = 0,
    .cached_circ_u = {0.0f, 0.0f, 0.0f},
    .cached_circ_v = {0.0f, 0.0f, 0.0f},
};

Stimulation EmptyStimulation = {
    .name = "Empty",
    .type = Point,
    .position = {0.0f, 0.0f, 0.1f},
    .strength = 100,
    .startPoint = {0.0f, 0.0f, 0.1f},
    .endPoint = {0.0f, 0.0f, 0.1f},
    .segments = 0,
    .normalVector = {0.0f, 0.0f, 0.0f},
    .radius = 0.0f,
    .frequency = 0.0f,
    .cached_period_us = 0,
    .cached_circ_u = {0.0f, 0.0f, 0.0f},
    .cached_circ_v = {0.0f, 0.0f, 0.0f},
};

static float unit_circle_cos[NUM_STIMULATION_SAMPLES];
static float unit_circle_sin[NUM_STIMULATION_SAMPLES];
static uint8_t unit_circle_ready;
static int cached_discrete_segments = -1;
static int cached_discrete_index = -1;
static float cached_discrete_cos;
static float cached_discrete_sin;
static uint8_t indexed_linear_order[INDEXED_LINEAR_SAMPLE_COUNT];

static void Ensure_Unit_Circle_Table(void)
{
    if (unit_circle_ready)
    {
        return;
    }

    for (uint32_t i = 0; i < NUM_STIMULATION_SAMPLES; i++)
    {
        float progress = (float)i / (float)NUM_STIMULATION_SAMPLES;
        float angle = 2.0f * (float)M_PI * progress;
        unit_circle_cos[i] = cosf(angle);
        unit_circle_sin[i] = sinf(angle);
    }

    __DMB();
    unit_circle_ready = 1U;
}

const Stimulation DemoPointStimulation = {
    .name = "Point",
    .type = Point,
    .position = {0.0f, 0.0f, 0.1f},
    .strength = 100,
    .startPoint = {0.0f, 0.0f, 0.1f},
    .endPoint = {0.0f, 0.0f, 0.1f},
    .segments = 1,
    .normalVector = {0.0f, 0.0f, 1.0f},
    .radius = 0.0f,
    .frequency = 200.0f,
    .cached_period_us = 0,
    .cached_circ_u = {0.0f, 0.0f, 0.0f},
    .cached_circ_v = {0.0f, 0.0f, 0.0f},
};

const Stimulation DLM_2_Stimulation = {
    .name = "DLM_2",
    .type = Discrete,
    .position = {0.0f, 0.0f, 0.1f},
    .strength = 100,
    .startPoint = {0.0f, 0.0f, 0.1f},
    .endPoint = {0.0f, -0.0f, 0.1f},
    .segments = 2,
    .normalVector = {0.0f, 0.0f, 1.0f},
    .radius = 6.25e-3,
    .frequency = 200.0f,
    .cached_period_us = 0,
    .cached_circ_u = {0.0f, 0.0f, 0.0f},
    .cached_circ_v = {0.0f, 0.0f, 0.0f},
};

const Stimulation DLM_3_Stimulation = {
    .name = "DLM_3",
    .type = Discrete,
    .position = {0.0f, 0.0f, 0.1f},
    .strength = 100,
    .startPoint = {0.0f, 0.0f, 0.1f},
    .endPoint = {0.0f, -0.0f, 0.1f},
    .segments = 3,
    .normalVector = {0.0f, 0.0f, 1.0f},
    .radius = 4.81e-3,
    .frequency = 200.0f,
    .cached_period_us = 0,
    .cached_circ_u = {0.0f, 0.0f, 0.0f},
    .cached_circ_v = {0.0f, 0.0f, 0.0f},
};

const Stimulation DemoULM_LStimulation = {
    .name = "ULM_L",
    .type = Linear,
    .position = {0.0f, 0.0f, 0.1f},
    .strength = 100,
    .startPoint = {0.0f, 0.015f, 0.1f},
    .endPoint = {0.0f, -0.015f, 0.1f},
    .segments = 1,
    .normalVector = {0.0f, 0.0f, 1.0f},
    .radius = 0.0f,
    .frequency = 200.0f,
    .cached_period_us = 0,
    .cached_circ_u = {0.0f, 0.0f, 0.0f},
    .cached_circ_v = {0.0f, 0.0f, 0.0f},
};

const Stimulation DemoULM_L_M02_Stimulation = {
    .name = "ULM_L_M0.2",
    .type = Linear,
    .position = {0.0f, 0.0f, 0.1f},
    .strength = 100,
    .startPoint = {0.0f, 2.5e-3f, 0.1f},
    .endPoint = {0.0f, -2.5e-3f, 0.1f},
    .segments = 1,
    .normalVector = {0.0f, 0.0f, 1.0f},
    .radius = 0.0f,
    .frequency = 200.0f,
    .cached_period_us = 0,
    .cached_circ_u = {0.0f, 0.0f, 0.0f},
    .cached_circ_v = {0.0f, 0.0f, 0.0f},
};

const Stimulation DemoULM_L_M05_Stimulation = {
    .name = "ULM_L_M0.5",
    .type = Linear,
    .position = {0.0f, 0.0f, 0.1f},
    .strength = 100,
    .startPoint = {0.0f, 6.25e-3f, 0.1f},
    .endPoint = {0.0f, -6.25e-3f, 0.1f},
    .segments = 1,
    .normalVector = {0.0f, 0.0f, 1.0f},
    .radius = 0.0f,
    .frequency = 200.0f,
    .cached_period_us = 0,
    .cached_circ_u = {0.0f, 0.0f, 0.0f},
    .cached_circ_v = {0.0f, 0.0f, 0.0f},
};

const Stimulation DemoULM_L_M10_Stimulation = {
    .name = "ULM_L_M1.0",
    .type = Linear,
    .position = {0.0f, 0.0f, 0.1f},
    .strength = 100,
    .startPoint = {0.0f, 12.5e-3f, 0.1f},
    .endPoint = {0.0f, -12.5e-3f, 0.1f},
    .segments = 1,
    .normalVector = {0.0f, 0.0f, 1.0f},
    .radius = 0.0f,
    .frequency = 200.0f,
    .cached_period_us = 0,
    .cached_circ_u = {0.0f, 0.0f, 0.0f},
    .cached_circ_v = {0.0f, 0.0f, 0.0f},
};

const Stimulation DemoULM_L_M20_Stimulation = {
    .name = "ULM_L_M2.0",
    .type = Linear,
    .position = {0.0f, 0.0f, 0.1f},
    .strength = 100,
    .startPoint = {0.0f, 25e-3f, 0.1f},
    .endPoint = {0.0f, -25e-3f, 0.1f},
    .segments = 1,
    .normalVector = {0.0f, 0.0f, 1.0f},
    .radius = 0.0f,
    .frequency = 200.0f,
    .cached_period_us = 0,
    .cached_circ_u = {0.0f, 0.0f, 0.0f},
    .cached_circ_v = {0.0f, 0.0f, 0.0f},
};

const Stimulation DemoLM_LStimulation = {
    .name = "LM_L",
    .type = Linear,
    .position = {0.0f, 0.0f, 0.1f},
    .strength = 100,
    .startPoint = {0.0f, 7.5e-3, 0.1f},
    .endPoint = {0.0f, -7.5e-3, 0.1f},
    .segments = 2,
    .normalVector = {0.0f, 0.0f, 1.0f},
    .radius = 0.0f,
    .frequency = 200.0f,
    .cached_period_us = 0,
    .cached_circ_u = {0.0f, 0.0f, 0.0f},
    .cached_circ_v = {0.0f, 0.0f, 0.0f},
};

const Stimulation DemoLM_CStimulation = {
    .name = "LM_C",
    .type = Circular,
    .position = {0.0f, 0.0f, 0.1f},
    .strength = 100,
    .startPoint = {0.0f, 0.0f, 0.1f},
    .endPoint = {0.0f, 0.0f, 0.1f},
    .normalVector = {0.0f, 0.0f, 1.0f},
    .radius = 4.77e-3f,
    .frequency = 200.0f,
    .cached_period_us = 0,
    .cached_circ_u = {0.0f, 0.0f, 0.0f},
    .cached_circ_v = {0.0f, 0.0f, 0.0f},
};

const Stimulation DemoTwinTrapStimulation = {
    .name = "TwinTrap",
    .type = TwinTrap,
    .position = {0.0f, 0.0f, 0.1f},
    .strength = 100,
    .startPoint = {0.0f, 0.00f, 0.1f},
    .endPoint = {0.0f, 0.00f, 0.1f},
    .normalVector = {0.0f, 0.0f, 1.0f},
    .radius = 0.0f,
    .frequency = 200.0f,
    .cached_period_us = 0,
    .cached_circ_u = {0.0f, 0.0f, 0.0f},
    .cached_circ_v = {0.0f, 0.0f, 0.0f},
};

const Stimulation *DemoStimulations[] = {
    &DLM_2_Stimulation,
    &DLM_3_Stimulation,
    &DemoULM_LStimulation,
    &DemoLM_LStimulation,
    &DemoLM_CStimulation,
    &DemoULM_L_M02_Stimulation,
    &DemoULM_L_M05_Stimulation,
    &DemoULM_L_M10_Stimulation,
    &DemoULM_L_M20_Stimulation,
};

void Switch_Demo_Mode()
{
    if (Get_Calibration_Mode() || Audio_Is_Active())
        return;

    static GPIO_PinState debouncedState = GPIO_PIN_SET;
    static GPIO_PinState lastRawState = GPIO_PIN_SET;
    static uint32_t lastDebounceTime = 0;
    const uint32_t debounceDelay = 50;

    GPIO_PinState currentRawState = HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);

    if (currentRawState != lastRawState)
    {
        lastDebounceTime = HAL_GetTick();
    }
    lastRawState = currentRawState;

    if ((HAL_GetTick() - lastDebounceTime) > debounceDelay)
    {
        if (currentRawState != debouncedState)
        {
            if (debouncedState == GPIO_PIN_SET && currentRawState == GPIO_PIN_RESET)
            {
                demo_mode = (demo_mode + 1) % (sizeof(DemoStimulations) / sizeof(DemoStimulations[0]));
                Set_Stimulation(DemoStimulations[demo_mode]);
            }
            debouncedState = currentRawState;
        }
    }
}

static int Is_Stimulation_Param_Equal(const Stimulation *s1, const Stimulation *s2)
{
    if (s1->type != s2->type)
        return 0;

    // Compare scalar parameters
    if (s1->strength != s2->strength)
        return 0;
    if (s1->frequency != s2->frequency)
        return 0;
    if (s1->radius != s2->radius)
        return 0;
    if (s1->segments != s2->segments)
        return 0;
    // Compare vector parameters
    if (memcmp(s1->position, s2->position, sizeof(s1->position)) != 0)
        return 0;
    if (memcmp(s1->startPoint, s2->startPoint, sizeof(s1->startPoint)) != 0)
        return 0;
    if (memcmp(s1->endPoint, s2->endPoint, sizeof(s1->endPoint)) != 0)
        return 0;
    if (memcmp(s1->normalVector, s2->normalVector, sizeof(s1->normalVector)) != 0)
        return 0;

    return 1;
}

static void Set_Stimulation_Internal(const Stimulation *stimulation, int force_update)
{
    int was_phase_set_mode = phase_set_mode;
    phase_set_mode = 0;

    Stimulation sanitized_stimulation = *stimulation;
    sanitized_stimulation.strength = DMA_Clamp_Stimulation_Strength(sanitized_stimulation.strength);
    if (sanitized_stimulation.segments < 1)
    {
        sanitized_stimulation.segments = 1;
    }
    else if (sanitized_stimulation.segments > (int)NUM_STIMULATION_SAMPLES)
    {
        sanitized_stimulation.segments = (int)NUM_STIMULATION_SAMPLES;
    }
    if (!force_update && !was_phase_set_mode &&
        Is_Stimulation_Param_Equal(&CurrentStimulation, &sanitized_stimulation))
    {
        return;
    }

    CurrentStimulation = sanitized_stimulation;

    // Pre-calculate period
    if (CurrentStimulation.frequency > 0.0f)
    {
        CurrentStimulation.cached_period_us = (uint32_t)(1e6f / CurrentStimulation.frequency);
    }
    else
    {
        CurrentStimulation.cached_period_us = 0;
    }

    // Pre-calculate Circular vectors
    switch (CurrentStimulation.type)
    {
    case Point:
        break;
    case Discrete:
    case Circular:
    {
        float n[3] = {CurrentStimulation.normalVector[0], CurrentStimulation.normalVector[1], CurrentStimulation.normalVector[2]};
        Vector3Normalize(n);

        float t_vec[3];
        if (fabsf(n[0]) < 0.9f)
        {
            t_vec[0] = 1.0f;
            t_vec[1] = 0.0f;
            t_vec[2] = 0.0f;
        }
        else
        {
            t_vec[0] = 0.0f;
            t_vec[1] = 1.0f;
            t_vec[2] = 0.0f;
        }

        Vector3Cross(CurrentStimulation.cached_circ_u, t_vec, n);
        Vector3Normalize(CurrentStimulation.cached_circ_u);
        Vector3Cross(CurrentStimulation.cached_circ_v, n, CurrentStimulation.cached_circ_u);
        break;
    }
    default:
        break;
    }

    Update_Full_Waveform_Buffer();
}

void Set_Stimulation(const Stimulation *stimulation)
{
    if (stimulation == NULL || stimulation->type == IndexedLinear || Audio_Is_Active())
    {
        return;
    }
    Set_Stimulation_Internal(stimulation, 0);
}

int Set_Indexed_Linear_Stimulation(const Stimulation *stimulation,
                                   const uint8_t *spatial_order,
                                   uint8_t sample_count,
                                   uint8_t control_flags)
{
    if (stimulation == NULL || stimulation->type != IndexedLinear ||
        !Indexed_Linear_Validate_Order(spatial_order, sample_count, control_flags) ||
        Audio_Is_Active())
    {
        return 0;
    }

    memcpy(indexed_linear_order, spatial_order, sizeof(indexed_linear_order));
    Set_Stimulation_Internal(stimulation, 1);
    return 1;
}

static void Update_Stimulation_State_Internal(float progress, int32_t sample_index)
{
    if (Get_Calibration_Mode() == 1 || Get_Phase_Set_Mode() == 1)
    {
        return;
    }

    switch (CurrentStimulation.type)
    {
    case Point:
        Set_Point_Focus(CurrentStimulation.position);
        break;
    case Discrete:
    {
        if (CurrentStimulation.segments < 1) break;

        int segment_index = (int)(progress * (float)CurrentStimulation.segments);
        if (segment_index >= CurrentStimulation.segments) segment_index = CurrentStimulation.segments - 1;

        if (cached_discrete_segments != CurrentStimulation.segments ||
            cached_discrete_index != segment_index)
        {
            float angle = (float)segment_index * 2.0f * (float)M_PI /
                          (float)CurrentStimulation.segments;
            cached_discrete_cos = cosf(angle);
            cached_discrete_sin = sinf(angle);
            cached_discrete_segments = CurrentStimulation.segments;
            cached_discrete_index = segment_index;
        }
        float cos_a = cached_discrete_cos;
        float sin_a = cached_discrete_sin;

        float discretePosition[3];
        // Use cached u and v vectors
        discretePosition[0] = CurrentStimulation.position[0] + CurrentStimulation.radius * (cos_a * CurrentStimulation.cached_circ_u[0] + sin_a * CurrentStimulation.cached_circ_v[0]);
        discretePosition[1] = CurrentStimulation.position[1] + CurrentStimulation.radius * (cos_a * CurrentStimulation.cached_circ_u[1] + sin_a * CurrentStimulation.cached_circ_v[1]);
        discretePosition[2] = CurrentStimulation.position[2] + CurrentStimulation.radius * (cos_a * CurrentStimulation.cached_circ_u[2] + sin_a * CurrentStimulation.cached_circ_v[2]);

        Set_Point_Focus(discretePosition);
        break;
    }
    case Linear:
    {
        float linearPosition[3];
        int segments = CurrentStimulation.segments;
        if (segments < 1) segments = 1;

        float total_segments = (float)segments;
        float p = progress * total_segments;
        int current_segment = (int)p;

        // Clamp to valid range to handle progress = 1.0f
        if (current_segment >= segments) current_segment = segments - 1;

        float segment_progress = p - (float)current_segment;

        // Even segments: Start -> End
        // Odd segments: End -> Start
        if (current_segment % 2 == 0)
        {
            Vector3Lerp(linearPosition, CurrentStimulation.startPoint, CurrentStimulation.endPoint, segment_progress);
        }
        else
        {
            Vector3Lerp(linearPosition, CurrentStimulation.endPoint, CurrentStimulation.startPoint, segment_progress);
        }

        Set_Point_Focus(linearPosition);
        break;
    }
    case IndexedLinear:
    {
        uint32_t time_index;
        if (sample_index >= 0 && (uint32_t)sample_index < NUM_STIMULATION_SAMPLES)
        {
            time_index = (uint32_t)sample_index;
        }
        else
        {
            time_index = (uint32_t)(progress * (float)NUM_STIMULATION_SAMPLES);
            if (time_index >= NUM_STIMULATION_SAMPLES)
            {
                time_index = NUM_STIMULATION_SAMPLES - 1U;
            }
        }

        uint32_t spatial_index = indexed_linear_order[time_index];
        float spatial_progress = (float)spatial_index /
                                 (float)NUM_STIMULATION_SAMPLES;
        float linearPosition[3];
        Vector3Lerp(linearPosition, CurrentStimulation.startPoint,
                    CurrentStimulation.endPoint, spatial_progress);
        Set_Point_Focus(linearPosition);
        break;
    }
    case Circular:
    {
        float cos_a;
        float sin_a;
        if (sample_index >= 0 && (uint32_t)sample_index < NUM_STIMULATION_SAMPLES)
        {
            Ensure_Unit_Circle_Table();
            cos_a = unit_circle_cos[sample_index];
            sin_a = unit_circle_sin[sample_index];
        }
        else
        {
            float angle = 2.0f * (float)M_PI * progress;
            cos_a = cosf(angle);
            sin_a = sinf(angle);
        }

        float circularPosition[3];
        // Use cached u and v vectors
        circularPosition[0] = CurrentStimulation.position[0] + CurrentStimulation.radius * (cos_a * CurrentStimulation.cached_circ_u[0] + sin_a * CurrentStimulation.cached_circ_v[0]);
        circularPosition[1] = CurrentStimulation.position[1] + CurrentStimulation.radius * (cos_a * CurrentStimulation.cached_circ_u[1] + sin_a * CurrentStimulation.cached_circ_v[1]);
        circularPosition[2] = CurrentStimulation.position[2] + CurrentStimulation.radius * (cos_a * CurrentStimulation.cached_circ_u[2] + sin_a * CurrentStimulation.cached_circ_v[2]);

        Set_Point_Focus(circularPosition);
        break;
    }
    case TwinTrap:
    {
        Set_Twin_Trap_Focus(CurrentStimulation.position);
        break;
    }
    default:
        break;
    }
}

void Update_Stimulation_State(float progress)
{
    Update_Stimulation_State_Internal(progress, -1);
}

void Update_Stimulation_State_Sample(uint32_t sample_index)
{
    if (sample_index >= NUM_STIMULATION_SAMPLES)
    {
        return;
    }

    float progress = (float)sample_index / (float)NUM_STIMULATION_SAMPLES;
    Update_Stimulation_State_Internal(progress, (int32_t)sample_index);
}

int Get_Stimulation_Enabled()
{
    return is_stimulation_enabled;
}

void Stimulation_Enable()
{
    if (Audio_Is_Active()) return;
    if (is_stimulation_enabled) return;
    is_stimulation_enabled = 1;
    Update_Full_Waveform_Buffer();
}

void Stimulation_Disable()
{
    if (Audio_Is_Active()) return;
    if (!is_stimulation_enabled) return;
    is_stimulation_enabled = 0;
    Update_Full_Waveform_Buffer();
}

int Get_Num_Demo_Stimulations()
{
    return sizeof(DemoStimulations) / sizeof(DemoStimulations[0]);
}

int Get_Demo_Mode()
{
    return demo_mode;
}

int Get_Phase_Set_Mode()
{
    return phase_set_mode;
}
