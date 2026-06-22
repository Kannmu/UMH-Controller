#define _USE_MATH_DEFINES
// #include <math.h>
#include "stimulation.h"
#include "calibration.h"
#include "utiles.h"
#include "custom_math.h"
#include "dma_manager.h"

volatile int phase_set_mode = 0;
volatile int is_stimulation_enabled = 1;

volatile int demo_mode = -1;

float spiral_lut[SPIRAL_LUT_SIZE] = {0};

static float F_arc(float theta)
{
    return theta * sqrtf(theta * theta + 1.0f) + logf(theta + sqrtf(theta * theta + 1.0f));
}

static void Generate_Spiral_LUT(float radius)
{
    float theta_max = SPIRAL_THETA_MAX;
    float F_max = F_arc(theta_max);

    for (int i = 0; i < SPIRAL_LUT_SIZE; i++)
    {
        float p = (float)i / (float)(SPIRAL_LUT_SIZE - 1);
        float target = p * F_max;

        float lo = 0.0f;
        float hi = theta_max;
        for (int iter = 0; iter < 30; iter++)
        {
            float mid = (lo + hi) * 0.5f;
            if (F_arc(mid) < target)
                lo = mid;
            else
                hi = mid;
        }
        spiral_lut[i] = (lo + hi) * 0.5f;
    }
}

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
    .radius = 4.775e-3f,
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


const Stimulation DemoSquareStimulation = {
    .name = "Square",
    .type = Square,
    .position = {0.0f, 0.0f, 0.1f},
    .strength = 100,
    .startPoint = {0.0f, 0.0f, 0.1f},
    .endPoint = {0.0f, 0.0f, 0.1f},
    .segments = 1,
    .normalVector = {0.0f, 0.0f, 1.0f},
    .radius = 3.125e-3f,
    .frequency = 200.0f,
    .cached_period_us = 0,
    .cached_circ_u = {0.0f, 0.0f, 0.0f},
    .cached_circ_v = {0.0f, 0.0f, 0.0f},
};

const Stimulation DemoSTM_TriangleStimulation = {
    .name = "STM_Triangle",
    .type = STM_Triangle,
    .position = {0.0f, 0.0f, 0.1f},
    .strength = 100,
    .startPoint = {0.0f, 0.0f, 0.1f},
    .endPoint = {0.0f, 0.0f, 0.1f},
    .segments = 1,
    .normalVector = {0.0f, 0.0f, 1.0f},
    .radius = 4.811e-3f,
    .frequency = 200.0f,
    .cached_period_us = 0,
    .cached_circ_u = {0.0f, 0.0f, 0.0f},
    .cached_circ_v = {0.0f, 0.0f, 0.0f},
};

const Stimulation DemoZStimulation = {
    .name = "Z",
    .type = Zigzag,
    .position = {0.0f, 0.0f, 0.1f},
    .strength = 100,
    .startPoint = {0.0f, 0.0f, 0.1f},
    .endPoint = {0.0f, 0.0f, 0.1f},
    .segments = 0,
    .normalVector = {0.0f, 0.0f, 1.0f},
    .radius = 3.663e-3f,
    .frequency = 200.0f,
    .cached_period_us = 0,
    .cached_circ_u = {0.0f, 0.0f, 0.0f},
    .cached_circ_v = {0.0f, 0.0f, 0.0f},
};

const Stimulation DemoSpiralInStimulation = {
    .name = "SpiralIn",
    .type = ArchimedeanSpiralInward,
    .position = {0.0f, 0.0f, 0.1f},
    .strength = 100,
    .startPoint = {0.0f, 0.0f, 0.1f},
    .endPoint = {0.0f, 0.0f, 0.1f},
    .segments = 0,
    .normalVector = {0.0f, 0.0f, 1.0f},
    .radius = 5.107e-3f,
    .frequency = 200.0f,
    .cached_period_us = 0,
    .cached_circ_u = {0.0f, 0.0f, 0.0f},
    .cached_circ_v = {0.0f, 0.0f, 0.0f},
};

const Stimulation DemoSpiralOutStimulation = {
    .name = "SpiralOut",
    .type = ArchimedeanSpiralOutward,
    .position = {0.0f, 0.0f, 0.1f},
    .strength = 100,
    .startPoint = {0.0f, 0.0f, 0.1f},
    .endPoint = {0.0f, 0.0f, 0.1f},
    .segments = 0,
    .normalVector = {0.0f, 0.0f, 1.0f},
    .radius = 5.107e-3f,
    .frequency = 200.0f,
    .cached_period_us = 0,
    .cached_circ_u = {0.0f, 0.0f, 0.0f},
    .cached_circ_v = {0.0f, 0.0f, 0.0f},
};

const Stimulation *DemoStimulations[] = {
    &DemoLM_CStimulation,
    &DemoSquareStimulation,
    &DemoSTM_TriangleStimulation,
    &DemoZStimulation,
    &DemoSpiralInStimulation,
    &DemoSpiralOutStimulation,
};

void Switch_Demo_Mode()
{
    if (Get_Calibration_Mode())
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

void Set_Stimulation(const Stimulation *stimulation)
{
    Stimulation sanitized_stimulation = *stimulation;
    sanitized_stimulation.strength = DMA_Clamp_Stimulation_Strength(sanitized_stimulation.strength);

    if (Is_Stimulation_Param_Equal(&CurrentStimulation, &sanitized_stimulation))
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
    case Square:
    case STM_Triangle:
    case Zigzag:
    case ArchimedeanSpiralInward:
    case ArchimedeanSpiralOutward:
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

        if (CurrentStimulation.type == ArchimedeanSpiralInward ||
            CurrentStimulation.type == ArchimedeanSpiralOutward)
        {
            float r = CurrentStimulation.radius;
            if (r <= 0.0f) r = 0.005107f;
            Generate_Spiral_LUT(r);
        }
        break;
    }
    default:
        break;
    }

    Update_Full_Waveform_Buffer();
}

void Update_Stimulation_State(float progress)
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

        float angle = (float)segment_index * 2.0f * (float)M_PI / (float)CurrentStimulation.segments;
        float cos_a = cosf(angle);
        float sin_a = sinf(angle);

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
    case Circular:
    {
        float angle = 2.0f * (float)M_PI * progress;
        float cos_a = cosf(angle);
        float sin_a = sinf(angle);

        float circularPosition[3];
        // Use cached u and v vectors
        circularPosition[0] = CurrentStimulation.position[0] + CurrentStimulation.radius * (cos_a * CurrentStimulation.cached_circ_u[0] + sin_a * CurrentStimulation.cached_circ_v[0]);
        circularPosition[1] = CurrentStimulation.position[1] + CurrentStimulation.radius * (cos_a * CurrentStimulation.cached_circ_u[1] + sin_a * CurrentStimulation.cached_circ_v[1]);
        circularPosition[2] = CurrentStimulation.position[2] + CurrentStimulation.radius * (cos_a * CurrentStimulation.cached_circ_u[2] + sin_a * CurrentStimulation.cached_circ_v[2]);

        Set_Point_Focus(circularPosition);
        break;
    }
    case Square:
    {
        float r = CurrentStimulation.radius;
        if (r <= 0.0f) r = 0.0105f;

        float p = progress * 4.0f;
        int seg = (int)p;
        if (seg >= 4) seg = 3;
        float t = p - (float)seg;

        float cu[3] = {CurrentStimulation.cached_circ_u[0], CurrentStimulation.cached_circ_u[1], CurrentStimulation.cached_circ_u[2]};
        float cv[3] = {CurrentStimulation.cached_circ_v[0], CurrentStimulation.cached_circ_v[1], CurrentStimulation.cached_circ_v[2]};

        float signs[4][2] = {{-1.0f, -1.0f}, {1.0f, -1.0f}, {1.0f, 1.0f}, {-1.0f, 1.0f}};
        float corners[4][3];
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                corners[i][j] = CurrentStimulation.position[j] + r * (signs[i][0] * cu[j] + signs[i][1] * cv[j]);
            }
        }

        int next = (seg + 1) % 4;
        float squarePos[3];
        Vector3Lerp(squarePos, corners[seg], corners[next], t);
        Set_Point_Focus(squarePos);
        break;
    }
    case STM_Triangle:
    {
        float r = CurrentStimulation.radius;
        if (r <= 0.0f) r = 0.005f;

        float p = progress * 3.0f;
        int seg = (int)p;
        if (seg >= 3) seg = 2;
        float t = p - (float)seg;

        float cu[3] = {CurrentStimulation.cached_circ_u[0], CurrentStimulation.cached_circ_u[1], CurrentStimulation.cached_circ_u[2]};
        float cv[3] = {CurrentStimulation.cached_circ_v[0], CurrentStimulation.cached_circ_v[1], CurrentStimulation.cached_circ_v[2]};

        float vertices[3][3];
        for (int i = 0; i < 3; i++)
        {
            float angle = -(float)M_PI / 2.0f + (float)i * 2.0f * (float)M_PI / 3.0f;
            float ca = cosf(angle);
            float sa = sinf(angle);
            for (int j = 0; j < 3; j++)
            {
                vertices[i][j] = CurrentStimulation.position[j] + r * (ca * cu[j] + sa * cv[j]);
            }
        }

        int next = (seg + 1) % 3;
        float triPos[3];
        Vector3Lerp(triPos, vertices[seg], vertices[next], t);
        Set_Point_Focus(triPos);
        break;
    }
    case Zigzag:
    {
        float r = CurrentStimulation.radius;
        if (r <= 0.0f) r = 0.003663f;

        float cu[3] = {CurrentStimulation.cached_circ_u[0], CurrentStimulation.cached_circ_u[1], CurrentStimulation.cached_circ_u[2]};
        float cv[3] = {CurrentStimulation.cached_circ_v[0], CurrentStimulation.cached_circ_v[1], CurrentStimulation.cached_circ_v[2]};

        // Z-shape: 3 segments. Seg1: (-r,r)->(r,r), Seg2: (r,r)->(-r,-r), Seg3: (-r,-r)->(r,-r)
        // Lengths: seg1=2r, seg2=2√2r, seg3=2r. Total = r(4+2√2)
        float seg1_end = 2.0f / (4.0f + 2.0f * sqrtf(2.0f));       // ≈ 0.2930
        float seg2_end = (2.0f + 2.0f * sqrtf(2.0f)) / (4.0f + 2.0f * sqrtf(2.0f)); // ≈ 0.7074

        float start[2], end[2];
        if (progress < seg1_end)
        {
            float t = progress / seg1_end;
            start[0] = -r; start[1] =  r;
            end[0]   =  r; end[1]   =  r;
            float x = start[0] + t * (end[0] - start[0]);
            float y = start[1] + t * (end[1] - start[1]);
            float pos[3];
            pos[0] = CurrentStimulation.position[0] + x * cu[0] + y * cv[0];
            pos[1] = CurrentStimulation.position[1] + x * cu[1] + y * cv[1];
            pos[2] = CurrentStimulation.position[2] + x * cu[2] + y * cv[2];
            Set_Point_Focus(pos);
        }
        else if (progress < seg2_end)
        {
            float t = (progress - seg1_end) / (seg2_end - seg1_end);
            start[0] =  r; start[1] =  r;
            end[0]   = -r; end[1]   = -r;
            float x = start[0] + t * (end[0] - start[0]);
            float y = start[1] + t * (end[1] - start[1]);
            float pos[3];
            pos[0] = CurrentStimulation.position[0] + x * cu[0] + y * cv[0];
            pos[1] = CurrentStimulation.position[1] + x * cu[1] + y * cv[1];
            pos[2] = CurrentStimulation.position[2] + x * cu[2] + y * cv[2];
            Set_Point_Focus(pos);
        }
        else
        {
            float t = (progress - seg2_end) / (1.0f - seg2_end);
            start[0] = -r; start[1] = -r;
            end[0]   =  r; end[1]   = -r;
            float x = start[0] + t * (end[0] - start[0]);
            float y = start[1] + t * (end[1] - start[1]);
            float pos[3];
            pos[0] = CurrentStimulation.position[0] + x * cu[0] + y * cv[0];
            pos[1] = CurrentStimulation.position[1] + x * cu[1] + y * cv[1];
            pos[2] = CurrentStimulation.position[2] + x * cu[2] + y * cv[2];
            Set_Point_Focus(pos);
        }
        break;
    }
    case ArchimedeanSpiralOutward:
    {
        float r_max = CurrentStimulation.radius;
        if (r_max <= 0.0f) r_max = 0.005107f;
        float a = r_max / SPIRAL_THETA_MAX;

        float cu[3] = {CurrentStimulation.cached_circ_u[0], CurrentStimulation.cached_circ_u[1], CurrentStimulation.cached_circ_u[2]};
        float cv[3] = {CurrentStimulation.cached_circ_v[0], CurrentStimulation.cached_circ_v[1], CurrentStimulation.cached_circ_v[2]};

        int idx = (int)(progress * (float)(SPIRAL_LUT_SIZE - 1));
        float theta = spiral_lut[idx];
        float r = a * theta;
        float cos_t = cosf(theta);
        float sin_t = sinf(theta);

        float pos[3];
        pos[0] = CurrentStimulation.position[0] + r * (cos_t * cu[0] + sin_t * cv[0]);
        pos[1] = CurrentStimulation.position[1] + r * (cos_t * cu[1] + sin_t * cv[1]);
        pos[2] = CurrentStimulation.position[2] + r * (cos_t * cu[2] + sin_t * cv[2]);
        Set_Point_Focus(pos);
        break;
    }
    case ArchimedeanSpiralInward:
    {
        float r_max = CurrentStimulation.radius;
        if (r_max <= 0.0f) r_max = 0.005107f;
        float a = r_max / SPIRAL_THETA_MAX;

        float cu[3] = {CurrentStimulation.cached_circ_u[0], CurrentStimulation.cached_circ_u[1], CurrentStimulation.cached_circ_u[2]};
        float cv[3] = {CurrentStimulation.cached_circ_v[0], CurrentStimulation.cached_circ_v[1], CurrentStimulation.cached_circ_v[2]};

        int idx = (int)((1.0f - progress) * (float)(SPIRAL_LUT_SIZE - 1));
        float theta = spiral_lut[idx];
        float r = a * theta;
        float cos_t = cosf(theta);
        float sin_t = sinf(theta);

        float pos[3];
        pos[0] = CurrentStimulation.position[0] + r * (cos_t * cu[0] + sin_t * cv[0]);
        pos[1] = CurrentStimulation.position[1] + r * (cos_t * cu[1] + sin_t * cv[1]);
        pos[2] = CurrentStimulation.position[2] + r * (cos_t * cu[2] + sin_t * cv[2]);
        Set_Point_Focus(pos);
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

int Get_Stimulation_Enabled()
{
    return is_stimulation_enabled;
}

void Stimulation_Enable()
{
    if (is_stimulation_enabled) return;
    is_stimulation_enabled = 1;
    Update_Full_Waveform_Buffer();
}

void Stimulation_Disable()
{
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
