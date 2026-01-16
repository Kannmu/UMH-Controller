#define _USE_MATH_DEFINES
// #include <math.h>
#include "stimulation.h"
#include "calibration.h"
#include "utiles.h"
#include "custom_math.h"

int phase_set_mode = 0;

int demo_mode = 0;

Stimulation CurrentStimulation = {
    .type = PointStimulation,
    .position = {0.0f, 0.0f, 0.1f},
    .strength = 100,
    .startPoint = {0.0f, 0.0f, 0.1f},
    .endPoint = {0.0f, 0.0f, 0.1f},
    .centerPoint = {0.0f, 0.0f, 0.1f},
    .normalVector = {0.0f, 0.0f, 0.0f},
    .radius = 0.0f,
    .frequency = 200.0f,
    .cached_period_us = 0,
    .cached_circular_u = {0.0f, 0.0f, 0.0f},
    .cached_circular_v = {0.0f, 0.0f, 0.0f},
};

Stimulation EmptyStimulation = {
    .type = PointStimulation,
    .position = {0.0f, 0.0f, 0.1f},
    .strength = 0,
    .startPoint = {0.0f, 0.0f, 0.1f},
    .endPoint = {0.0f, 0.0f, 0.1f},
    .centerPoint = {0.0f, 0.0f, 0.1f},
    .normalVector = {0.0f, 0.0f, 0.0f},
    .radius = 0.0f,
    .frequency = 0.0f,
    .cached_period_us = 0,
    .cached_circular_u = {0.0f, 0.0f, 0.0f},
    .cached_circular_v = {0.0f, 0.0f, 0.0f},
};

const Stimulation DemoPointStimulation = {
    .type = PointStimulation,
    .position = {0.0f, 0.0f, 0.1f},
    .strength = 100,
    .startPoint = {0.0f, 0.0f, 0.1f},  
    .endPoint = {0.0f, 0.0f, 0.1f},
    .centerPoint = {0.0f, 0.0f, 0.1f},
    .normalVector = {0.0f, 0.0f, 1.0f},
    .radius = 0.0f,
    .frequency = 200.0f,
    .cached_period_us = 0,
    .cached_circular_u = {0.0f, 0.0f, 0.0f},
    .cached_circular_v = {0.0f, 0.0f, 0.0f},
};

const Stimulation DemoVibrationStimulation = {
    .type = VibrationStimulation,
    .position = {0.0f, 0.0f, 0.1f},
    .strength = 100,
    .startPoint = {0.0f, 0.01f, 0.1f},
    .endPoint = {0.0f, -0.01f, 0.1f},
    .centerPoint = {0.0f, 0.0f, 0.1f},
    .normalVector = {0.0f, 0.0f, 1.0f},
    .radius = 0.0f,
    .frequency = 200.0f,
    .cached_period_us = 0,
    .cached_circular_u = {0.0f, 0.0f, 0.0f},
    .cached_circular_v = {0.0f, 0.0f, 0.0f},
};

const Stimulation DemoLinearStimulation = {
    .type = Linear,
    .position = {0.0f, 0.0f, 0.1f},
    .strength = 100,
    .startPoint = {0.0f, 0.01f, 0.1f},
    .endPoint = {0.0f, -0.01f, 0.1f},
    .centerPoint = {0.0f, 0.0f, 0.1f},
    .normalVector = {0.0f, 0.0f, 1.0f},
    .radius = 0.0f,
    .frequency = 200.0f,
    .cached_period_us = 0,
    .cached_circular_u = {0.0f, 0.0f, 0.0f},
    .cached_circular_v = {0.0f, 0.0f, 0.0f},
};

const Stimulation DemoCircularStimulation = {
    .type = Circular,
    .position = {0.0f, 0.0f, 0.1f},
    .strength = 100,
    .startPoint = {0.0f, 0.0f, 0.1f},
    .endPoint = {0.0f, 0.0f, 0.1f},
    .centerPoint = {0.0f, 0.0f, 0.1f},
    .normalVector = {0.0f, 0.0f, 1.0f},
    .radius = 0.01f,
    .frequency = 200.0f,
    .cached_period_us = 0,
    .cached_circular_u = {0.0f, 0.0f, 0.0f},
    .cached_circular_v = {0.0f, 0.0f, 0.0f},
};

const Stimulation DemoTwinTrapStimulation = {
    .type = TwinTrap,
    .position = {0.0f, 0.0f, 0.1f},
    .strength = 100,
    .startPoint = {0.0f, 0.00f, 0.1f},
    .endPoint = {0.0f, 0.00f, 0.1f},
    .centerPoint = {0.0f, 0.0f, 0.1f},
    .normalVector = {0.0f, 0.0f, 1.0f},
    .radius = 0.0f,
    .frequency = 200.0f,
    .cached_period_us = 0,
    .cached_circular_u = {0.0f, 0.0f, 0.0f},
    .cached_circular_v = {0.0f, 0.0f, 0.0f},
};

const Stimulation DemoRUSTStimulation = {


};


const Stimulation *DemoStimulations[] = {
    &CurrentStimulation,
    &DemoPointStimulation,
    &DemoVibrationStimulation,
    &DemoLinearStimulation,
    &DemoCircularStimulation,
    &DemoTwinTrapStimulation,
};

void Switch_Demo_Mode()
{
    if(Get_Calibration_Mode()) return;

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
    if (s1->type != s2->type) return 0;

    // Compare scalar parameters
    if (s1->strength != s2->strength) return 0;
    if (s1->frequency != s2->frequency) return 0;
    if (s1->radius != s2->radius) return 0;

    // Compare vector parameters
    if (memcmp(s1->position, s2->position, sizeof(s1->position)) != 0) return 0;
    if (memcmp(s1->startPoint, s2->startPoint, sizeof(s1->startPoint)) != 0) return 0;
    if (memcmp(s1->endPoint, s2->endPoint, sizeof(s1->endPoint)) != 0) return 0;
    if (memcmp(s1->centerPoint, s2->centerPoint, sizeof(s1->centerPoint)) != 0) return 0;
    if (memcmp(s1->normalVector, s2->normalVector, sizeof(s1->normalVector)) != 0) return 0;

    return 1;
}

void Set_Stimulation(const Stimulation *stimulation)
{
    if (Is_Stimulation_Param_Equal(&CurrentStimulation, stimulation))
    {
        return;
    }

    CurrentStimulation = *stimulation;

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
        case PointStimulation:
            break;
        case VibrationStimulation:
            break;
        case Linear:
            break;
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

            Vector3Cross(CurrentStimulation.cached_circular_u, t_vec, n);
            Vector3Normalize(CurrentStimulation.cached_circular_u);
            Vector3Cross(CurrentStimulation.cached_circular_v, n, CurrentStimulation.cached_circular_u);
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
    case PointStimulation:
        Set_Point_Focus(CurrentStimulation.position);
        break;
    case VibrationStimulation:
    {
        if (progress < 0.5f)
        {
            Set_Point_Focus(CurrentStimulation.startPoint);
        }
        else
        {
            Set_Point_Focus(CurrentStimulation.endPoint);
        }
        break;
    }
    case Linear:
    {
        float linearPosition[3];
        Vector3Lerp(linearPosition, CurrentStimulation.startPoint, CurrentStimulation.endPoint, progress);
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
        circularPosition[0] = CurrentStimulation.centerPoint[0] + CurrentStimulation.radius * (cos_a * CurrentStimulation.cached_circular_u[0] + sin_a * CurrentStimulation.cached_circular_v[0]);
        circularPosition[1] = CurrentStimulation.centerPoint[1] + CurrentStimulation.radius * (cos_a * CurrentStimulation.cached_circular_u[1] + sin_a * CurrentStimulation.cached_circular_v[1]);
        circularPosition[2] = CurrentStimulation.centerPoint[2] + CurrentStimulation.radius * (cos_a * CurrentStimulation.cached_circular_u[2] + sin_a * CurrentStimulation.cached_circular_v[2]);

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

int Get_Demo_Mode()
{
    return demo_mode;
}

int Get_Phase_Set_Mode()
{
    return phase_set_mode;
}