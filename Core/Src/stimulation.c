#define _USE_MATH_DEFINES
// #include <math.h>
#include "stimulation.h"
#include "calibration.h"
#include "utiles.h"
#include "custom_math.h"

int plane_mode = 0;

int phase_set_mode = 0;

Stimulation CurrentStimulation = {
    .type = PointStimulation,
    .position = {0.0f, 0.0f, 0.05f},
    .strength = 100,
    .startPoint = {0.0f, 0.0f, 0.05f},
    .endPoint = {0.0f, 0.0f, 0.05f},
    .centerPoint = {0.0f, 0.0f, 0.05f},
    .normalVector = {0.0f, 0.0f, 0.0f},
    .radius = 0.0f,
    .frequency = 0.0f,
    .progress = 0.0f,
    .cached_period_us = 0,
    .cached_u = {0.0f, 0.0f, 0.0f},
    .cached_v = {0.0f, 0.0f, 0.0f},
};

Stimulation EmptyStimulation = {
    .type = PointStimulation,
    .position = {0.0f, 0.0f, 0.05f},
    .strength = 0,
    .startPoint = {0.0f, 0.0f, 0.05f},
    .endPoint = {0.0f, 0.0f, 0.05f},
    .centerPoint = {0.0f, 0.0f, 0.05f},
    .normalVector = {0.0f, 0.0f, 0.0f},
    .radius = 0.0f,
    .frequency = 0.0f,
    .progress = 0.0f,
    .cached_period_us = 0,
    .cached_u = {0.0f, 0.0f, 0.0f},
    .cached_v = {0.0f, 0.0f, 0.0f},
};

void Switch_Plane_Mode()
{
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
            // Toggle Plane Mode on Falling Edge
            if (debouncedState == GPIO_PIN_SET && currentRawState == GPIO_PIN_RESET)
            {
                plane_mode = 1 - plane_mode;
                if (plane_mode == 1)
                {
                    Set_Plane_Wave();
                }
                Update_All_DMABuffer();
            }
            debouncedState = currentRawState;
        }
    }
}

int Get_Plane_Mode()
{
    return plane_mode;
}

void Set_Stimulation(Stimulation *stimulation)
{
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

    // Pre-calculate CircularSTM vectors
    if (CurrentStimulation.type == CircularSTM)
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

        Vector3Cross(CurrentStimulation.cached_u, t_vec, n);
        Vector3Normalize(CurrentStimulation.cached_u);
        Vector3Cross(CurrentStimulation.cached_v, n, CurrentStimulation.cached_u);
    }
}

void Apply_Stimulation()
{
    if (Get_Calibration_Mode() == 1 || Get_Plane_Mode() == 1)
    {
        return;
    }

    // Calculate progress using cached period
    if (CurrentStimulation.cached_period_us > 0)
    {
        CurrentStimulation.progress = (float)(DWT_GetMicroseconds() % CurrentStimulation.cached_period_us) / CurrentStimulation.cached_period_us;
    }
    else
    {
        CurrentStimulation.progress = 0.0f;
    }

    switch (CurrentStimulation.type)
    {
    case PointStimulation:
        Set_Point_Focus(CurrentStimulation.position);
        break;
    case VibrationStimulation:
    {
        if (CurrentStimulation.progress < 0.5f)
        {
            Set_Point_Focus(CurrentStimulation.startPoint);
        }
        else
        {
            Set_Point_Focus(CurrentStimulation.endPoint);
        }
    }

    break;
    case LinearSTM:
    {
        float linearPosition[3];
        Vector3Lerp(linearPosition, CurrentStimulation.startPoint, CurrentStimulation.endPoint, CurrentStimulation.progress);
        Set_Point_Focus(linearPosition);
    }
    break;
    case CircularSTM:
    {
        float angle = 2.0f * (float)M_PI * CurrentStimulation.progress;
        float cos_a = cosf(angle);
        float sin_a = sinf(angle);

        float circularPosition[3];
        // Use cached u and v vectors
        circularPosition[0] = CurrentStimulation.centerPoint[0] + CurrentStimulation.radius * (cos_a * CurrentStimulation.cached_u[0] + sin_a * CurrentStimulation.cached_v[0]);
        circularPosition[1] = CurrentStimulation.centerPoint[1] + CurrentStimulation.radius * (cos_a * CurrentStimulation.cached_u[1] + sin_a * CurrentStimulation.cached_v[1]);
        circularPosition[2] = CurrentStimulation.centerPoint[2] + CurrentStimulation.radius * (cos_a * CurrentStimulation.cached_u[2] + sin_a * CurrentStimulation.cached_v[2]);

        Set_Point_Focus(circularPosition);
    }
    break;
    default:
        break;
    }

    Update_All_DMABuffer();
}