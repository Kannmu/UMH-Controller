#define _USE_MATH_DEFINES
#include "calibration.h"
#include "transducer.h"
#include "dma_manager.h"
#include "utiles.h"

int calibration_mode = 0;

// 校准参数说明：每个元素对应换能器的延迟校准值（单位：微秒us）
// Calibration Parameters for Device 1 ID: 003C00203233
// float Transducer_Calibration_Array[] = {
//     22.5, 0, 24.7, 24, 5.5,
//     0, 15.2, 10, 22.5, 16, 12.5,
//     9.6, 22.6, 2.6, 12.4, 2.3, 10.5, 9.5,
//     24, 24, 24.5, 6.9, 17.9, 24.3, 8, 22.2,
//     7.9, 16.2, 3.1, 5.9, 6.9, 14.3, 8.5, 22.8,
//     24.5, 18.9, 11, 5.5, 18.3, 13.3, 21.8, 0,
//     8.6, 23.1, 0, 24.5, 1.5, 23, 24.6,
//     0, 16, 7.9, 23, 1.5, 0,
//     6, 0, 19, 12.5, 20.6,
//     0};


// Calibration Parameters for Device 2 ID: 003C00193233
float Transducer_Calibration_Array[] = {
    15.47,  9.42, 14.97,  5.23,  7.66,
    14.84,  8.56, 14.28,  6.14,  5.23,
    16.02, 10.00,  5.90,  5.04, 15.22,
    5.51,  6.28,  5.40,  5.32,  6.22,
    4.98,  5.05, 19.97,  5.45, 15.78,
    14.98,  5.57,  4.97,  5.80, 14.65,
    5.62,  5.78,  5.21,  5.54, 14.40,
    6.17,  5.77, 17.00, 15.39,  5.85,
    8.94, 10.32, 20.22, 21.25, 20.31,
    8.65, 18.44,  9.63, 21.20, 21.23,
    19.99,  8.81, 21.29,  8.57, 19.49,
    8.94,  9.33,  8.61, 10.01,  8.63,
    0};

// float Transducer_Calibration_Array[] = {
//     0, 0, 0, 0, 0,
//     0, 0, 0, 0, 0, 0,
//     0, 0, 0, 0, 0, 0, 0,
//     0, 0, 0, 0, 0, 0, 0, 0,
//     0, 0, 0, 0, 0, 0, 0, 0,
//     0, 0, 0, 0, 0, 0, 0, 0,
//     0, 0, 0, 0, 0, 0, 0,
//     0, 0, 0, 0, 0, 0,
//     0, 0, 0, 0, 0,
//     0};


void Switch_Calibration_Mode()
{
    static GPIO_PinState debouncedState = GPIO_PIN_SET;
    static GPIO_PinState lastRawState = GPIO_PIN_SET;
    static uint32_t lastDebounceTime = 0;
    const uint32_t debounceDelay = 50;

    GPIO_PinState currentRawState = HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin);

    if (currentRawState != lastRawState)
    {
        lastDebounceTime = HAL_GetTick();
    }
    lastRawState = currentRawState;

    if ((HAL_GetTick() - lastDebounceTime) > debounceDelay)
    {
        if (currentRawState != debouncedState)
        {
            // Toggle Calibration Mode on Falling Edge
            if (debouncedState == GPIO_PIN_SET && currentRawState == GPIO_PIN_RESET)
            {
                calibration_mode = 1 - calibration_mode;

                if (calibration_mode == 1)
                {
                    Enter_Calibration_Mode();
                }
                else
                {
                    Load_Calib_to_Transducers();
                }
                Update_Full_Waveform_Buffer();
            }
            debouncedState = currentRawState;
        }
    }
}

int Get_Calibration_Mode()
{
    return calibration_mode;
}