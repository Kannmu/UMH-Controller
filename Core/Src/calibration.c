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

// float Transducer_Calibration_Array[] = {
//     23.23, 17.31, 18.92, 8.48, 21.21,
//     7.08, 15.13, 12.35, 16.95, 7.33,
//     12.32, 2.85, 15.51, 5.70, 13.94,
//     6.21, 15.40, 14.06, 3.78, 17.85,
//     11.42, 4.67, 3.85, 10.15, 14.13,
//     11.05, 14.55, 7.49, 5.13, 23.81,
//     0.91, 6.14, 7.64, 16.81, 7.23,
//     15.42, 8.85, 0.93, 1.55, 11.26,
//     22.50, 15.48, 21.91, 1.88, 15.31,
//     16.03, 13.26, 24.63, 0.60, 18.92,
//     13.70, 1.56, 3.97, 17.46, 15.94,
//     21.61, 14.91, 1.06, 16.68, 23.36,
//     0};

// Calibration Parameters for Device 2 ID: 003C00193233
float Transducer_Calibration_Array[] = {
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
    0};





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