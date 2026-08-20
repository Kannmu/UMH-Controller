#define _USE_MATH_DEFINES
#include "calibration.h"
#include "transducer.h"
#include "dma_manager.h"
#include "utiles.h"
#include "audio_playback.h"

int calibration_mode = 0;

// 校准参数说明：每个元素对应换能器的延迟校准值（单位：微秒us）
// Calibration Parameters for Device 1 ID: 003C00203233
// float Transducer_Calibration_Array[] = {
//     23.75, 10.98, 10.84, 13.84, 11.74,
//     11.36, 12.15, 11.81, 25.00, 11.53,
//     24.43, 24.52, 24.87, 11.75, 23.86,
//     11.58, 11.19,  4.30, 12.96, 23.73,
//     12.16, 24.54, 13.29, 12.50, 23.45,
//     23.10, 11.74, 11.41, 11.38,  0.31,
//     24.60,  0.36,  0.31,  1.08, 12.80,
//      3.27,  0.19,  0.42, 12.63, 23.85,
//      0.09, 11.89,  0.19,  0.48, 13.46,
//     12.92, 12.05,  0.55, 11.69, 12.14,
//     11.72, 14.13,  0.10, 24.86, 12.06,
//     12.92, 12.04, 16.90,  0.05,  0.20,
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
    if (Audio_Is_Active()) return;
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
