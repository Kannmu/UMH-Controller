#define _USE_MATH_DEFINES
#include "calibration.h"
#include "transducer.h"
#include "dma_manager.h"
#include "utiles.h"

int calibration_mode = 0;

// 校准参数说明：每个元素对应换能器的延迟校准值（单位：微秒us）
float Transducer_Calibration_Array[] = {
    22.5, 0, 24.7, 24, 5.5,
    0, 15.2, 10, 22.5, 16, 12.5,
    9.6, 22.6, 2.6, 12.4, 2.3, 10.5, 9.5,
    24, 24, 24.5, 6.9, 17.9, 24.3, 8, 22.2,
    7.9, 16.2, 3.1, 5.9, 6.9, 14.3, 8.5, 22.8,
    24.5, 18.9, 11, 5.5, 18.3, 13.3, 21.8, 0,
    8.6, 23.1, 0, 24.5, 1.5, 23, 24.6,
    0, 16, 7.9, 23, 1.5, 0,
    6, 0, 19, 12.5, 20.6,
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