#define _USE_MATH_DEFINES
#include "calibration.h"
#include "transducer.h"
#include "dma_manager.h"
#include "utiles.h"

int calibration_mode = 0;

// 校准参数说明：每个元素对应换能器的延迟校准值（单位：微秒us）
float Transducer_Calibration_Array[] = {
    0,0,0,0,0,
    0,0,0,0,0,0,
    0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,
    0,0,0,0,0,0,
    0,0,0,0,0,
    0
};


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
                    Clean_Transducers_Calib();
                }
                else
                {
                    Set_Transducers_Calib();
                }
                Update_All_DMABuffer(2);
            }
            debouncedState = currentRawState;
        }
    }
}

int Get_Calibration_Mode()
{
    return calibration_mode;
}