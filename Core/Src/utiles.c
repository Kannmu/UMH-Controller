#define _USE_MATH_DEFINES
#include <stdio.h>
#include "utiles.h"
#include "stimulation.h"
#include "calibration.h"
#include "dma_manager.h"

uint32_t sysTickDelta = 0;
float System_Loop_Freq = 0.0f;
double updateDMABufferDeltaTime = 0;
double updateDMABufferDeltaTimeByType[STIM_MAX_TYPES] = {0};

void Init_DWT()
{
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t DWT_GetCycles(void)
{
    return DWT->CYCCNT;
}

uint32_t DWT_GetMicroseconds(void)
{
    return DWT_GetCycles() / (SystemCoreClock / US_PER_SEC);
}

/* HEARTBEAT (PA15): Simple breathing LED via 1st‑order sigma‑delta modulation.
 * Must be called at 1 kHz (from SysTick_Handler) for flicker‑free output.
 * No hardware timer, no LUT — just integer add/compare.
 * Triangular wave (0→255→0) over ~4.1 s. */
void Update_LED_Status(void)
{
    static int16_t sd_err;

    /* Triangular wave: 0→255 in 2048 ms, then 255→0 in 2048 ms */
    uint16_t phase = HAL_GetTick() & 2047;   /* 0..2047 wraps every 2.048 s */
    uint8_t  target = (phase < 1024) ? (uint8_t)(phase >> 3)
                                     : (uint8_t)((2047 - phase) >> 3);

    /* 1st‑order sigma‑delta DAC */
    sd_err += (int16_t)target;
    if (sd_err >= 0) {
        HAL_GPIO_WritePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin, GPIO_PIN_SET);
        sd_err -= 256;
    } else {
        HAL_GPIO_WritePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin, GPIO_PIN_RESET);
    }
}

char* Get_Device_Serial_Number(void)
{
    static char serial_str[25];
    uint32_t uid0 = HAL_GetUIDw0();
    uint32_t uid1 = HAL_GetUIDw1();
    uint32_t uid2 = HAL_GetUIDw2();
    snprintf(serial_str, sizeof(serial_str), "%08lX%08lX%08lX", (unsigned long)uid0, (unsigned long)uid1, (unsigned long)uid2);
    return serial_str;
}

extern ADC_HandleTypeDef hadc3;

static void Get_ADC3_Values(uint32_t *temp_raw, uint32_t *vref_raw)
{
    HAL_ADC_Start(&hadc3);
    if (HAL_ADC_PollForConversion(&hadc3, 100) == HAL_OK)
        *temp_raw = HAL_ADC_GetValue(&hadc3);
    if (HAL_ADC_PollForConversion(&hadc3, 100) == HAL_OK)
        *vref_raw = HAL_ADC_GetValue(&hadc3);
    HAL_ADC_Stop(&hadc3);
}

float Get_Voltage_VDDA(void)
{
    uint32_t temp_raw = 0, vref_raw = 0;
    Get_ADC3_Values(&temp_raw, &vref_raw);
    uint32_t vdda_mv = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(vref_raw, ADC_RESOLUTION_16B);
    return (float)vdda_mv / 1000.0f;
}

float Get_Temperature(void)
{
    uint32_t temp_raw = 0, vref_raw = 0;
    Get_ADC3_Values(&temp_raw, &vref_raw);
    uint32_t vdda_mv = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(vref_raw, ADC_RESOLUTION_16B);
    int32_t temperature = __HAL_ADC_CALC_TEMPERATURE(vdda_mv, temp_raw, ADC_RESOLUTION_16B);
    return (float)temperature;
}

float Get_Refresh_Rate(void)
{
    if (updateDMABufferDeltaTime > 0.0)
        return (float)(1000.0 / updateDMABufferDeltaTime);
    return 0.0f;
}
