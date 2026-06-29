#define _USE_MATH_DEFINES
#include <stdio.h>
#include "utiles.h"
#include "stimulation.h"
#include "calibration.h"
#include "dma_manager.h"

uint32_t sysTickDelta = 0;
float System_Loop_Freq = 0.0f;
double updateDMABufferDeltaTime = 0;

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
    return DWT_GetCycles() / (SystemCoreClock / 1000000);
}

/* HEARTBEAT (PA15): 1Hz direct GPIO toggle. Port A has no transducers,
 * so DMA does not write GPIOA->ODR. Direct toggle is safe. */
void Update_LED_Status(void)
{
    static uint32_t last_toggle = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_toggle >= 500)
    {
        HAL_GPIO_TogglePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin);
        last_toggle = now;
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

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;

static void Get_ADC1_Values(uint32_t *v33_raw, uint32_t *v50_raw)
{
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
        *v33_raw = HAL_ADC_GetValue(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
        *v50_raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
}

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

float Get_Voltage_3V3(void)
{
    return 0.0f;   /* PA0 (ADC1_INP16) — idle in normal mode, reserved */
}

float Get_Voltage_5V0(void)
{
    return 0.0f;   /* PA1 (ADC1_INP17) — idle in normal mode, reserved */
}

float Get_Temperature(void)
{
    uint32_t temp_raw = 0, vref_raw = 0;
    Get_ADC3_Values(&temp_raw, &vref_raw);
    uint32_t vdda_mv = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(vref_raw, ADC_RESOLUTION_16B);
    int32_t temperature = __HAL_ADC_CALC_TEMPERATURE(vdda_mv, temp_raw, ADC_RESOLUTION_16B);
    return (float)temperature;
}
