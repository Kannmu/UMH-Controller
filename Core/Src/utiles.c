#define _USE_MATH_DEFINES
#include "utiles.h"
#include "stimulation.h"
#include "calibration.h"

// debug.c
const uint16_t HALF_LED_BLINK_PERIOD = 500U;
uint16_t led0_ticks = 0;
uint32_t sysTickDelta= 0;
uint32_t FPS = 0;
int led0_state = 0;
int last_led0_state = 0;
uint16_t stm_test_ticks = 0;
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

// 获取系统启动后的CPU周期数
uint32_t DWT_GetCycles(void)
{
    return DWT->CYCCNT;
}

// 转换为微秒（根据系统时钟频率调整）
uint32_t DWT_GetMicroseconds(void)
{
    return DWT_GetCycles() / (SystemCoreClock / 1000000);
}

void Calculate_FPS()
{
    static uint32_t lastTick = 0;
    uint32_t currentTick = HAL_GetTick();
    uint32_t tickDelta = currentTick - lastTick;
    
    if(tickDelta >= 1000) { // 每秒更新一次
        // 使用SysTick计数器计算更精确的FPS
        uint32_t cycleCount = (SystemCoreClock / 1000) * tickDelta;
        FPS = cycleCount / sysTickDelta;
        lastTick = currentTick;
    }
}

void LED_Indicate_Blink()
{
    uint8_t progress = led0_ticks / (HALF_LED_BLINK_PERIOD);
    if (progress < 1)
    {
        led0_state = 0U;
    }
    else if (progress >= 1 && progress < 2)
    {
        led0_state = 1;
    }
    else if (progress >= 2)
    {
        led0_ticks = 0U;
    }
    
    if (led0_state != last_led0_state)
    {
        last_led0_state = led0_state;
        // Set_LED_State(LED0_Pin, led0_state);
    }
}

void HAL_Delay_us(uint32_t nus)
{
    uint32_t told, tnow, tcnt = 0;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow;
            else
                tcnt += SysTick->LOAD - tnow + told;
            told = tnow;
            if (tcnt >= nus * (SystemCoreClock / 1000000))
                break;
        }
    };
}

void Restore_LED_State()
{
    for (int i = 0; i < DMA_Buffer_Resolution; i++)
    {
        // Restore Indicate LED State
        if (led0_state)
        {
            DMA_Buffer[0][i] &= ~LED0_Pin; 
        }
        else
        {
            DMA_Buffer[0][i] |= LED0_Pin; 
        }

        // Restore Calibration LED State
        if (Get_Calibration_Mode())
        {
            DMA_Buffer[0][i] &= ~LED1_Pin; 
        }
        else
        {
            DMA_Buffer[0][i] |= LED1_Pin; 

        }

        // Restore Plane LED State
        if (Get_Plane_Mode())
        {
            DMA_Buffer[0][i] &= ~LED2_Pin;
        }
        else
        {
            DMA_Buffer[0][i] |= LED2_Pin; 
        }
    }
}

void Set_LED_State(uint16_t pin, int state)
{
    for (int i = 0; i < DMA_Buffer_Resolution; i++)
    {
        if (state)
        {
            DMA_Buffer[0][i] &= ~pin; // LED 灭
        }
        else
        {
            DMA_Buffer[0][i] |= pin; // LED 亮
        }
    }
}

void Toggle_LED_State(uint16_t pin)
{
    for (int i = 0; i < DMA_Buffer_Resolution; i++)
    {
        DMA_Buffer[0][i] ^= pin; // LED 切换状态
    }
}

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;

static void Get_ADC1_Values(uint32_t *v33_raw, uint32_t *v50_raw)
{
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        *v33_raw = HAL_ADC_GetValue(&hadc1);
    }
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        *v50_raw = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
}

static void Get_ADC3_Values(uint32_t *temp_raw, uint32_t *vref_raw)
{
    HAL_ADC_Start(&hadc3);
    if (HAL_ADC_PollForConversion(&hadc3, 100) == HAL_OK)
    {
        *temp_raw = HAL_ADC_GetValue(&hadc3);
    }
    if (HAL_ADC_PollForConversion(&hadc3, 100) == HAL_OK)
    {
        *vref_raw = HAL_ADC_GetValue(&hadc3);
    }
    HAL_ADC_Stop(&hadc3);
}

float Get_Voltage_VDDA(void)
{
    uint32_t temp_raw = 0, vref_raw = 0;
    Get_ADC3_Values(&temp_raw, &vref_raw);
    
    // Calculate VDDA in mV
    uint32_t vdda_mv = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(vref_raw, ADC_RESOLUTION_16B);
    
    // Return Voltage in V
    return (float)vdda_mv / 1000.0f;
}

float Get_Voltage_3V3(void)
{
    // PA6 ADC1_INP3 Single-ended
    uint32_t v33_raw = 0, v50_raw = 0;
    uint32_t temp_raw = 0, vref_raw = 0;
    
    Get_ADC1_Values(&v33_raw, &v50_raw);
    Get_ADC3_Values(&temp_raw, &vref_raw);
    
    uint32_t vdda_mv = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(vref_raw, ADC_RESOLUTION_16B);
    uint32_t v_mv = __HAL_ADC_CALC_DATA_TO_VOLTAGE(vdda_mv, v33_raw, ADC_RESOLUTION_16B);
    
    return (float)v_mv / 1000.0f * 2.0f; // 10k + 10k divider
}

float Get_Voltage_5V0(void)
{
    // PA3 ADC1_INP15 Single-ended
    uint32_t v33_raw = 0, v50_raw = 0;
    uint32_t temp_raw = 0, vref_raw = 0;
    
    Get_ADC1_Values(&v33_raw, &v50_raw);
    Get_ADC3_Values(&temp_raw, &vref_raw);
    
    uint32_t vdda_mv = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(vref_raw, ADC_RESOLUTION_16B);
    uint32_t v_mv = __HAL_ADC_CALC_DATA_TO_VOLTAGE(vdda_mv, v50_raw, ADC_RESOLUTION_16B);
    
    return (float)v_mv / 1000.0f * 2.0f; // 10k + 10k divider
}



float Get_Temperature(void)
{
    uint32_t temp_raw = 0, vref_raw = 0;
    Get_ADC3_Values(&temp_raw, &vref_raw);
    
    // Calculate VDDA first as it is needed for Temperature calculation
    uint32_t vdda_mv = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(vref_raw, ADC_RESOLUTION_16B);
    
    // Calculate Temperature in Degree Celsius
    int32_t temperature = __HAL_ADC_CALC_TEMPERATURE(vdda_mv, temp_raw, ADC_RESOLUTION_16B);
    
    return (float)temperature;
}


