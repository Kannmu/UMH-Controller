#define _USE_MATH_DEFINES
#include <stdio.h>
#include "utiles.h"
#include "stimulation.h"
#include "calibration.h"
#include "dma_manager.h" // For Waveform_Storage if needed, but we removed Set_LED_State

// debug.c
const uint16_t HALF_LED_BLINK_PERIOD = 500U;
uint16_t led0_ticks = 0;
uint32_t sysTickDelta= 0;
uint32_t FPS = 0;
// int led0_state = 0; // Removed
// int last_led0_state = 0; // Removed
float System_Loop_Freq = 0.0f;
double updateDMABufferDeltaTime = 0;

static int led2_active_state = 0;

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

static uint16_t last_led_mask = 0xFFFF; // Default All OFF (Active Low)

void Update_LED_Status(void)
{
    uint32_t now = HAL_GetTick();

    // --- LED0 Logic (1Hz Heartbeat) ---
    // 500ms ON, 500ms OFF
    // Active Low: 0 is ON
    int led0_on = (now % 1000) < 500;
    
    // Update legacy led0_ticks for compatibility if needed (0-999)
    led0_ticks = (uint16_t)(now % 1000);

    // --- LED1 Logic (Calibration Mode) ---
    int led1_on = Get_Calibration_Mode();

    // --- LED2 Logic (Demo Mode 4-bit) ---
    static int led2_on = 0;
    static uint32_t led2_next_event = 0;
    static int led2_bit_index = 3; // Start from MSB (Bit 3)
    static int led2_seq_state = 0; // 0: Prepare Bit, 1: Bit Active, 2: Inter-bit Gap
    
    if (now >= led2_next_event)
    {
        int mode = Get_Demo_Mode();
        
        switch (led2_seq_state)
        {
            case 0: // Prepare to show bit
                if (led2_bit_index < 0)
                {
                    // Sequence finished
                    led2_on = 0;
                    led2_next_event = now + 1500; // Long gap between cycles
                    led2_bit_index = 3; // Reset to MSB
                    // State remains 0, next time will start sequence
                }
                else
                {
                    // Start showing bit
                    led2_on = 1;
                    int bit = (mode >> led2_bit_index) & 1;
                    // 0: Short (200ms), 1: Long (600ms)
                    led2_next_event = now + (bit ? 600 : 200); 
                    led2_seq_state = 1;
                }
                break;
                
            case 1: // Bit finished, turn off
                led2_on = 0;
                led2_next_event = now + 300; // Gap between bits
                led2_bit_index--;
                led2_seq_state = 0;
                break;
        }
    }
    
    // Update legacy led2_active_state
    led2_active_state = led2_on;

    // --- Construct Mask ---
    uint16_t mask = 0;
    
    // Active Low Logic: 
    // Pin Bit 0 -> ON
    // Pin Bit 1 -> OFF
    
    if (!led0_on) mask |= LED0_Pin;
    if (!led1_on) mask |= LED1_Pin;
    if (!led2_on) mask |= LED2_Pin;

    // --- Update DMA if changed ---
    if (mask != last_led_mask)
    {
        DMA_Update_LED_State(mask);
        last_led_mask = mask;
    }
}

uint16_t Get_Current_LED_Mask(void)
{
    // Return the cached calculated mask
    // If Update_LED_Status hasn't been called yet, calculate it or return default
    if (last_led_mask == 0xFFFF && HAL_GetTick() < 100) 
    {
         // Fallback for early call before first Update loop
         return 0; // All ON? Or use default logic. 
         // Safest is to just call Update_LED_Status logic once or return a safe value.
         // Let's just return last_led_mask which defaults to All OFF.
    }
    return last_led_mask;
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

char* Get_Device_Serial_Number(void)
{
    static char serial_str[25]; // 96 bits = 12 bytes = 24 hex chars + 1 null terminator
    
    uint32_t uid0 = HAL_GetUIDw0();
    uint32_t uid1 = HAL_GetUIDw1();
    uint32_t uid2 = HAL_GetUIDw2();

    // Format as a 24-character hex string
    snprintf(serial_str, sizeof(serial_str), "%08lX%08lX%08lX", (unsigned long)uid0, (unsigned long)uid1, (unsigned long)uid2);

    return serial_str;
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
