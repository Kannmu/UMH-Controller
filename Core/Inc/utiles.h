#pragma once
#include "main.h"
#include "dma_manager.h"

/* HEARTBEAT: PA15 (main.h HEARTBEAT_Pin), 由 Port A DMA 缓冲的 led_mask 机制驱动。 */

// Debug Parameters
extern uint32_t sysTickDelta;
extern float System_Loop_Freq;
extern double updateDMABufferDeltaTime;

void Init_DWT(void);
uint32_t DWT_GetCycles(void);
uint32_t DWT_GetMicroseconds(void);
void Update_LED_Status(void);
uint16_t Get_Current_LED_Mask(void);

char* Get_Device_Serial_Number(void);
float Get_Voltage_VDDA(void);
float Get_Voltage_3V3(void);
float Get_Voltage_5V0(void);
float Get_Temperature(void);
