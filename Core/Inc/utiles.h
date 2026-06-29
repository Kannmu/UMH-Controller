#pragma once
#include "main.h"

# define Heartbeat_Interval_ms 500

/* HEARTBEAT: PA15, direct GPIO toggle (no DMA — only Ports B/C/D/E use DMA for transducers). */

extern uint32_t sysTickDelta;
extern float System_Loop_Freq;
extern double updateDMABufferDeltaTime;

void Init_DWT(void);
uint32_t DWT_GetCycles(void);
uint32_t DWT_GetMicroseconds(void);
void Update_LED_Status(void);

char* Get_Device_Serial_Number(void);
float Get_Voltage_VDDA(void);
float Get_Voltage_3V3(void);
float Get_Voltage_5V0(void);
float Get_Temperature(void);
float Get_Refresh_Rate(void);
