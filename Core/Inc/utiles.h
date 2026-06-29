#pragma once
#include "main.h"

/* HEARTBEAT: PA15, sigma‑delta modulated PWM driven by a 256‑entry sin² brightness LUT.
   Breathing effect (~14.6 BPM, 4.1 s cycle). Direct GPIO access is safe (no DMA on Port A). */

extern uint32_t sysTickDelta;
extern float System_Loop_Freq;
extern double updateDMABufferDeltaTime;
extern double updateDMABufferDeltaTimeByType[5];

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
