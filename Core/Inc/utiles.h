#pragma once
#include "main.h"

/* HEARTBEAT: PA15, sigma‑delta modulated PWM driven by a 256‑entry sin² brightness LUT.
   Breathing effect (~14.6 BPM, 4.1 s cycle). Direct GPIO access is safe (no DMA on Port A). */

extern uint32_t sysTickDelta;
extern float System_Loop_Freq;
extern double updateDMABufferDeltaTime;

/* Per-type DMA buffer update time. Bounded by STIM_MAX_TYPES; indexed by
 * Stim_Get_Index_By_Type_Id(). Sized to cover the linker-section registry
 * (currently 5 types); avoids runtime heap allocation. */
#define STIM_MAX_TYPES 16
extern double updateDMABufferDeltaTimeByType[STIM_MAX_TYPES];

void Init_DWT(void);
uint32_t DWT_GetCycles(void);
uint32_t DWT_GetMicroseconds(void);
void Update_LED_Status(void);

char* Get_Device_Serial_Number(void);
float Get_Voltage_VDDA(void);

float Get_Temperature(void);
float Get_Refresh_Rate(void);
