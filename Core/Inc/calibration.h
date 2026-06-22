#pragma once
#include "main.h"
#include "transducer.h"

// calibration.h
extern volatile int calibration_mode;
extern float Transducer_Calibration_Array[];



void Calibration_Init(void);
void Switch_Calibration_Mode(void);
int Get_Calibration_Mode(void);

