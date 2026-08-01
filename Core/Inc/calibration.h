#pragma once
#include "main.h"
#include "transducer.h"

// calibration.h
/* calibration_mode is written by the main loop and read from the USB RX ISR
 * response path; declared volatile for cross-context visibility. */
extern volatile int calibration_mode;
extern float Transducer_Calibration_Array[];



int Get_Calibration_Mode(void);

