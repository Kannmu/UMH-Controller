#pragma once
#include "main.h"
#include "transducer.h"

typedef enum {
    CALIB_IDLE = 0,
    CALIB_PROMPT,
    CALIB_MEASURING,
    CALIB_SHOW_RESULT,
    CALIB_DONE,
    CALIB_ERROR
} SemiCalibState;

#define CALIB_AMP_GOOD     0.003f
#define CALIB_AMP_MARGINAL 0.001f
#define CALIB_MEASURE_TIMEOUT_MS 50U

typedef struct {
    float    calib_us;
    float    amplitude;
    uint8_t  quality;  /* 2=good, 1=marginal, 0=bad */
} CalibResult;

extern volatile SemiCalibState calib_state;
extern volatile uint8_t        calib_current_element;
extern CalibResult             calib_results[NUM_REAL_TRANSDUCER];
extern volatile uint8_t        calib_results_valid[NUM_REAL_TRANSDUCER];
extern volatile uint8_t        calib_button_pressed;

void SemiCalib_Init(void);
SemiCalibState SemiCalib_Tick(void);
