#include "calib_semi.h"
#include "calib_adc.h"
#include "dma_manager.h"
#include "calibration.h"
#include "eeprom.h"
#include "transducer.h"
#include <string.h>
#include <math.h>

volatile SemiCalibState calib_state = CALIB_IDLE;
volatile uint8_t        calib_current_element = 0;
CalibResult             calib_results[NUM_REAL_TRANSDUCER];
volatile uint8_t        calib_results_valid[NUM_REAL_TRANSDUCER];
volatile uint8_t        calib_button_pressed = 0;

static uint32_t measure_start_time;

void SemiCalib_Init(void)
{
    memset(calib_results, 0, NUM_REAL_TRANSDUCER * sizeof(CalibResult));
    memset((void *)calib_results_valid, 0, NUM_REAL_TRANSDUCER * sizeof(uint8_t));
    calib_state           = CALIB_IDLE;
    calib_current_element = 0;
    calib_button_pressed  = 0;
}

SemiCalibState SemiCalib_Tick(void)
{
    extern ADC_HandleTypeDef hadc1;
    uint32_t now = HAL_GetTick();

    switch (calib_state) {

    case CALIB_IDLE:
        break;

    case CALIB_PROMPT:
        if (calib_button_pressed == 1) {
            /* CONFIRM: start measurement */
            calib_button_pressed = 0;

            Calib_SetSingleTransducer(calib_current_element);
            Calib_ADC_Configure();
            adc_capture_done = 0;

            __HAL_TIM_SET_COUNTER(&htim6, 0);
            HAL_TIM_Base_Start(&htim6);

            HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, CALIB_ADC_BUFFER_SIZE);

            measure_start_time = now;
            calib_state = CALIB_MEASURING;
        }
        else if (calib_button_pressed == 2) {
            /* RETURN: skip this element */
            calib_button_pressed = 0;
            calib_results_valid[calib_current_element] = 0;
            calib_results[calib_current_element].calib_us = 0.0f;
            calib_results[calib_current_element].quality   = 0;

            if (calib_current_element < NUM_REAL_TRANSDUCER - 1) {
                calib_current_element++;
            } else {
                calib_state = CALIB_DONE;
            }
        }
        break;

    case CALIB_MEASURING:
        if (adc_capture_done) {
            float amp;
            float phase = Calib_IQ_Demodulate(adc_buffer, CALIB_ADC_BUFFER_SIZE, &amp);
            float calib_us = Calib_PhaseToMicroseconds(phase);

            calib_results[calib_current_element].calib_us  = calib_us;
            calib_results[calib_current_element].amplitude = amp;
            if (amp >= CALIB_AMP_GOOD)
                calib_results[calib_current_element].quality = 2;
            else if (amp >= CALIB_AMP_MARGINAL)
                calib_results[calib_current_element].quality = 1;
            else
                calib_results[calib_current_element].quality = 0;
            calib_results_valid[calib_current_element] = 1;

            HAL_ADC_Stop_DMA(&hadc1);
            HAL_TIM_Base_Stop(&htim6);
            Calib_ADC_Deinit();

            calib_state = CALIB_SHOW_RESULT;
        }
        else if (now - measure_start_time > CALIB_MEASURE_TIMEOUT_MS) {
            /* Timeout: mark as failed */
            HAL_ADC_Stop_DMA(&hadc1);
            HAL_TIM_Base_Stop(&htim6);
            Calib_ADC_Deinit();

            calib_results[calib_current_element].quality = 0;
            calib_results_valid[calib_current_element] = 0;
            calib_state = CALIB_SHOW_RESULT;
        }
        break;

    case CALIB_SHOW_RESULT:
        if (calib_button_pressed == 1) {
            /* CONFIRM: accept and advance */
            calib_button_pressed = 0;
            if (calib_current_element < NUM_REAL_TRANSDUCER - 1) {
                calib_current_element++;
                calib_state = CALIB_PROMPT;
            } else {
                calib_state = CALIB_DONE;
            }
        }
        else if (calib_button_pressed == 2) {
            /* RETURN: retry same element */
            calib_button_pressed = 0;
            calib_state = CALIB_PROMPT;
        }
        break;

    case CALIB_DONE: {
        float calib_array[NUM_REAL_TRANSDUCER];
        for (int i = 0; i < NUM_REAL_TRANSDUCER; i++)
            calib_array[i] = calib_results[i].calib_us;

        memcpy(Transducer_Calibration_Array, calib_array, sizeof(float) * NUM_REAL_TRANSDUCER);
        EEPROM_SaveCalibration(calib_array);

        Calib_SetNormalDrive();
        Load_Calib_to_Transducers();

        /* Stop TIM6 and leave ADC1 in polling mode */
        HAL_TIM_Base_Stop(&htim6);
        Calib_ADC_Deinit();

        calib_state = CALIB_IDLE;
        break;
    }

    default:
        break;
    }

    return calib_state;
}
