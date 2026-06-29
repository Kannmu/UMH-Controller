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
CalibResult             calib_results[60];
volatile uint8_t        calib_results_valid[60];
volatile uint8_t        calib_button_pressed = 0;

static uint32_t measure_start_time;

void SemiCalib_Init(void)
{
    memset(calib_results, 0, sizeof(calib_results));
    memset((void *)calib_results_valid, 0, sizeof(calib_results_valid));
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

            HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, 8000);

            measure_start_time = now;
            calib_state = CALIB_MEASURING;
        }
        else if (calib_button_pressed == 2) {
            /* RETURN: skip this element */
            calib_button_pressed = 0;
            calib_results_valid[calib_current_element] = 0;
            calib_results[calib_current_element].calib_us = 0.0f;
            calib_results[calib_current_element].quality   = 0;

            if (calib_current_element < 59) {
                calib_current_element++;
            } else {
                calib_state = CALIB_DONE;
            }
        }
        break;

    case CALIB_MEASURING:
        if (adc_capture_done) {
            float amp;
            float phase = Calib_IQ_Demodulate(adc_buffer, 8000, &amp);
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
        else if (now - measure_start_time > 50) {
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
            if (calib_current_element < 59) {
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
        float calib_array[60];
        for (int i = 0; i < 60; i++)
            calib_array[i] = calib_results[i].calib_us;

        /* Copy to the live calibration array, then persist to EEPROM.
         * EEPROM shares I2C3 with OLED; GUI rendering is paused during this call
         * because SemiCalib_Tick runs inside GUI_Tick — no concurrent I2C access. */
        memcpy(Transducer_Calibration_Array, calib_array, sizeof(float) * 60);
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
