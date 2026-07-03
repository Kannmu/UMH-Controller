# define _USE_MATH_DEFINES
# include "transducer.h"
# include "calibration.h"
# include "dma_manager.h"
# include "communication.h"
# include "custom_math.h"

const float Wave_K = ((2.0*M_PI*TRANSDUCER_BASE_FREQ)/SPEED_OF_SOUND);

// Transducer Array — V5.5 同心圆环 (6/12/18/24 = 60) + 虚拟 + 2 触发
const TransducerPinDef TransducerPins[NUM_TOTAL_CHANNELS] = {
    /* Ring 0 (6 elements, r=12.6mm) */
    {GPIOD, 2, GPIO_PIN_1},
    {GPIOB, 0, GPIO_PIN_6},
    {GPIOB, 0, GPIO_PIN_7},
    {GPIOC, 1, GPIO_PIN_0},
    {GPIOE, 3, GPIO_PIN_8},
    {GPIOD, 2, GPIO_PIN_13},
    /* Ring 1 (12 elements, r=25.2mm) */
    {GPIOB, 0, GPIO_PIN_9},
    {GPIOC, 1, GPIO_PIN_1},
    {GPIOC, 1, GPIO_PIN_2},
    {GPIOE, 3, GPIO_PIN_9},
    {GPIOE, 3, GPIO_PIN_10},
    {GPIOD, 2, GPIO_PIN_11},
    {GPIOD, 2, GPIO_PIN_12},
    {GPIOC, 1, GPIO_PIN_12},
    {GPIOD, 2, GPIO_PIN_0},
    {GPIOB, 0, GPIO_PIN_4},
    {GPIOB, 0, GPIO_PIN_5},
    {GPIOB, 0, GPIO_PIN_8},
    /* Ring 2 (18 elements, r=37.8mm) */
    {GPIOD, 2, GPIO_PIN_8},
    {GPIOD, 2, GPIO_PIN_9},
    {GPIOD, 2, GPIO_PIN_10},
    {GPIOC, 1, GPIO_PIN_8},
    {GPIOC, 1, GPIO_PIN_10},
    {GPIOC, 1, GPIO_PIN_11},
    {GPIOD, 2, GPIO_PIN_6},
    {GPIOD, 2, GPIO_PIN_7},
    {GPIOB, 0, GPIO_PIN_3},
    {GPIOE, 3, GPIO_PIN_0},
    {GPIOE, 3, GPIO_PIN_1},
    {GPIOE, 3, GPIO_PIN_2},
    {GPIOC, 1, GPIO_PIN_3},
    {GPIOC, 1, GPIO_PIN_4},
    {GPIOC, 1, GPIO_PIN_5},
    {GPIOE, 3, GPIO_PIN_11},
    {GPIOE, 3, GPIO_PIN_12},
    {GPIOE, 3, GPIO_PIN_13},
    /* Ring 3 (24 elements, r=50.4mm) */
    {GPIOE, 3, GPIO_PIN_15},
    {GPIOB, 0, GPIO_PIN_10},
    {GPIOB, 0, GPIO_PIN_11},
    {GPIOB, 0, GPIO_PIN_12},
    {GPIOB, 0, GPIO_PIN_13},
    {GPIOB, 0, GPIO_PIN_14},
    {GPIOB, 0, GPIO_PIN_15},
    {GPIOD, 2, GPIO_PIN_14},
    {GPIOD, 2, GPIO_PIN_15},
    {GPIOC, 1, GPIO_PIN_6},
    {GPIOC, 1, GPIO_PIN_7},
    {GPIOD, 2, GPIO_PIN_2},
    {GPIOD, 2, GPIO_PIN_3},
    {GPIOD, 2, GPIO_PIN_4},
    {GPIOD, 2, GPIO_PIN_5},
    {GPIOE, 3, GPIO_PIN_3},
    {GPIOE, 3, GPIO_PIN_4},
    {GPIOE, 3, GPIO_PIN_5},
    {GPIOE, 3, GPIO_PIN_6},
    {GPIOB, 0, GPIO_PIN_0},
    {GPIOB, 0, GPIO_PIN_1},
    {GPIOB, 0, GPIO_PIN_2},
    {GPIOE, 3, GPIO_PIN_7},
    {GPIOE, 3, GPIO_PIN_14},
    /* VIRTUALTRANSDUCER — PC13 */
    {GPIOC, 1, GPIO_PIN_13},
    /* TRIGGER0 — PC14 */
    {GPIOC, 1, GPIO_PIN_14},
    /* TRIGGER1 — PC15 */
    {GPIOC, 1, GPIO_PIN_15},
};

Transducer TransducerArray[NUM_TOTAL_CHANNELS];

void Transducer_Init(void)
{
    static const uint8_t ring_counts[NUM_RINGS] = {6, 12, 18, 24};
    static const float   ring_radius[NUM_RINGS] = {12.6e-3f, 25.2e-3f, 37.8e-3f, 50.4e-3f};

    for (size_t i = 0; i < NUM_TOTAL_CHANNELS; i++)
    {
        Transducer *t = &TransducerArray[i];
        t->index     = (uint8_t)i;
        t->port      = TransducerPins[i].port;
        t->port_num  = TransducerPins[i].port_num;
        t->pin       = TransducerPins[i].pin;
        t->calib     = (i < NUM_REAL_TRANSDUCER)
                       ? (uint16_t)(Transducer_Calibration_Array[i] * BufferGapPerMicroseconds)
                       : 0;
        t->position3D[2] = 0.0f;   // Z=0 (阵列平面)
        t->distance  = 0;
        t->phase     = 0;
        t->duty      = DMA_DUTY_CYCLE_MAX;
        t->shift_buffer_bits = 0;

        if (i < NUM_REAL_TRANSDUCER)
        {
            // 求所在圆环 r 与环内序号 k (从内向外, 逆时针编号)
            uint8_t r = 0, k = (uint8_t)i;
            while (r < NUM_RINGS && k >= ring_counts[r])
            {
                k -= ring_counts[r];
                r++;
            }
            float angle = (float)r * GOLDEN_ANGLE_RAD
                          + (float)k * (2.0f * (float)M_PI) / (float)ring_counts[r];
            t->ring       = r;
            t->ring_index = k;
            t->position3D[0] = ring_radius[r] * cosf(angle);   // X
            t->position3D[1] = ring_radius[r] * sinf(angle);   // Y
        }
        else
        {
            // 特殊通道 (虚拟/触发): 位置置 0
            t->ring       = 0;
            t->ring_index = 0;
            t->position3D[0] = 0.0f;
            t->position3D[1] = 0.0f;
        }
    }
}

void Enter_Calibration_Mode()
{
    for (int i = 0; i < NUM_REAL_TRANSDUCER; i++)
    {
        TransducerArray[i].calib = 0;
        TransducerArray[i].phase = 0;
        TransducerArray[i].shift_buffer_bits = 0;
    }
}

void Load_Calib_to_Transducers()
{
    for (int i = 0; i < NUM_REAL_TRANSDUCER; i++)
    {
        TransducerArray[i].calib = (uint16_t)(Transducer_Calibration_Array[i] * BufferGapPerMicroseconds);
    }
}

// Update Point to Transducers Parameters
void Set_Point_Focus(float *position)
{
    for (int i = 0; i < NUM_REAL_TRANSDUCER; i++)
    {
        // Distance Calculation
        TransducerArray[i].distance = Euler_Distance(TransducerArray[i].position3D, position);

        // Distance to Phase
        TransducerArray[i].phase = Distance_to_Phase(TransducerArray[i].distance);

        // Phase to Gap Ticks
        TransducerArray[i].shift_buffer_bits = Phase_to_Gap_Ticks(TransducerArray[i].phase);
    }
}

void Set_Twin_Trap_Focus(float *position)
{
    for (int i = 0; i < NUM_REAL_TRANSDUCER; i++)
    {
        // 1. Focusing Lens Phase
        TransducerArray[i].distance = Euler_Distance(TransducerArray[i].position3D, position);
        float phi_focus = Distance_to_Phase(TransducerArray[i].distance);

        // 2. Twin Signature (基于几何位置分割)
        float phi_twin = (TransducerArray[i].position3D[1] > 0) ? 0.0f : (float)M_PI;

        // 3. Final Phase
        TransducerArray[i].phase = fmod(phi_focus + phi_twin, 2.0 * M_PI);

        // Phase to Gap Ticks
        TransducerArray[i].shift_buffer_bits = Phase_to_Gap_Ticks(TransducerArray[i].phase);
    }
}

// Set Phases and Duty Cycles to Transducers
void Set_Transducers(uint8_t *data)
{
    for (int i = 0; i < NUM_REAL_TRANSDUCER; i++)
    {
        uint16_t phase_raw = data[i * SERIAL_TRANSDUCER_BYTES + 0] | (data[i * SERIAL_TRANSDUCER_BYTES + 1] << 8);
        uint8_t duty_raw = data[i * SERIAL_TRANSDUCER_BYTES + 2];

        TransducerArray[i].phase = (phase_raw / 65535.0f) * (2.0f * M_PI);
        TransducerArray[i].duty = duty_raw / 255.0f;
        TransducerArray[i].shift_buffer_bits = Phase_to_Gap_Ticks(TransducerArray[i].phase);
    }
    Update_Full_Waveform_Buffer();
}

float Distance_to_Phase(float distance)
{
    // return fmod((distance * Wave_K), (2.0 * M_PI));
    // return (2.0 * M_PI) - (fmod((distance * Wave_K), (2.0 * M_PI)));
    float phase = distance * Wave_K;
    float twopi = 2.0f * (float)M_PI;
    int quotient = (int)(phase / twopi);
    return phase - (float)quotient * twopi;
}

float Phase_to_Gap_Ticks(float phase)
{
    return (phase / (2.0 * M_PI * TRANSDUCER_BASE_FREQ)) / TIME_GAP_PER_DMA_BUFFER_BIT;
}
