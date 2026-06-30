# define _USE_MATH_DEFINES
# include "transducer.h"
# include "calibration.h"
# include "dma_manager.h"
# include "communication.h"
# include "custom_math.h"

float Wave_K = ((2.0*M_PI*TRANSDUCER_BASE_FREQ)/SPEED_OF_SOUND);

// Transducer Array — V5.5 同心圆环 (6/12/18/24 = 60) + 虚拟 + 2 触发
const char *TransducerPins[] =
{
    // Ring 0 (6 阵元, r=12.6mm, 首阵元在 0°/+X 轴)
    "PD1", "PB6", "PB7",  "PC0", "PE8",  "PD13",

    // Ring 1 (12 阵元, r=25.2mm, 整环旋转 137.5°)
    "PB9", "PC1", "PC2", "PE9", "PE10",
    "PD11", "PD12", "PC12", "PD0", "PB4", "PB5",  "PB8",

    // Ring 2 (18 阵元, r=37.8mm, 整环旋转 2×137.5°)
    "PD8", "PD9", "PD10", "PC8", "PC10", "PC11",
    "PD6", "PD7", "PB3", "PE0", "PE1", "PE2",
    "PC3", "PC4", "PC5", "PE11", "PE12", "PE13",

    // Ring 3 (24 阵元, r=50.4mm, 整环旋转 3×137.5°)
    "PE15", "PB10", "PB11", "PB12", "PB13", "PB14",
    "PB15", "PD14", "PD15", "PC6", "PC7", "PD2",
    "PD3", "PD4", "PD5", "PE3", "PE4", "PE5",
    "PE6", "PB0", "PB1", "PB2", "PE7", "PE14",

    // VIRTUALTRANSDUCER — PC13, 40kHz 相位参考
    "PC13",

    // TRIGGER0 — PC14, 200Hz 周期开头 1ms 脉冲
    "PC14",

    // TRIGGER1 — PC15, 可配置 (默认全0)
    "PC15"
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
        t->port      = map_pin_name_to_gpio_port(TransducerPins[i]);
        t->port_num  = map_pin_name_to_gpio_port_num(TransducerPins[i]);
        t->pin       = map_pin_name_to_pin_number(TransducerPins[i]);
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

GPIO_TypeDef *map_pin_name_to_gpio_port(const char *pin_name)
{
    if (pin_name == NULL)
        return NULL;

    switch (pin_name[1])
    {
    case 'A':
        return GPIOA;
    case 'B':
        return GPIOB;
    case 'C':
        return GPIOC;
    case 'D':
        return GPIOD;
    case 'E':
        return GPIOE;
    case 'F':
        return GPIOF;
    default:
        return GPIOA;
    }
}

static const uint16_t port_num_map[] = {
    ['A']=0xFF, ['B']=0, ['C']=1, ['D']=2, ['E']=3
};

uint8_t map_pin_name_to_gpio_port_num(const char *pin) {
    return port_num_map[(int)pin[1]];
}

uint16_t map_pin_name_to_pin_number(const char *pin_name)
{

    if (pin_name == NULL)
        return 0;

    char pin_number_str[4];
    strncpy(pin_number_str, &pin_name[2], 3);
    pin_number_str[3] = '\0';

    int pin_number = atoi(pin_number_str);
    switch (pin_number)
    {
    case 0:
        return GPIO_PIN_0;
    case 1:
        return GPIO_PIN_1;
    case 2:
        return GPIO_PIN_2;
    case 3:
        return GPIO_PIN_3;
    case 4:
        return GPIO_PIN_4;
    case 5:
        return GPIO_PIN_5;
    case 6:
        return GPIO_PIN_6;
    case 7:
        return GPIO_PIN_7;
    case 8:
        return GPIO_PIN_8;
    case 9:
        return GPIO_PIN_9;
    case 10:
        return GPIO_PIN_10;
    case 11:
        return GPIO_PIN_11;
    case 12:
        return GPIO_PIN_12;
    case 13:
        return GPIO_PIN_13;
    case 14:
        return GPIO_PIN_14;
    case 15:
        return GPIO_PIN_15;
    default:
        return GPIO_PIN_0;
    }
}
