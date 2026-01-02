# define _USE_MATH_DEFINES
# include "transducer.h"
# include "calibration.h"
# include "dma_manager.h"
# include "custom_math.h"

float Wave_K = ((2.0*M_PI*Transducer_Base_Freq)/SoundSpeed);

// Transducer Array
const char *TransducerPins[] =
    {
        "PB8", "PE1", "PE2", "PE3", "PE5", 
        
        "PD7", "PB7", "PE0", "PE4", "PE6", "PC15", 
        
        "PD3", "PD6", "PB6", "PB9", "PC13", "PC0", "PC3",

        "PD1", "PD2", "PD5", "PB5", "PC14", "PC1", "PA0", "PA1",

        "PC8", "PD0", "PC6", "PD4", "PC2", "PC5", "PA2", "PA7",

        "PC7", "PD15", "PD13", "PD10", "PE9", "PB2", "PB0", "PC4",

        "PD14", "PD12", "PD9", "PE13", "PE10", "PE7", "PB1",

        "PD11", "PD8", "PB14", "PE14", "PE11", "PE8",

        "PB15", "PB13", "PB12", "PE15", "PE12",

        "PC10"};

Transducer TransducerArray[NumTransducer];

void Transducer_Init(void)
{
    const int row_lengths[] = {5, 6, 7, 8, 9, 8, 7, 6, 5};
    const float dy = TransducerGap * 0.86602540378f; // sqrt(3)/2

    for (size_t i = 0; i < NumTransducer; i++)
    {
        // TransducerArray[i] = (Transducer *)malloc(sizeof(Transducer));
        TransducerArray[i].index = i;
        TransducerArray[i].port = map_pin_name_to_gpio_port(TransducerPins[i]);
        TransducerArray[i].port_num = map_pin_name_to_gpio_port_num(TransducerPins[i]);
        TransducerArray[i].pin = map_pin_name_to_pin_number(TransducerPins[i]);
        TransducerArray[i].calib = Transducer_Calibration_Array[i] * BufferGapPerMicroseconds;

        if (i < 60)
        {
            int r = 0, k = i;
            for (r = 0; r < 9; r++)
            {
                int row_count = (r == 4) ? 8 : row_lengths[r];
                if (k < row_count)
                    break;
                k -= row_count;
            }
            int j = (r == 4 && k >= 4) ? k + 1 : k;

            TransducerArray[i].row = r;
            TransducerArray[i].column = j;
            TransducerArray[i].position3D[0] = (j - (row_lengths[r] - 1.0f) / 2.0f) * TransducerGap; // X
            TransducerArray[i].position3D[1] = (4 - r) * dy;                                        // Y
        }
        else
        {
            TransducerArray[i].row = 0;
            TransducerArray[i].column = 0;
            TransducerArray[i].position3D[0] = 0;
            TransducerArray[i].position3D[1] = 0;
        }

        TransducerArray[i].position3D[2] = 0; // Z
        TransducerArray[i].distance = 0;
        TransducerArray[i].phase = 0;
        TransducerArray[i].duty = 0.5;
        TransducerArray[i].shift_buffer_bits = 0;
    }
}

void Clean_Transducers_Calib()
{
    for (int i = 0; i < NumTransducer-1; i++)
    {
        TransducerArray[i].calib = 0;
    }
}

void Set_Transducers_Calib()
{
    for (int i = 0; i < NumTransducer-1; i++)
    {
        TransducerArray[i].calib = Transducer_Calibration_Array[i] * BufferGapPerMicroseconds;
    }
}

void Set_Plane_Wave()
{
    for (int i = 0; i < NumTransducer-1; i++)
    {
        TransducerArray[i].phase = 0;
        TransducerArray[i].shift_buffer_bits = 0;
    }
}

// Update Point to Transducers Parameters
void Set_Point_Focus(float *position)
{
    for (int i = 0; i < NumTransducer-1; i++)
    {
        // Distance Calculation
        TransducerArray[i].distance = Euler_Distance(TransducerArray[i].position3D, position);

        // Distance to Phase
        TransducerArray[i].phase = Distance_to_Phase(TransducerArray[i].distance);

        // Phase to Gap Ticks
        TransducerArray[i].shift_buffer_bits = Phase_to_Gap_Ticks(TransducerArray[i].phase);
    }
}

// Set Phases to Transducers
void Set_Phases(float phases[])
{
    for (int i = 0; i < NumTransducer-1; i++)
    {
        TransducerArray[i].phase = phases[i];
        TransducerArray[i].shift_buffer_bits = Phase_to_Gap_Ticks(TransducerArray[i].phase);
    }
}

float Distance_to_Phase(float distance)
{
    return (2.0 * M_PI) - (fmod((distance * Wave_K), (2.0 * M_PI)));
}

float Phase_to_Gap_Ticks(float phase)
{
    return (phase / (2.0 * M_PI * Transducer_Base_Freq)) / TimeGapPerDMABufferBit;
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
    ['A']=0, ['B']=1, ['C']=2, ['D']=3, ['E']=4
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
