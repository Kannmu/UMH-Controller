#pragma once
#include "main.h"
#include "stimulation.h"

# define ARRAY_SIZE 5
# define TRANSDUCER_SIZE (10.0e-3)
# define TRANSDUCER_SPACING (10.0e-3)

# define NUM_TRANSDUCER 61U // The first 60 is real transducers, the last one is virtual transducer for calibration

# define SPEED_OF_SOUND 343.2

# define TRANSDUCER_BASE_FREQ 40000UL

# define TRANSDUCER_PERIOD  ((long double)(1.0 / TRANSDUCER_BASE_FREQ))

# define WAVE_LENGTH (TRANSDUCER_PERIOD*SPEED_OF_SOUND)

typedef struct Stimulation Stimulation;

// Transducer Class
typedef struct Transducer
{
    uint8_t index;
    uint8_t row;
    uint8_t column;
    float position3D[3];

    GPIO_TypeDef *port;
    uint8_t port_num;
    uint16_t pin;
    uint16_t calib;

    float distance;
    float phase;
    uint16_t shift_buffer_bits;

    float duty;
} Transducer;


extern const char *TransducerPins[];
extern Transducer TransducerArray[NUM_TRANSDUCER];


extern float Wave_K;

void Transducer_Init(void);
void Enter_Calibration_Mode(void);
void Load_Calib_to_Transducers(void);

void Set_Point_Focus(float *position);
void Set_Twin_Trap_Focus(float *position);
void Set_Phases(float phases[]);
void Set_Plane_Wave(void);

float Distance_to_Phase(float distance);
float Phase_to_Gap_Ticks(float phase);

GPIO_TypeDef *map_pin_name_to_gpio_port(const char *);
uint8_t map_pin_name_to_gpio_port_num(const char *);
uint16_t map_pin_name_to_pin_number(const char *);

