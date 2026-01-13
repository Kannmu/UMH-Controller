# pragma once
# include "main.h"
# include "transducer.h"

# define STIMULATION_FREQ (200U)
# define STIMULATION_PERIOD (1.0 / STIMULATION_FREQ)
# define NUM_STIMULATION_SAMPLES (uint32_t)(TRANSDUCER_BASE_FREQ / STIMULATION_FREQ)

typedef enum StimulationType
{
    PointStimulation = 0,
    VibrationStimulation = 1,
    Linear = 2,
    Circular = 3,
}StimulationType;


typedef struct Stimulation
{
  // Stimulation Parameters
  StimulationType type; // Type of Stimulation

  // Point Stimulation Parameters
  float position[3]; // X Axis is Along Row, Y Axis Along Column, and Z Axis Target Outside Direction of the Array. In Meters. For Point Stimulation.

  // Vibration Stimulation Parameters
  float startPoint[3]; // Start Point for Vibration
  float endPoint[3]; // End Point for Vibration
  
  // Circular Stimulation Parameters
  float centerPoint[3]; // Center Point for Circular Stimulation. In Meters.
  float normalVector[3]; // Normal Vector for Circular Stimulation. In Meters.
  float radius; // Radius for Circular Stimulation. In Meters.

  // General Parameters
  float strength;     // Overall strength Coefficient, Default to 100
  float frequency;

  float progress; // from 0 to 1.

  // Cached Values (Internal Use)
  uint32_t cached_period_us;
  float cached_u[3];
  float cached_v[3];
}Stimulation;

extern int phase_set_mode;

extern Stimulation CurrentStimulation;

extern Stimulation EmptyStimulation;

int Get_Phase_Set_Mode(void);
void Set_Stimulation(Stimulation *stimulation);
void Update_Stimulation_State(float progress);
