# pragma once
# include "main.h"
# include "transducer.h"

# define STIMULATION_FREQ (200U)
# define STIMULATION_PERIOD (1.0 / STIMULATION_FREQ)
# define NUM_STIMULATION_SAMPLES (uint32_t)(TRANSDUCER_BASE_FREQ / STIMULATION_FREQ)


typedef enum StimulationType
{
    Point = 0,
    Discrete = 1,
    Linear = 2,
    Circular = 3,
    TwinTrap = 4,
}StimulationType;


typedef struct Stimulation
{
  char name[32];
  // Stimulation Parameters
  StimulationType type; // Type of Stimulation

  // Point Stimulation Parameters
  float position[3]; // X Axis is Along Row, Y Axis Along Column, and Z Axis Target Outside Direction of the Array. In Meters. For Point Stimulation.

  // Stimulation Parameters
  float startPoint[3]; // Start Point
  float endPoint[3]; // End Point
  
  int segments; // Number of Segments for Linear and Discrete Stimulation

  // Circular Stimulation Parameters
  float normalVector[3]; // Normal Vector for Circular Stimulation. In Meters.
  float radius; // Radius for Circular Stimulation. In Meters.

  // General Parameters
  float strength;     // Overall strength Coefficient, Default to 100
  float frequency;

  // Cached Values (Internal Use)
  uint32_t cached_period_us;
  float cached_circ_u[3]; // For Circular and CSF Stimulation
  float cached_circ_v[3]; // For Circular and CSF Stimulation

}Stimulation;

extern int phase_set_mode;

extern int demo_mode;

extern Stimulation CurrentStimulation;

extern Stimulation EmptyStimulation;

extern const Stimulation *DemoStimulations[];

void Switch_Demo_Mode(void);
int Get_Demo_Mode(void);
int Get_Num_Demo_Stimulations(void);

int Get_Phase_Set_Mode(void);
int Get_Stimulation_Enabled(void);
void Stimulation_Enable(void);
void Stimulation_Disable(void);
void Set_Stimulation(const Stimulation *stimulation);
void Update_Stimulation_State(float progress);
