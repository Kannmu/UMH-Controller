# pragma once
# include "main.h"
# include "stimulation.h"

# define TRANSDUCER_SIZE (10.0e-3)
# define TRANSDUCER_SPACING (10.0e-3)

/* V5.5 阵列: 4 圈同心圆环 (6/12/18/24 = 60 真实阵元) */
# define NUM_REAL_TRANSDUCER 60U

/* 特殊通道: 与真实阵元共用 DMA 缓冲通道, 保证时间轴同步 */
# define VIRTUAL_INDEX      60U   /* PC13 — 40kHz 相位参考 (连续方波, 相位0) */
# define TRIGGER0_INDEX     61U   /* PC14 — 200Hz 周期开头 1ms 脉冲 */
# define TRIGGER1_INDEX     62U   /* PC15 — 可配置 (默认全0) */
# define NUM_TOTAL_CHANNELS 63U   /* 60 真实 + 虚拟 + 2 触发 */

# define SPEED_OF_SOUND 343.2

# define TRANSDUCER_BASE_FREQ 40000UL

# define TRANSDUCER_PERIOD  ((long double)(1.0 / TRANSDUCER_BASE_FREQ))

# define WAVE_LENGTH (TRANSDUCER_PERIOD*SPEED_OF_SOUND)

# define TRANSDUCER_PERIOD_US (1e6f / (float)TRANSDUCER_BASE_FREQ)

# define US_PER_SEC   1000000U
# define US_PER_SEC_F 1e6f

/* V5.5 同心圆环几何参数 */
# define NUM_RINGS         4U
# define GOLDEN_ANGLE_DEG  137.5f
# define GOLDEN_ANGLE_RAD  (GOLDEN_ANGLE_DEG * (float)M_PI / 180.0f)

typedef struct Stimulation Stimulation;

// Transducer Class
typedef struct Transducer
{
    uint8_t index;
    uint8_t ring;          /* 所在圆环 (0..3); 特殊通道为 0 */
    uint8_t ring_index;    /* 环内序号 */
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
extern Transducer TransducerArray[NUM_TOTAL_CHANNELS];


extern float Wave_K;

void Transducer_Init(void);
void Enter_Calibration_Mode(void);
void Load_Calib_to_Transducers(void);

void Set_Point_Focus(float *position);
void Set_Twin_Trap_Focus(float *position);
void Set_Transducers(uint8_t *data);
void Set_Plane_Wave(void);

float Distance_to_Phase(float distance);
float Phase_to_Gap_Ticks(float phase);

GPIO_TypeDef *map_pin_name_to_gpio_port(const char *);
uint8_t map_pin_name_to_gpio_port_num(const char *);
uint16_t map_pin_name_to_pin_number(const char *);
