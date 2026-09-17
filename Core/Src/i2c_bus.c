#include "i2c_bus.h"
#include "i2c.h"
#include "main.h"
#include "cmsis_os.h"

static osMutexId_t bus_mutex;
static StaticSemaphore_t bus_mutex_memory;
static const osMutexAttr_t bus_mutex_attributes = {
  .name = "i2c1-bus",
  .attr_bits = 0u,
  .cb_mem = &bus_mutex_memory,
  .cb_size = sizeof(bus_mutex_memory)
};

void i2c_bus_init(void)
{
  if (bus_mutex == NULL && osKernelGetState() != osKernelInactive)
    bus_mutex = osMutexNew(&bus_mutex_attributes);
}

int i2c_bus_lock(uint32_t timeout_ms)
{
  if (bus_mutex == NULL) i2c_bus_init();
  if (bus_mutex == NULL) return -1;
  return osMutexAcquire(bus_mutex, timeout_ms == osWaitForever ? osWaitForever :
                        pdMS_TO_TICKS(timeout_ms)) == osOK ? 0 : -1;
}

void i2c_bus_unlock(void)
{
  if (bus_mutex != NULL) (void)osMutexRelease(bus_mutex);
}

volatile uint32_t i2c_bus_recover_attempts;
volatile uint32_t i2c_bus_recover_failures;
static volatile uint8_t i2c_bus_dead;
static uint32_t i2c_bus_last_recover_ms;

static void i2c_bus_short_delay(void)
{
  volatile uint32_t n;
  for (n = 0u; n < 240u; ++n) __NOP();
}

/* Bit-bang up to 64 SCL cycles while watching SDA.  Many I2C slaves release
 * a stuck SDA only after their current byte/ACK phase has been clocked out;
 * nine clocks are not always enough.  Finish with a STOP and only re-init the
 * peripheral when both lines are actually released. */
static int i2c_bus_recover_impl(void)
{
  GPIO_InitTypeDef gpio = {0};
  uint8_t i;
  int released = 0;
  (void)HAL_I2C_DeInit(&hi2c1);
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Pin = GPIO_PIN_15;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOB, &gpio);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  i2c_bus_short_delay();
  for (i = 0u; i < 64u; ++i) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    i2c_bus_short_delay();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    i2c_bus_short_delay();
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) != GPIO_PIN_RESET) {
      released = 1;
      break;
    }
  }
  /* STOP: SDA low while SCL high, then release SDA. */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  i2c_bus_short_delay();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  i2c_bus_short_delay();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  i2c_bus_short_delay();
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) != GPIO_PIN_RESET) released = 1;
  {
    int scl_high = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) != GPIO_PIN_RESET);
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);
    MX_I2C1_Init();
    if (released != 0 && scl_high != 0) {
      i2c_bus_dead = 0u;
      return 0;
    }
    i2c_bus_dead = 1u;
    return -1;
  }
}

int i2c_bus_is_dead(void)
{
  return i2c_bus_dead != 0u;
}

void i2c_bus_recover_locked(void)
{
  if (i2c_bus_dead != 0u &&
      (HAL_GetTick() - i2c_bus_last_recover_ms) < 1000u) return;
  i2c_bus_last_recover_ms = HAL_GetTick();
  ++i2c_bus_recover_attempts;
  if (i2c_bus_recover_impl() != 0) ++i2c_bus_recover_failures;
}

int i2c_bus_recover(void)
{
  int locked = 0;
  int result;
  if (i2c_bus_dead != 0u &&
      (HAL_GetTick() - i2c_bus_last_recover_ms) < 1000u) return -1;
  if (bus_mutex == NULL && osKernelGetState() != osKernelInactive) i2c_bus_init();
  if (bus_mutex != NULL) {
    /* Never de-init/re-init I2C behind the back of a task that already owns
     * the bus: doing so while HAL is mid-transfer leaves BUSY_TX/state
     * inconsistencies that make all later EEPROM/OLED transactions fail.
     * A timeout simply reports busy and lets the caller retry. */
    if (i2c_bus_lock(100u) != 0) return -1;
    locked = 1;
  }
  i2c_bus_last_recover_ms = HAL_GetTick();
  ++i2c_bus_recover_attempts;
  result = i2c_bus_recover_impl();
  if (locked != 0) i2c_bus_unlock();
  if (result != 0) ++i2c_bus_recover_failures;
  return result;
}

static int i2c_bus_lines_low(void)
{
  return (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET) ||
         (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET);
}

/* A stuck bus is not always visible on the pins: an interrupted HAL
 * transaction can leave State != READY and/or the peripheral BUSY flag set
 * while both lines are already high.  Treat that as stuck too, otherwise
 * every later OLED/EEPROM transfer returns HAL_BUSY forever. */
static int i2c_bus_is_stuck(void)
{
  if (i2c_bus_lines_low() != 0) return 1;
  if (hi2c1.State != HAL_I2C_STATE_READY) return 1;
  if ((hi2c1.Instance->ISR & I2C_ISR_BUSY) != 0u) return 1;
  return 0;
}

void i2c_bus_recover_if_stuck_locked(void)
{
  if (i2c_bus_dead != 0u) return;
  if (i2c_bus_is_stuck() != 0) i2c_bus_recover_locked();
}

int i2c_bus_recover_if_stuck(void)
{
  if (i2c_bus_dead != 0u) return -1;
  if (i2c_bus_is_stuck() == 0) return 0;
  return i2c_bus_recover();
}