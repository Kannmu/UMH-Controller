#include "i2c_bus.h"
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
