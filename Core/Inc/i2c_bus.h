#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stdint.h>

void i2c_bus_init(void);
int i2c_bus_lock(uint32_t timeout_ms);
void i2c_bus_unlock(void);

#endif
