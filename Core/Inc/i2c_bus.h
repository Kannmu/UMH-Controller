#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stdint.h>

void i2c_bus_init(void);
int i2c_bus_lock(uint32_t timeout_ms);
void i2c_bus_unlock(void);

/* Bit-bang 9 SCL clocks + STOP on PA15/PB7 and re-init I2C1.  This releases
 * a slave that is holding SDA/SCL low after a timed-out transfer, which would
 * otherwise leave the I2C peripheral permanently BUSY and freeze the OLED.
 * i2c_bus_recover() takes the bus mutex itself; the _locked variant is for
 * callers that already hold it (EEPROM read/write retry paths). */
int i2c_bus_recover(void);
void i2c_bus_recover_locked(void);
/* Check the SCL/SDA pin levels and recover immediately when a slave (or the
 * peripheral) is holding the bus low.  The locked variant is for callers that
 * already hold the bus mutex. */
int i2c_bus_recover_if_stuck(void);
void i2c_bus_recover_if_stuck_locked(void);
int i2c_bus_is_dead(void);

#endif
