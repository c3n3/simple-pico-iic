# simple-pico-iic
A simple pico iic library

# Slave Usage
```c++
#include "hardware/i2c.h"
#include <simple_i2c/I2cSlave.hpp>
#include <map>

#define TEMPERATURE_REG_ADDR 0x10
#define IIC_SLAVE_ADDRESS 0x45
#define SDA_LINE 0
#define SCL_LINE 1
#define IIC_FREQ 100*1000
typedef float register_temperature_t;

void temperature_reg_write(register_temperature_t writeValue) {
    // Function is automatically called on iic write to slave
}

void temperature_reg_read(register_temperature_t lastValue) {
    // Function is automatically called on iic read
}

// Construct register device
// Reads are automatically handled
static I2cSlave::Reg<register_temperature_t> temperatureRegister(temperature_reg_write, temperature_reg_read);

int main() {
    // blah blah init code....

    // Map register address with register object
    static std::map<uint8_t, I2cSlave::RegBase*> registers = {
        {TEMPERATURE_REG_ADDR, &temperatureRegister},
    };
    // Init slave code
    I2cSlave::init<i2c0> (&registers, 0x45, SDA_LINE, SCL_LINE, IIC_FREQ);
}
```

# Master Usage
```c++
#include "hardware/i2c.h"
#include "simple_i2c/I2cMaster.hpp"

#define TEMPERATURE_REG_ADDR 0x10
#define IIC_SLAVE_ADDRESS 0x45
#define SDA_LINE 0
#define SCL_LINE 1
#define IIC_FREQ 100*1000
typedef float register_temperature_t;

int main() {
    // blah blah init code....

    // Start up the iic line
    I2cMaster::init(i2c0, SDA_LINE, SCL_LINE, IIC_FREQ);

    // Read register and get value
    register_temperature_t result = I2cMaster::regRead<register_temperature_t>(i2c0, IIC_SLAVE_ADDRESS, IIC_SLAVE_ADDRESS);
}
```
