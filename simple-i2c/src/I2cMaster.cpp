#include <simple_i2c/I2cMaster.hpp>
#include "pico/stdlib.h"
#include "pico/binary_info.h"

#include <alloca.h>

void I2cMaster::init(i2c_inst_t* i2c, uint8_t sda, uint8_t scl, uint32_t freq)
{
    i2c_init(i2c, freq);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);
    bi_decl(bi_2pins_with_func(sda, scl, GPIO_FUNC_I2C));
}

// Write 1 byte to the specified register
void I2cMaster::regWriteRaw(i2c_inst_t *i2c, 
                const uint addr, 
                const uint8_t reg, 
                uint8_t *buf,
                const uint8_t nbytes)
{
    uint8_t* msg = (uint8_t*)alloca(nbytes + 1);

    // Append register address to front of data packet
    msg[0] = reg;
    for (int i = 0; i < nbytes; i++) {
        msg[i + 1] = buf[i];
    }

    i2c_write_blocking(i2c, addr, msg, (nbytes + 1), false);
}

int I2cMaster::regReadRaw(  i2c_inst_t *i2c,
                const uint addr,
                const uint8_t reg,
                uint8_t *buf,
                const uint8_t nbytes)
{
    if (nbytes == 0) {
        return 0;
    }

    int num_bytes_read = 0;

    // Write to start register transfer
    i2c_write_blocking(i2c, addr, &reg, 1, true);
    num_bytes_read = i2c_read_blocking(i2c, addr, buf, nbytes, false);

    return num_bytes_read;
}
