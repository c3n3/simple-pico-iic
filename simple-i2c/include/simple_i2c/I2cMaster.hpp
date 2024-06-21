#include "hardware/i2c.h"
#include <stdio.h>
#include <stdint.h>

namespace I2cMaster {
    void init(i2c_inst_t* i2c, uint8_t sda, uint8_t scl, uint32_t freq);

    /**
     * @brief Write to a 'register' over i2c
     *
     * @param i2c I2c device to use
     * @param addr I2c address
     * @param reg Register
     * @param buf Buffer of bytes to transfer
     * @param nbytes Number of bytes to transfer
     */
    void regWriteRaw(i2c_inst_t *i2c, 
                    const uint addr, 
                    const uint8_t reg, 
                    uint8_t *buf,
                    const uint8_t nbytes);

    /**
     * @brief Read from a register
     *
     * @param i2c I2c device to use
     * @param addr I2c address
     * @param reg Register to read from
     * @param buf Buffer of bytes to read into
     * @param nbytes Number of bytes to read
     * @return int Number of read bytes
     */
    int regReadRaw(i2c_inst_t *i2c,
                    const uint addr,
                    const uint8_t reg,
                    uint8_t *buf,
                    const uint8_t nbytes);

    /**
     * @brief Read a specific type from a register
     * 
     * @tparam T Register datatype
     * @param i2c I2c device to use
     * @param addr I2c address
     * @param reg Register to read
     * @return T Data
     */
    template<typename T>
    T regRead(i2c_inst_t *i2c, const uint addr, const uint8_t reg)
    {
        T ret;
        regReadRaw(i2c, addr, reg, reinterpret_cast<uint8_t*>(&ret), sizeof(T));
        return ret;
    }

    /**
     * @brief Write a data type to a register
     * 
     * @tparam T 
     * @param i2c I2c device to use
     * @param addr I2c address
     * @param reg Register to write
     * @param value Value to write to register
     */
    template<typename T>
    void regWrite(i2c_inst_t *i2c, const uint8_t addr, const uint8_t reg, T value)
    {
        regWriteRaw(i2c, addr, reg, reinterpret_cast<uint8_t*>(&value), sizeof(T));
    }
};
