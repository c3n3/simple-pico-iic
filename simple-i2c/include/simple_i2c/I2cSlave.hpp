// Modified from an example provided by:
// Graham Smith

#ifndef __I2C_SLAVE_HPP__
#define __I2C_SLAVE_HPP__

#include <map>

#include "hardware/i2c.h"

namespace I2cSlave {
    /**
     * @brief I2c register base class
     */
    class RegBase {
    public:
        /**
         * @brief Handle a stream of bytes
         * 
         * @param buf Buffer of bytes
         * @param length Length of bytes
         */
        virtual void handleWrite(uint8_t* buf, size_t length) = 0;

        /**
         * @brief Handle a stream of bytes
         * 
         * @param buf Buffer of bytes
         * @param length Length of bytes
         */
        virtual void handleRead() = 0;

        /**
         * @brief Read the stored value in the register
         * 
         * @return uint8_t* 
         */
        virtual uint8_t* read() = 0;

        /**
         * @brief Get the size in bytes of the register
         *
         * @return size_t
         */
        virtual size_t size() = 0;
    };

    /**
     * @brief Typed i2c register
     *
     * @tparam T The data type stored in the register
     */
    template<typename T>
    class Reg : public RegBase {
    public:
        /**
         * @brief Callback function type. Used when a full
         * set of bytes has been input into the register
         */
        typedef void (*callback_t)(T);
    private:
        callback_t wcback_;
        callback_t rcback_;
        T value_;
    public:
        /**
         * @brief Get the latest value of the register
         *
         * @return T
         */
        T getValue();

        /**
         * @brief Set the value of the register
         */
        void setValue(T value);

        /**
         * @brief Create a new register
         * 
         * @param wcback Write callback function
         * @param rcback Read callback function
         */
        Reg(callback_t wcback=nullptr, callback_t rcback=nullptr);

        /**
         * @brief Handle a stream of bytes
         * 
         * @param buf Bytes input into register
         * @param length Length of bytes
         */
        void handleWrite(uint8_t* buf, size_t length);

        /**
         * @brief Handle a stream of bytes
         * 
         * @param buf Bytes input into register
         * @param length Length of bytes
         */
        void handleRead();

        /**
         * @brief Get the byte buffer from the register
         *
         * @return uint8_t* Bytes
         */
        uint8_t* read();

        /**
         * @brief Gets the size of the read() buffer
         *
         * @return size_t
         */
        size_t size();
    };

    /**
     * @brief Initialize an i2c slave instance
     *
     * @tparam INST The i2c device to use
     * @param map A map of addresses to registers.
     * @param addr I2c device address
     * @param sda Sda pin
     * @param scl Scl pin
     * @param freq Frequency to run the i2c bus at
     */
    template<i2c_inst_t* INST>
    void init(std::map<uint8_t, RegBase*>* map, uint8_t addr, uint8_t sda, uint8_t scl, uint32_t freq = 100*1000);
}

#include "I2cSlaveImpl.hpp"

#endif
