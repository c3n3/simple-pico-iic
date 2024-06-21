#include "I2cSlave.hpp"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "pico/stdlib.h"

static std::map<uint8_t, I2cSlave::RegBase*>* map_;

template<typename T>
I2cSlave::Reg<T>::Reg(I2cSlave::Reg<T>::callback_t wcback, I2cSlave::Reg<T>::callback_t rcback)
    : wcback_(wcback), rcback_(rcback)
{}

template<typename T>
void I2cSlave::Reg<T>::handleWrite(uint8_t* buf, size_t length)
{
    if (length >= sizeof(T)) {
        value_ = *((T*)buf);
        if (wcback_ != nullptr) {
            wcback_(value_);
        }
    }
}

template<typename T>
void I2cSlave::Reg<T>::handleRead()
{
    if (rcback_ != nullptr) {
        rcback_(value_);
    }
}

template<typename T>
T I2cSlave::Reg<T>::getValue()
{
    return value_;
}

template<typename T>
void I2cSlave::Reg<T>::setValue(T value)
{
    value_ = value;
}

template<typename T>
uint8_t* I2cSlave::Reg<T>::read()
{
    return reinterpret_cast<uint8_t*>(&value_);
}

template<typename T>
size_t I2cSlave::Reg<T>::size()
{
    return sizeof(T);
}

template<i2c_inst_t* INST>
void __i2c_irq_handler() {
    static uint8_t buffer_[256];
    static uint8_t loc = 0;
    static uint8_t addr = 0x00;
    static bool writing = false;
    static std::map<uint8_t, I2cSlave::RegBase*>& map = *map_;

    // Get interrupt status
    uint32_t status = INST->hw->intr_stat;
    // Check to see if we have received data from the I2C controller
    if (status & I2C_IC_INTR_STAT_R_RX_FULL_BITS) {
        // Read the data (this will clear the interrupt)
        uint32_t value = INST->hw->data_cmd;
        // Check if this is the 1st byte we have received
        if (value & I2C_IC_DATA_CMD_FIRST_DATA_BYTE_BITS) {
            // If so treat it as the address to use
            addr = (uint8_t)(value & I2C_IC_DATA_CMD_DAT_BITS);
            loc = 0;
        } else {
            // If not 1st byte then store the data in the RAM
            // and increment the address to point to next byte
            if (map.count(addr)) {
                buffer_[loc] = (uint8_t)(value & I2C_IC_DATA_CMD_DAT_BITS);
                loc++;
                if (map[addr]->size() == loc) {
                    map[addr]->handleWrite(buffer_, loc);
                    loc = 0;
                }
            }
        }
    }

    // Check to see if the I2C controller is requesting data from the RAM
    if (status & I2C_IC_INTR_STAT_R_RD_REQ_BITS) {
        if (loc >= map[addr]->size()) {
            loc = 0;
        }

        // Write the data from the current address in RAM
        INST->hw->data_cmd = (uint32_t)map[addr]->read()[loc];

        // Clear the interrupt
        INST->hw->clr_rd_req;
        // Increment the address
        loc++;
        if (loc == map[addr]->size()) {
            map[addr]->handleRead();
        }
    }
}

template<i2c_inst_t* INST>
void I2cSlave::init(std::map<uint8_t, RegBase*>* map, uint8_t addr, uint8_t sda, uint8_t scl, uint32_t freq)
{
    const auto irq = INST == i2c0 ? I2C0_IRQ : I2C1_IRQ;
    map_ = map;
    i2c_init(INST, freq);
    i2c_set_slave_mode(INST, true, addr);

    // Setup GPIO pins to use and add pull up resistors
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);

    // Enable the I2C interrupts we want to process
    INST->hw->intr_mask = (I2C_IC_INTR_MASK_M_RD_REQ_BITS | I2C_IC_INTR_MASK_M_RX_FULL_BITS);

    // Set up the interrupt handler to service I2C interrupts
    irq_set_exclusive_handler(irq, __i2c_irq_handler<INST>);

    // Enable I2C interrupt
    irq_set_enabled(irq, true);
}
