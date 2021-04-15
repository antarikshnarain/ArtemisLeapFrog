/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG 
 * Author     : Antariksh Narain
----------------------------------------------------------------- */

#include "I2C.hpp"

I2C::~I2C()
{
    printf("Closing I2C communication.\n");
    close(this->f_dev);
}

I2C::I2C(int address)
{
    this->device_id = address;
    this->f_dev = open("/dev/i2c-1", O_RDWR);
    if(this->f_dev < 0)
    {
        printf("Failed to open /dev/i2c-1.\n");
        return;
    }
    if(ioctl(this->f_dev, I2C_SLAVE, this->device_id) < 0)
    {
        printf("Could not get I2C bus.\n");
        return;
    }
    printf("Starting I2C communication for 0x%x.\n", this->device_id);
}

void I2C::i2cWrite(int8_t register_address, int8_t register_value)
{
    i2c_smbus_write_byte_data(this->f_dev, register_address, register_value);
}

int8_t I2C::i2cRead(int8_t register_address)
{
    return i2c_smbus_read_byte_data(this->f_dev, register_address);
}