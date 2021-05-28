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

I2C::I2C(int port, int address)
{
    this->device_id = address;
    std::string i2c_port = "/dev/i2c-" + std::to_string(port);
    this->f_dev = open(i2c_port.c_str(), O_RDWR);
    if(this->f_dev < 0)
    {
        printf("Failed to open %s.\n", i2c_port.c_str());
        return;
    }
    if(ioctl(this->f_dev, I2C_SLAVE, this->device_id) < 0)
    {
        printf("Could not get I2C bus.\n");
        return;
    }
    printf("Starting I2C communication for 0x%x.\n", this->device_id);
}

void I2C::i2cWrite(int8_t register_address, uint8_t register_value)
{
    i2c_smbus_write_byte_data(this->f_dev, register_address, register_value);
}

void I2C::i2cWriteWord(int8_t register_address, uint16_t register_value)
{
    i2c_smbus_write_word_data(this->f_dev, register_address, register_value);
}

void I2C::i2cSend(int8_t register_address)
{
    i2c_smbus_write_byte(this->f_dev, register_address);
}


int8_t I2C::i2cRead(int8_t register_address)
{
    return i2c_smbus_read_byte_data(this->f_dev, register_address);
}

uint16_t I2C::i2cReadWord(int8_t register_address)
{
    return i2c_smbus_read_word_data(this->f_dev, register_address);
}