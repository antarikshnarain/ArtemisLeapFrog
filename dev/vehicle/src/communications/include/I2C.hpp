/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Library to communicate with devices using I2C communication.
 * Reference  : https://github.com/alex-mous/MPU6050-C-CPP-Library-for-Raspberry-Pi
 * Compilation: -li2c
----------------------------------------------------------------- */

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <time.h>
extern "C" {
	#include <linux/i2c-dev.h>
	#include <i2c/smbus.h>
}
#include <cmath>
#include <thread>
#include <vector>
#include <future>
#include <chrono>
#include <string>

#ifndef _I2C_h_
#define _I2C_h_

class I2C
{
private:
    int device_id;
    int f_dev;

public:
    ~I2C();
    // \brief initializes I2C communication.
    // \param i2c port
    // \param device address
    I2C(int,int);

    // \brief Write byte data
    // \param register address
    // \param value
    void i2cWrite(int8_t, uint8_t);

    // \brief Write word data
    // \param register address
    // \param word value
    void i2cWriteWord(int8_t, uint16_t);

    // \brief Send byte
    // \param register address
    void i2cSend(int8_t);

    // \brief Read register value
    // \param register address
    // \return register value
    int8_t i2cRead(int8_t);

    // \brief Read word value
    // \param register address
    // \return word register value
    uint16_t i2cReadWord(int8_t);
};

#endif