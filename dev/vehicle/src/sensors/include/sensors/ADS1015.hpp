/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Library to communicate with MPU6050 sensor using I2C
 * Reference  : https://github.com/alex-mous/MPU6050-C-CPP-Library-for-Raspberry-Pi
----------------------------------------------------------------- */


#include <iostream>
#include <I2C.hpp>

#ifndef _ADS1015_h_
#define _ADS1015_h_

using namespace std;

#define CONV_REG 0x00
#define CONFIG_REG 0x01

#define READ_A0 0xc183
#define READ_A1 0xd183
#define READ_A2 0xe183
#define READ_A3 0xf183

#define DELAY_READ 1000*1 // 1000 ns

#define NUM_ANALOG 2

#define RATE 1

class ADS1015: public I2C
{
private:
    // Thread manager variables
    std::promise<void> exit_signal;
    std::future<void> exit_future;

protected:

    void readValues();
    // \brief thread function updating the values
    void adsManager(std::future<void> _future);

public:
    int analog_value[NUM_ANALOG] = {0};

    ~ADS1015();
    // \brief Function to Initialize with the selected device address
    // \param device address
    ADS1015(int);
};

#endif