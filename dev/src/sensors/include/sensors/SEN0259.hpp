/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG 
 * Author     : Antariksh Narain
 * Description: 
 * 
----------------------------------------------------------------- */

#define BAUD 115200

#include <stdint.h>
#include <cstdio>
#include "sensors/Serial.hpp"

#define MSG_SIZE 9 // bytes

class SEN0259: public Serial
{
private:
    int distance = 0;
    int strength = 0;
    char * sensor_data;
    // \brief read data from serial port
    // \return false if checksum failed or invalid format
    bool readData();
public:
    SEN0259(string port, int baudrate);
    int GetDistance(void);
    int GetStrength(void);
};