/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG 
 * Author     : Antariksh Narain
 * Description: 
 * 
----------------------------------------------------------------- */

#include "MPU6050.h"
#include <vector>
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>

using namespace std;

#pragma region PUBLIC MEMBERS

MPU6050::MPU6050(int device_address)
{
    this->id = wiringPiI2CSetup(Device_Address);
    wiringPiI2CWriteReg8(this->id, SMPLRT_DIV, 0x07); /* Write to sample rate register */
    wiringPiI2CWriteReg8(this->id, PWR_MGMT_1, 0x01); /* Write to power management register */
    wiringPiI2CWriteReg8(this->id, CONFIG, 0);        /* Write to Configuration register */
    wiringPiI2CWriteReg8(this->id, GYRO_CONFIG, 24);  /* Write to Gyro Configuration register */
    wiringPiI2CWriteReg8(this->id, INT_ENABLE, 0x01); /*Write to interrupt enable register */   
}

vector<float> MPU6050::RawValues()
{
    return vector<float>{this->Ax, this->Ay, this->Az, this->Gx, this->Gy, this->Gz, this->TempC}; 
}

vector<double> MPU6050::PrincipalAxisValues()
{
    this->pitch = Ax / (Ax^2 + Az^2);
    this->roll = Ay / (Ay^2 + Az^2);
}

#pragma endregion

#pragma region PRIVATE MEMBERS
short MPU6050::registerValue(int addr)
{
    short high_byte, low_byte, value;
    high_byte = wiringPiI2CReadReg8(this->id, addr);
    low_byte = wiringPiI2CReadReg8(this->id, addr + 1);
    value = (high_byte << 8) | low_byte;
    return value;
}
void MPU6050::Calibrate(int duration)
{
    // TODO: Read sensor values for the given duration (seconds)
    time_t s_time = time();
    do
    {
        
    }while(time() - s_time < duration);
    // TODO: Calulate the mean, variance, max and min values
}
#pragma endregion