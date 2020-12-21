/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG 
 * Author     : Antariksh Narain
 * Description: 
 * 
----------------------------------------------------------------- */

#include "MPU6050.hpp"

#define RAD_TO_DEG 57.2958

#pragma region PUBLIC MEMBERS

MPU6050::MPU6050(int device_address)
{
    this->id = wiringPiI2CSetup(Device_Address);
    wiringPiI2CWriteReg8(this->id, SMPLRT_DIV, 0x07); /* Write to sample rate register */
    wiringPiI2CWriteReg8(this->id, PWR_MGMT_1, 0x01); /* Write to power management register */
    wiringPiI2CWriteReg8(this->id, CONFIG, 0);        /* Write to Configuration register */
    wiringPiI2CWriteReg8(this->id, GYRO_CONFIG, 24);  /* Write to Gyro Configuration register */
    wiringPiI2CWriteReg8(this->id, INT_ENABLE, 0x01); /*Write to interrupt enable register */

    // Update local variables
}

vector<float> MPU6050::RawValues()
{
    return vector<float>{this->Ax, this->Ay, this->Az, this->Gx, this->Gy, this->Gz, this->TempC};
}

vector<double> MPU6050::PrincipalAxisValues()
{
    this->pitch = Ax / (Ax ^ 2 + Az ^ 2);
    this->roll = Ay / (Ay ^ 2 + Az ^ 2);
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
void MPU6050::readValues()
{
    this->Ax = this->registerValue(ACCEL_XOUT_H) this->Ay = this->registerValue(ACCEL_YOUT_H);
    this->Az = this->registerValue(ACCEL_ZOUT_H);
    this->Gx = this->registerValue(GYRO_XOUT_H);
    this->Gy = this->registerValue(GYRO_YOUT_H);
    this->Gz = this->registerValue(GYRO_ZOUT_H);
    this->TempC = this->registerValue(TEMP_OUT_H);

    double roll = atan(this->Ay / sqrt(this->Ax * this->Ax + this->Az * this->Az)) * RAD_TO_DEG;
    double pitch = atan2(-this->Ax, this->Az) * RAD_TO_DEG;
    this->filters[0].setAngle(roll);
    this->filters[1].setAngle(pitch);
}
void MPU6050::Calibrate(int duration)
{
    // TODO: Read sensor values for the given duration (seconds)
    time_t s_time = time();
    int ctr = 0;
    do
    {
        this->offsetA[0] += this->Ax;
        this->offsetA[1] += this->Ay;
        this->offsetA[2] += this->Az;
        this->offsetG[0] += this->Gx;
        this->offsetG[1] += this->Gy;
        this->offsetG[2] += this->Gz;
    } while (time() - s_time < duration);
    // TODO: Calulate the mean, variance, max and min values
    for (int i = 0; i < 3; i++)
    {
        this->offsetA[i] /= ctr;
        this->offsetG[i] /= ctr;
    }
}
vector<double> MPU6050::PrincipalAxisValues()
{
    return vector<double>{
        this->filters[0].getAngle(this->Gx, this->Gx / 131.0, this->dt),
        this->filters[1].getAngle(this->Gy, this->Gy / 131.0, this->dt)};
}
#pragma endregion