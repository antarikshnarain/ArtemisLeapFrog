/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Library to communicate with MPU6050 sensor using I2C
 * Reference  : https://github.com/alex-mous/MPU6050-C-CPP-Library-for-Raspberry-Pi
----------------------------------------------------------------- */


#include <iostream>
#include "I2C.hpp"

#ifndef _MPU6050_h_
#define _MPU6050_h_

using namespace std;

#define PWR_MGMT_1 0x6B
#define CONFIG 0x1A
#define SMPLRT_DIV 0x19
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define FIFO_EN 0x23
#define INT_ENABLE 0x38

#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define TEMP_OUT_H 0x41
#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_H 0x47

#define GYRO_SENS 131.0
#define ACCEL_SENS 16384.0

#define RAD_TO_DEG 57.29577951308
#define TAU 0.05 //Complementary filter percentage

#define RATE 1 // rate in ms

class MPU6050: public I2C
{
private:
    float accel_linear[3];
    float accel_angular[3];
    float rpy[3];
    float TempC;
    float offsetA[3] = {0.0};
    float offsetG[3] = {0.0};

    float _accel_angle[3];
    float _gyro_angle[3];
    bool _first_run = true;

    // Thread manager variables
    std::promise<void> exit_signal;
    std::future<void> exit_future;

protected:
    // \brief Function to read raw register value
    int16_t registerValue(int8_t);

    // \brief read values from the registers
    void readRawValues();

    // \brief read values with offset
    void readValues();

    // \brief compute principal axis values
    void principalAxisValues();

    // \brief thread function updating the values
    void imuManager(std::future<void> _future);

public:
    bool calc_yaw = false;

    ~MPU6050();
    // \brief Function to Initialize with the selected device address
    // \param device address
    // \param Optional : calibration cycles
    MPU6050(int, int);

    // \brief Function to return RAW values from the sensor.
    // \return Vector of values for linear and angular acceleration with temperature.
    vector<float> RawValues();

    // \brief Function to calculate Pitch and Roll
    // \return Vector of Roll, Pitch and Yaw
    vector<float> PrincipalAxisValues();

    // \brief Calculate sensor offset for linear and angular values.
    // \param milliseconds to calibrate
    void Calibrate(int duration);
};

#endif