/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG 
 * Author     : Antariksh Narain
----------------------------------------------------------------- */

#include "MPU6050.hpp"

MPU6050::~MPU6050()
{
    printf("Destructor called!\n");
    this->exit_signal.set_value();
}

MPU6050::MPU6050(int device_address, int duration = 2000)
{
    this->id = wiringPiI2CSetup(device_address);
    wiringPiI2CWriteReg8(this->id, PWR_MGMT_1, 0x00);   /* Write to power management register */
    wiringPiI2CWriteReg8(this->id, CONFIG, 0x03);       /* Write to Configuration register */
    wiringPiI2CWriteReg8(this->id, SMPLRT_DIV, 0x04);   /* Write to sample rate register */
    wiringPiI2CWriteReg8(this->id, GYRO_CONFIG, 0x00);  /* Write to Gyro Configuration register */
    wiringPiI2CWriteReg8(this->id, ACCEL_CONFIG, 0x00); /* Write to Gyro Configuration register */
    // Update local variables
    this->Calibrate(duration);
    // Initialize Manager thread
    this->exit_future = this->exit_signal.get_future();
    thread(&MPU6050::imuManager, this, std::move(this->exit_future)).detach();
}

short MPU6050::registerValue(int addr)
{
    short high_byte, low_byte, value;
    high_byte = wiringPiI2CReadReg8(this->id, addr);
    low_byte = wiringPiI2CReadReg8(this->id, addr + 1);
    value = (high_byte << 8) | low_byte;
    printf("Value: %d, ", value);
    return value;
}
void MPU6050::readRawValues()
{
    // Get Acceleration Values
    this->accel_linear[0] = this->registerValue(ACCEL_XOUT_H);
    this->accel_linear[1] = this->registerValue(ACCEL_YOUT_H);
    this->accel_linear[2] = this->registerValue(ACCEL_ZOUT_H);
    // Get Gyro Values
    this->accel_angular[0] = this->registerValue(GYRO_XOUT_H);
    this->accel_angular[1] = this->registerValue(GYRO_YOUT_H);
    this->accel_angular[2] = this->registerValue(GYRO_ZOUT_H);
    // Get Temperature values
    this->TempC = this->registerValue(TEMP_OUT_H);
}
void MPU6050::readValues()
{
    this->readRawValues();
    // Get Acceleration Values
    this->accel_linear[0] = round((this->accel_linear[0] - this->offsetA[0]) * 1000.0 / ACCEL_SENS) * 1000.0;
    this->accel_linear[1] = round((this->accel_linear[1] - this->offsetA[1]) * 1000.0 / ACCEL_SENS) * 1000.0;
    this->accel_linear[2] = round((this->accel_linear[2] - this->offsetA[2]) * 1000.0 / ACCEL_SENS) * 1000.0;
    // Get Gyro Values
    this->accel_angular[0] = round((this->accel_angular[0] - this->offsetG[0]) * 1000.0 / GYRO_SENS) * 1000.0;
    this->accel_angular[1] = round((this->accel_angular[1] - this->offsetG[1]) * 1000.0 / GYRO_SENS) * 1000.0;
    this->accel_angular[2] = round((this->accel_angular[2] - this->offsetG[2]) * 1000.0 / GYRO_SENS) * 1000.0;
}

void MPU6050::imuManager(std::future<void> _future)
{
    printf("Starting IMU manager!\n");
    int ctr = 0;
    while (_future.wait_for(chrono::microseconds(this->dt)) == std::future_status::timeout)
    {
        this->readValues();
        //X (roll) axis
        this->_accel_angle[0] = atan2(this->accel_linear[2], this->accel_linear[1]) * RAD_TO_DEG - 90.0; //Calculate the angle with z and y convert to degrees and subtract 90 degrees to rotate
        this->_gyro_angle[0] = this->rpy[0] + this->accel_angular[0] * this->dt;                         //Use roll axis (X axis)

        //Y (pitch) axis
        _accel_angle[1] = atan2(this->accel_linear[2], this->accel_linear[0]) * RAD_TO_DEG - 90.0; //Calculate the angle with z and x convert to degrees and subtract 90 dethis->accel_angular[0]ees to rotate
        _gyro_angle[1] = this->rpy[1] + this->accel_angular[1] * this->dt;                         //Use pitch axis (Y axis)

        //Z (yaw) axis
        if (calc_yaw)
        {
            _gyro_angle[2] = this->rpy[2] + this->accel_angular[2] * this->dt; //Use yaw axis (Z axis)
        }

        if (this->_first_run)
        { //Set the gyroscope angle reference point if this is the first function run
            for (int i = 0; i <= 1; i++)
            {
                _gyro_angle[i] = _accel_angle[i]; //Start off with angle from accelerometer (absolute angle since gyroscope is relative)
            }
            _gyro_angle[2] = 0; //Set the yaw axis to zero (because the angle cannot be calculated with the accelerometer when vertical)
            this->_first_run = false;
        }

        float asum = abs(this->accel_linear[0]) + abs(this->accel_linear[1]) + abs(this->accel_linear[2]);    //Calculate the sum of the accelerations
        float gsum = abs(this->accel_angular[0]) + abs(this->accel_angular[1]) + abs(this->accel_angular[2]); //Calculate the sum of the gyro readings

        for (int i = 0; i <= 1; i++)
        { //Loop through roll and pitch axes
            if (abs(_gyro_angle[i] - _accel_angle[i]) > 5)
            { //Correct for very large drift (or incorrect measurment of gyroscope by longer loop time)
                _gyro_angle[i] = _accel_angle[i];
            }

            //Create result from either complementary filter or directly from gyroscope or accelerometer depending on conditions
            if (asum > 0.1 && asum < 3 && gsum > 0.3)
            {                                                                            //Check that th movement is not very high (therefore providing inacurate angles)
                this->rpy[i] = (1 - TAU) * (_gyro_angle[i]) + (TAU) * (_accel_angle[i]); //Calculate the angle using a complementary filter
            }
            else if (gsum > 0.3)
            { //Use the gyroscope angle if the acceleration is high
                this->rpy[i] = _gyro_angle[i];
            }
            else if (gsum <= 0.3)
            { //Use accelerometer angle if not much movement
                this->rpy[i] = _accel_angle[i];
            }
        }

        //The yaw axis will not work with the accelerometer angle, so only use gyroscope angle
        if (calc_yaw)
        { //Only calculate the angle when we want it to prevent large drift
            this->rpy[2] = _gyro_angle[2];
        }
        else
        {
            this->rpy[2] = 0;
            _gyro_angle[2] = 0;
        }
        
        ctr++;
        if(ctr == 10)
        {
            printf("Updated values!\n");
            ctr = 0;
        }
    }
    printf("Closing IMU Manager.\n");
}

void MPU6050::Calibrate(int count = 10000)
{
    printf("Calibrating...\n");
    // Read sensor values for <count>
    for (int i = 0; i < count; i++)
    {
        this->readRawValues();
        this->offsetA[0] += this->accel_linear[0];
        this->offsetA[1] += this->accel_linear[1];
        this->offsetA[2] += this->accel_linear[2];
        this->offsetG[0] += this->accel_angular[0];
        this->offsetG[1] += this->accel_angular[1];
        this->offsetG[2] += this->accel_angular[2];
    }
    // TODO: Calulate the mean, variance, max and min values
    for (int i = 0; i < 3; i++)
    {
        this->offsetA[i] /= count;
        this->offsetG[i] /= count;
    }
    // IMP: Remove 1g from the value calculated to compensate for gravity
    this->offsetA[2] -= ACCEL_SENS;
    printf("Offsets: ");
    for(int i=0;i<3;i++)
    {
        printf("%.2f | %.2f, ",this->offsetA[i], this->offsetG[i]);
    }
    printf("Done with Calibration!\n");
}

vector<float> MPU6050::RawValues()
{
    return vector<float>{
        this->accel_linear[0], this->accel_linear[1], this->accel_linear[2],
        this->accel_angular[0], this->accel_angular[1], this->accel_angular[2],
        this->TempC};
}

vector<float> MPU6050::PrincipalAxisValues()
{
    return vector<float>{this->rpy[0], this->rpy[1], this->rpy[2]};
}

#ifdef TEST_MPU6050
#include <bits/stdc++.h>
#include <unistd.h>
int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        printf("Pass port as parameters\n");
        return -1;
    }
    MPU6050 mpu(atoi(argv[1]), 1000);
    printf("Ax,Ay,Az,Gx,Gy,Gz,Temp,R,P,Y\n");
    int values = 100;
    while (values--)
    {
        for (const float v : mpu.RawValues())
        {
            printf("%.2f, ", v);
        }
        for (const float v : mpu.PrincipalAxisValues())
        {
            printf("%.2f, ", v);
        }
        printf("\n");
        usleep(100000);
    }
    return 0;
}
#endif
