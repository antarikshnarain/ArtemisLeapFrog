/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG 
 * Author     : Antariksh Narain
----------------------------------------------------------------- */

#include "sensors/MPU6050.hpp"

MPU6050::~MPU6050()
{
    printf("Closing MPU6050.\n");
    this->exit_signal.set_value();
    sleep(1);
}

MPU6050::MPU6050(int device_address, int cycles = 1000) : I2C(device_address)
{
    this->i2cWrite(SMPLRT_DIV, 4); // 7, 4
    this->i2cWrite(PWR_MGMT_1, 0); // 1, 0
    this->i2cWrite(CONFIG, 3); // 0, 3
    this->i2cWrite(GYRO_CONFIG, 0); // 24, 0
    this->i2cWrite(ACCEL_CONFIG, 0); // 0, 0

    // Update local variables
    this->Calibrate(cycles);
    // Initialize Manager thread
    this->exit_future = this->exit_signal.get_future();
    thread(&MPU6050::imuManager, this, std::move(this->exit_future)).detach();
}

int16_t MPU6050::registerValue(int8_t addr)
{
    int16_t t = (this->i2cRead(addr) << 8) | this->i2cRead(addr + 1);
    // if(t > 32768)
    // {
    //     t -= 65536;
    // }
    return t;
}
void MPU6050::readRawValues()
{
    int8_t start_addr = 0x3B;
    for(int i = 0; i < 3;i++)
    {
        this->accel_linear[i] = this->registerValue(start_addr);
        start_addr += 2; 
    }
    this->TempC = this->registerValue(start_addr);
    start_addr += 2;
    for(int i = 0; i < 3;i++)
    {
        this->accel_angular[i] = this->registerValue(start_addr); 
        start_addr += 2;
    }
}
void MPU6050::readValues()
{
    int8_t start_addr = 0x3B;
    // Add offset Acceleration Values
    for(int i = 0; i < 3;i++)
    {
        this->accel_linear[i] = round(this->registerValue(start_addr) - this->offsetA[i]) / ACCEL_SENS;
        start_addr += 2;
    }
    this->TempC = this->registerValue(start_addr);
    start_addr += 2; 
    // Add offset Gyro Values
    for(int i = 0; i < 3;i++)
    {
        this->accel_angular[i] = round(this->registerValue(start_addr) - this->offsetG[i]) / GYRO_SENS;
        start_addr += 2;
    }
}

void MPU6050::imuManager(std::future<void> _future)
{
    printf("Starting MPU6050 manager!\n");
    while (_future.wait_for(chrono::milliseconds(RATE)) == std::future_status::timeout)
    {
        this->readValues();
        //X (roll) axis
        this->_accel_angle[0] = atan2(this->accel_linear[2], this->accel_linear[1]) * RAD_TO_DEG - 90.0; //Calculate the angle with z and y convert to degrees and subtract 90 degrees to rotate
        this->_gyro_angle[0] = this->rpy[0] + this->accel_angular[0] * RATE;                         //Use roll axis (X axis)

        //Y (pitch) axis
        _accel_angle[1] = atan2(this->accel_linear[2], this->accel_linear[0]) * RAD_TO_DEG - 90.0; //Calculate the angle with z and x convert to degrees and subtract 90 dethis->accel_angular[0]ees to rotate
        _gyro_angle[1] = this->rpy[1] + this->accel_angular[1] * RATE;                         //Use pitch axis (Y axis)

        //Z (yaw) axis
        if (calc_yaw)
        {
            _gyro_angle[2] = this->rpy[2] + this->accel_angular[2] * RATE; //Use yaw axis (Z axis)
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
    }
    printf("Closing MPU6050 Manager.\n");
}

void MPU6050::Calibrate(int count = 10000)
{
    if(count == 0)
    {
        return;
    }
    printf("Calibrating...");
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
        this->offsetA[i] /= float(count);
        this->offsetG[i] /= float(count);

    }
    // IMP: Remove 1g from the value calculated to compensate for gravity
    this->offsetA[2] -= ACCEL_SENS;
    printf("Done with Calibration!\n");
}

vector<float> MPU6050::RawValues()
{
    //this->readValues();
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
