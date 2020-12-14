/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG 
 * Author     : Kunal Singla, Antariksh Narain
 * Description: 
 * 
----------------------------------------------------------------- */

#include<vector>

using namespace std;

#pragma region REGISTER VALUES
#define SMPLRT_DIV      0x19
#define CONFIG          0x1A
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define FIFO_EN         0x23
#define INT_ENABLE      0x38

#define ACCEL_XOUT_H    0x3B
#define ACCEL_YOUT_H    0x3C
#define ACCEL_ZOUT_H    0x3D
#define TEMP_OUT_H      0x41
#define GYRO_XOUT_H     0x43
#define GYRO_YOUT_H     0x44
#define GYRO_ZOUT_H     0x45

#define PWR_MGMT_1      0x6B
#pragma endregion

class MPU6050
{
    private:
    float Ax, Ay, Az;
    float Gx, Gy, Gz;
    float TempC;
    int id;

    private:
    // Function to read raw register value
    short registerValue(int addr);

    protected:
    double roll, pitch;

    public:
    // Function to Initialize with the selected device address
    MPU6050(int device_address);

    // Function to return RAW values from the sensor
    vector<float> RawValues();
    
    // Function to calculate Pitch and Roll
    vector<double> PrincipalAxisValues();

};