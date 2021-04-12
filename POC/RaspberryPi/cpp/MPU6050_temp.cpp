#include <iostream>
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

using namespace std;

int f_dev;
const int MPU_ADDR = 0x68;

bool InitIMU()
{
    f_dev = open("/dev/i2c-1", O_RDWR);
    if(f_dev < 0)
    {
        cout << "F_dev failed!";
        return false;
    }
    if(ioctl(f_dev, I2C_SLAVE, MPU_ADDR) < 0)
    {
        cout << "IOCTL failed!";
        return false;
    }

    i2c_smbus_write_byte_data(f_dev, SMPLRT_DIV, 7);   /* Write to sample rate register */
    i2c_smbus_write_byte_data(f_dev, PWR_MGMT_1, 1);   /* Write to sample rate register */
    i2c_smbus_write_byte_data(f_dev, CONFIG, 0);   /* Write to sample rate register */
    i2c_smbus_write_byte_data(f_dev, GYRO_CONFIG, 24);   /* Write to sample rate register */
    i2c_smbus_write_byte_data(f_dev, 0x38, 1);   /* Write to sample rate register */
    return true;
}

int main()
{
    if(!InitIMU())
    {
        return -1;
    }
    for(int i = 0x3b; i <= 0x48; i+=2)
    {
        cout << ((i2c_smbus_read_byte_data(f_dev, i) << 8) | (i2c_smbus_read_byte_data(f_dev, i+1))) << "\t";
    }
    cout << endl;
    return 0;
}