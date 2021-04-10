"""
Author: Antariksh Narain

Description:
The following python script takes input from the IMU sensor and does the following
1. Calibrate the sensor
2. Pass through a Kalman filter
3. Calculates roll and pitch
*4. Mean correction with other sensor value
"""

import numpy as np
import smbus
from time import sleep, time, time_ns
from collections import namedtuple

DataProperties = namedtuple('SensorProp', 'mean var min max')

class MPU6050:

    #some MPU6050 Registers and their Address
    PWR_MGMT_1   = 0x6B
    SMPLRT_DIV   = 0x19
    CONFIG       = 0x1A
    GYRO_CONFIG  = 0x1B
    INT_ENABLE   = 0x38
    ACCEL_XOUT_H = 0x3B
    ACCEL_YOUT_H = 0x3D
    ACCEL_ZOUT_H = 0x3F
    GYRO_XOUT_H  = 0x43
    GYRO_YOUT_H  = 0x45
    GYRO_ZOUT_H  = 0x47

    def __init__(self, device_addr=0x68):
        self.bus = smbus.SMBus(1)
        self.Device_Address = device_addr
        self.properties = []

        #write to sample rate register
        self.bus.write_byte_data(self.Device_Address, self.SMPLRT_DIV, 7)
        
        #Write to power management register
        self.bus.write_byte_data(self.Device_Address, self.PWR_MGMT_1, 1)
        
        #Write to Configuration register
        self.bus.write_byte_data(self.Device_Address, self.CONFIG, 0)
        
        #Write to Gyro configuration register
        self.bus.write_byte_data(self.Device_Address, self.GYRO_CONFIG, 24)
        
        #Write to interrupt enable register
        self.bus.write_byte_data(self.Device_Address, self.INT_ENABLE, 1)

    def RawSensorValues(self):
        #Read Accelerometer raw value
        acc_x = self._read_raw_data(self.ACCEL_XOUT_H)
        acc_y = self._read_raw_data(self.ACCEL_YOUT_H)
        acc_z = self._read_raw_data(self.ACCEL_ZOUT_H)
        
        #Read Gyroscope raw value
        gyro_x = self._read_raw_data(self.GYRO_XOUT_H)
        gyro_y = self._read_raw_data(self.GYRO_YOUT_H)
        gyro_z = self._read_raw_data(self.GYRO_ZOUT_H)
        
        #Full scale range +/- 250 degree/C as per sensitivity scale factor
        Ax = acc_x/16384.0
        Ay = acc_y/16384.0
        Az = acc_z/16384.0
        
        Gx = gyro_x/131.0
        Gy = gyro_y/131.0
        Gz = gyro_z/131.0        
        return np.array([Ax,Ay,Az,Gx,Gy,Gz], dtype=np.float)

    def MeanSensorValues(self):
        datas = []
        # s_time = time()
        # while (time() - s_time) < 1:
        #     datas.append(self.RawSensorValues())
        
        s_time = time_ns()
        while (time_ns() - s_time) < 1000000:
            datas.append(self.RawSensorValues())

        datas = np.array(datas)
        return datas.mean(axis=0)

    def RollPitch(self, Ax, Ay, Az):
        pitch = 180 * np.arctan2(Ax, np.sqrt(Ay**2+ Az**2))/np.pi
        roll = 180 * np.arctan2(Ay, np.sqrt(Ax**2 + Az**2))/np.pi
        return (roll,pitch)


    def Calibrate(self, duration_sec=10):
        datas = []
        s_time = time()
        while (time() - s_time) < duration_sec:
            datas.append(self.RawSensorValues())
        
        datas = np.array(datas)
        for i in range(6):
            self.properties.append(
                DataProperties(datas[:,i].mean(), datas[:,i].var(), datas[:,i].min(), datas[:,i].max()))


    def _read_raw_data(self, addr):
        #Accelero and Gyro value are 16-bit
        high = self.bus.read_byte_data(self.Device_Address, addr)
        low = self.bus.read_byte_data(self.Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value

    
if __name__ == "__main__":
    print (" Reading Data of Gyroscope and Accelerometer")
    mpu = MPU6050()
    mpu2 = MPU6050(0x69)
    np.set_printoptions(precision=2)
    np.set_printoptions(suppress=True)
    print(mpu.RawSensorValues())
    while True:
        values = mpu.MeanSensorValues()
        values2 = mpu2.MeanSensorValues()
        if len(values) == 0:
            continue
        #print(values, end='\t')
        #print("R=%.2f P=%.2f"%mpu.RollPitch(values[0],values[1],values[2]))
        print(values, values2, values - values2)
        print("R1=%.2f P1=%.2f"%mpu.RollPitch(values[0],values[1],values[2]), end='\t')
        print("R2=%.2f P2=%.2f"%mpu2.RollPitch(values2[0],values2[1],values2[2]))
        