

class IMU
{
private:
    //0 - acceleration_x, 1 - acceleration_y, 2 - acceleration_z
    double acceleration[3];
    //0 - yaw, 1 - pitch, 2 - roll
    double gyro[3];
public:
    IMU(int port);
    void init();
    void calibrate();
    void update(); //updates measurement values
    double * getAcceleration(); //returns acceleration array
};