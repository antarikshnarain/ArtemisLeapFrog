/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG 
 * Author     : Kunal Singla, Antariksh Narain
 * Description: 
 * 
----------------------------------------------------------------- */

#include "IMU.h"
#include "RangeSensor.h"
#include <stdlib.h>


class PseudoGPS
{
private:
	unsigned char stopCount_X, stopCount_Y;
	double acceleration_X[2], acceleration_Y[2];
	double velocity_X[2], velocity_Y[2];
	double position_X[2], position_Y[2];
	double positionScaleFactor;
	IMU * imu;
public:
    PseudoGPS(IMU * input, RangeSensor altitude);
    void movementEndCheck();
    void averageAccelerationSamples(unsigned char count);
    bool withinThreshold(double value, double threshold);
    void integrate(double * val, double * integrand);
    void position();
    double * getPosition();
};
