/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG 
 * Author     : Kunal Singla, Antariksh Narain
 * Original Source: https://www.nxp.com/docs/en/application-note/AN3397.pdf
 * Description: Class for getting position data from an accelerometer of an IMU.
 * 
----------------------------------------------------------------- */

#include "PseudoGPS.h"

/*
* Class Name: PseudoGPS
* Description: Class to calculate X and Y position based on acceleration.
*/
class PseudoGPS
{
private:
	unsigned char stopCount_X, stopCount_Y;
	double acceleration_X[2], acceleration_Y[2];
	double velocity_X[2], velocity_Y[2];
	double position_X[2], position_Y[2];
	double positionScaleFactor = 1; //to be changed as needed
	IMU * imu;

public:
    /*
    * Name: PseudoGPS
    * Constructor Prototype:    PseudoGPS(IMU * input, RangeSensor altitude)
    * Description:  Contructor to initialize PseduoGPS object.
    * Parameters:   IMU * input - IMU pointer to receive acceleration values
    * Side Effects: None
    * Error Conditions: None
    * Return Value: None
    */
	PseudoGPS(IMU * input)
	{
		imu = input;
	}

    /*
    * Name: movementEndCheck
    * Function Prototype:   void movementEndCheck();
    * Description:  If a certain number of acceleration samples are equal to zero
    *               we can assume movement has stopped. Accumulated Error
    *               generated in the velocity calculations is eliminated by
    *               resetting the velocity variables. This stops position
    *               increment and greatly eliminates position error.
    * Parameters:   None
    * Side Effects: None
    * Error Conditions: None
    * Return Value: None
    */
	void movementEndCheck()
	{
		// increment stopCount for direction if acceleration is zero
		if (acceleration_X[1] == 0)
			++stopCount_X;
		else
			stopCount_X = 0; // reset stopCount if accelerating in X direction

		if (acceleration_Y[1] == 0) //we do the same for the Y axis
			++stopCount_Y;
		else
			stopCount_Y = 0;

		// if stopCount exceed 25, set velocity array contents to 0;
		if (stopCount_X >= 25)
		{
			velocity_X[1] = 0;
			velocity_X[0] = 0;
		}

		if (stopCount_Y >= 25) // same for Y axis
		{
			velocity_Y[1] = 0;
			velocity_Y[0] = 0;
		}
	}

    /*
    * Name: averageAccelerationSamples
    * Function Prototype:   void averageAccelerationSamples(unsigned char count);
    * Description:  This function will average 'count' number of acceleration
    *               samples from the IMU.
    * Parameters:   count value from 0 to 255, recommended value of at least 64
    * Side Effects: None
    * Error Conditions: None
    * Return Value: None
    */
	void averageAccelerationSamples(unsigned char count)
	{
        // unsigned char is a max of 255
		for(char i = 0; i < count ; ++i)
		{
			imu->update();
			acceleration_X[1] += imu->getAcceleration()[0];
			acceleration_Y[1] += imu->getAcceleration()[1];
		}
		acceleration_X[1] /= count;
		acceleration_Y[1] /= count;
	}

    /*
    * Name: withinThreshold
    * Function Prototype:   bool withinThreshold(double value, double threshold);
    * Description:  This function will check if the 'value' is within plus or
    *               minus 'threshold'.
    * Parameters:   value - the value to check
    *               threshold - the range to check 'value' in
    * Side Effects: None
    * Error Conditions: None
    * Return Value: true if within range, false otherwise
    */
	bool withinThreshold(double value, double threshold)
	{
		if(value <= threshold && value >= (-1*threshold))
			return true;
		return false;
	}

    /*
    * Name: integrate
    * Function Prototype:   void integrate(double * val, double * integrand);
    * Description:  This function takes the integral given the integrand. Relies
    *               on user's ability to input correct pointers.
    * Example:  vel[1] = vel[0] + acc[0] + ((acc[1] - acc[0]) / 2.0);
    * Parameters:   integrand is the value you want to integrate. Val stores the
    *               value after integration.
    * Side Effects: None
    * Error Conditions: None
    * Return Value: None
    */
    void integrate(double * val, double * integrand)
    {
        // equation to integrate
        val[1] = val[0] + integrand[0] + ((integrand[1] - integrand[0]) / 2.0);
    }

    /*
    * Name: position
    * Function Prototype:   void position();
    * Description:  This function double integrates the acceleration values
    *               coming from an IMU to calculate position.
    * Parameters:   None
    * Side Effects: None
    * Error Conditions: None
    * Return Value: None
    */
	void position()
	{
		
		averageAccelerationSamples(64);

		// discrimination window for acceleration values
		if (withinThreshold(acceleration_X[1], 3))
		{
			acceleration_X[1] = 0;
		}

		if (withinThreshold(acceleration_Y[1], 3))
		{
			acceleration_Y[1] = 0;
		}

		// first X integration:
        integrate(velocity_X, acceleration_X);
		// second X integration:
        integrate(position_X, velocity_X);
        
		// first Y integration:
        integrate(velocity_Y, acceleration_Y);
		// second Y integration:
        integrate(position_Y, velocity_Y);

		// overwriting the old value with new value
		acceleration_X[0] = acceleration_X[1];
		acceleration_Y[0] = acceleration_Y[1];

		velocity_X[0] = velocity_X[1];
		velocity_Y[0] = velocity_Y[1];

		// [OPTIONAL] USED FOR SCALING MEASUREMENTS
		position_X[1] = position_X[1] * positionScaleFactor; 
		position_Y[1] = position_Y[1] * positionScaleFactor;

        //[OPTIONAL] RESCALE MEASUREMENTS, (prone to floating point issues)
		position_X[1] = position_X[1] / positionScaleFactor;
		position_Y[1] = position_Y[1] / positionScaleFactor;
		movementEndCheck();

		//overwrite old position value with new value
		position_X[0] = position_X[1];
		position_Y[0] = position_Y[1];
	}
    
     /*
    * Name: getPosition
    * Function Prototype:   double * getPosition();
    * Description:  This function returns an array containing 2 double values:
    *               the positional X and Y values.
    * Parameters:   None
    * Side Effects: USING MALLOC REQUIRES FREEING THE VARIABLE AFTER USE
    * Error Conditions: None
    * Return Value: Array containing position X and Y coordinates.
    */
    double * getPosition()
    {
        double * position;

        //allocating array on heap. Requires this to be freed after use.
        position = (double *)(malloc(2 * sizeof(double)));

        //position[0] is the latest X coord
        position[0] = position_X[1];
        //position[1] is the latest Y coord
        position[1] = position_Y[1];

        return position;
    }
};