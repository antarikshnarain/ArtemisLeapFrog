/*
Author : Antariksh Narain
Email  : antariksh.cloud@gmail.com
Project: LEAPFROG|SERC|ISI|USC
License: MIT License
*/

#include "Arduino.h"

class LinearActuator
{
private:
    // Forward pin
    int FORWARD_pin;
    // Backward pin
    int BACKWARD_pin;
    // Current Sensor input pin
    int analogIn;
    // Current sensor forward and backward threshold
    double forwardThres, backwardThres;

public:
    /*
    Constructor to initialize object for linear actuator
    */
    LinearActuator(int digitalPin1, int digitalPin2, int analogPin, double thresholdF, double thresholdB);
    /*
    Move the actuator forward
    */
    void forward();
    /*
    Move the actuator backward
    */
    void backward();
    /*
    Stop the actuator
    */
    void hault();
    /*
    Get current value
    */
    int CurrentValue();
};