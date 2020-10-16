/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG 
 * Author     : Antariksh Narain
 * Description: Library to control Servo signals
----------------------------------------------------------------- */

#include "Servo.h"

using namespace Actuators;

Servo::Servo(int pin)
{
    this->pin = pin;
    this->pos = 0;
#ifdef __ARDUINO__
// Add Code
#elif __Raspberry_PI__
    softServoSetup(this->pin);
#endif
}

void Servo::update(double position)
{
    this->pos = position;
#ifdef __ARDUINO__
// Add Code
#elif __Raspberry_PI__
    softServoWrite(this->pin, this->pos);
#endif
}
