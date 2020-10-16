/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG 
 * Author     : Antariksh Narain
 * Description: Library to control actuation of relay
----------------------------------------------------------------- */

#include "Relay.h"

using namespace Actuators;

Relay::Relay(int pin, bool state = false)
{
    this->pin = pin;
    this->state = state;
#ifdef __ARDUINO__
    pinMode(this->pin, OUTPUT);
    digitalWrite(this->pin, this->state);
#elif __Raspberry_PI__
    wiringPiSetup();
    pinMode(this->pin, OUTPUT);
    digitalWrite(this->pin, this->state);
#endif
}

void Relay::toggle()
{
    this->state = !this->state;
#ifdef __ARDUINO__
    digitalWrite(this->pin, this->state);
#elif __RASPBERRYPI__
    digitalWrite(this->pin, this->state);
#endif
}

void Relay::update(bool state)
{
    this->state = state;
#ifdef __ARDUINO__
    digitalWrite(this->pin, this->state);
#elif __RASPBERRYPI__
    digitalWrite(this->pin, this->state);
#endif
}