/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG 
 * Author     : Antariksh Narain
 * Description: Library to control linear actuator
 * Version1   : Simple actuator without feedback
----------------------------------------------------------------- */

#include "LinearActuator.h"

using namespace Actuators;

LinearActuator::LinearActuator(int servo_pin)
{
    this->mov_p = Servo(servo_pin);
}

LinearActuator::LinearActuator(int forward_pin, int backward_pin)
{
    this->mov_f = Relay(forward_pin);
    this->mov_b = Relay(backward_pin);
}

void LinearActuator::forward()
{
    this->mov_f.update(true);
    this->mov_b.update(false);
}

void LinearActuator::backward()
{
    this->mov_f.update(false);
    this->mov_b.update(true);
}

void LinearActuator::halt()
{
    this->mov_f.update(false);
    this->mov_b.update(false);
}

double LinearActuator::position()
{
    return 0;
}

void LinearActuator::move(double position)
{
    this->mov_p.update(position);
}