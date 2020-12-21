/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG 
 * Author     : Antariksh Narain
 * Description: Library to control linear actuator
 * Version1   : Simple actuator without feedback
----------------------------------------------------------------- */

#include "LinearActuator.hpp"

namespace Actuators
{
    LinearActuator::LinearActuator(int pin=-1)
    {
        this->signal_pin = pin;
        #if MOVE_LA == 0
        this->mov_p = Servo(this->signal_pin);
        #else
        this->mov_f = Relay(FORWARD_PIN);
        this->mov_b = Relay(BACKWARD_PIN);
        #endif
    }

    void LinearActuator::forward()
    {
#if MODE_LA == 0
        this->mov_p.update(this->current_position);
#elif MODE_LA == 1
        this->mov_f.update(true);
        this->mov_b.update(false);
#else
        this->mov_f.update(true);
        this->mov_b.update(false);
#endif
    }

    void LinearActuator::backward()
    {
#if MODE_LA == 0
        this->mov_p.update(this->current_position);
#elif MODE_LA == 1
        this->mov_f.update(false);
        this->mov_b.update(true);
#else
        this->mov_f.update(false);
        this->mov_b.update(true);
#endif
    }

    void LinearActuator::halt()
    {
#if MOVE_LA == 1 || MOVE_LA == 2
        this->mov_f.update(false);
        this->mov_b.update(false);
#endif
    }

    double LinearActuator::position()
    {
        return this->current_position;
    }

    bool LinearActuator::move(double position)
    {
        #if MOVE_LA == 0
        this->mov_p.update(position);
        #elif MOVE_LA == 1
        if(this->current_position < position)
        {
            this->forward();
            while(this->current_position < position)
            {
                this->current_position = analogRead(this->signal_pin);
            }
            this->halt();
        }
        else if (this->current_position > position)
        {
            this->backward();
            while(this->current_position > position)
            {
                this->current_position = analogRead(this->signal_pin);
            }
            this->halt();
        }
        #else
        if(position > 0)
        {
            this->forward();
        }
        else if(position < 0)
        {
            this->backward();
        }
        else
        {
            this->halt();
        }
        #endif
        this->current_position = position;

    }
} // namespace Acutuators