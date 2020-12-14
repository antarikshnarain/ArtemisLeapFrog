/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG 
 * Author     : Antariksh Narain
 * Description: Library to control linear actuator
 * Version1   : Simple actuator without feedback
----------------------------------------------------------------- */

#include <Relay.h>
#include <Servo.h>

namespace Actuators
{
    class LinearActuator
    {
    private:
        Relay mov_f;
        Relay mov_b;
        Servo mov_p;

    public:
        // Initialize Position based Linear Actuator
        LinearActuator(int servo_pin);
        // Initialize Linear Actuator
        LinearActuator(int forward_pin, int backward_pin);
        // Move Actuator forward
        void forward();
        // Move Actuator backward
        void backward();
        // Stop the Actuator
        void halt();
        // \return position of the actuator
        double position();
        // Move the actuator to desired position.
        // \param position of the actuator
        void move (double position);
    };
} // namespace Actuators