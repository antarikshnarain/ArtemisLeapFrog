/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG 
 * Author     : Antariksh Narain
 * Description: Library to control linear actuator
 * Version1   : Simple actuator without feedback
----------------------------------------------------------------- */

#include <Relay.h>
#include <Servo.h>

// Might need a header file for analog read

// 0-> Simple; 1->PositionFeedback; 2->ServoSignal
#define MODE_LA 0
#define FORWARD_PIN 10
#define BACKWARD_PIN 11

namespace Actuators
{
    class LinearActuator
    {
    private:
        double current_position;
        int signal_pin;
        Relay mov_f;
        Relay mov_b;
        Servo mov_p;

    protected:
       // Move Actuator forward
        void forward();
        // Move Actuator backward
        void backward();
        // Stop the Actuator
        void halt();

    public:
        // \brief Initialize Position based Linear Actuator
        // \param pin servo input pin or potentiometer pin
        LinearActuator(int);

        // \brief Move the Linear Actuator to the position
        // \param position desired position of the actuator
        bool move(double);

        // \brief Get Position of actuator
        double position();

    };
} // namespace Actuators