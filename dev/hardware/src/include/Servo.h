/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG 
 * Author     : Antariksh Narain
 * Description: Library to control Servo signals
 * Reference  : https://biicode-docs.readthedocs.io/raspberrypi/examples/wiringpi.html
----------------------------------------------------------------- */

#ifdef __ARDUINO__
#include <arduino.h>
#include <servo.h>
#elif __Raspberry_PI__
#include <wiringPi.h>
#include <softServo.h>
#endif

namespace Actuators
{
    class Servo
    {
    private:
        int pin;
        double pos;

    public:
        Servo() {}
        // Initialize the Servo pin
        Servo(int pin);
        // Set the position of the servo
        void update(double position);
    };

} // namespace Actuators