/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG 
 * Author     : Antariksh Narain
 * Description: Library to control actuation of relay
----------------------------------------------------------------- */

#ifdef __ARDUINO__
#include <arduino.h>
#elif __Raspberry_PI__
#include <wiringPi.h>
#endif

namespace Actuators
{
    class Relay
    {
    private:
        int pin;
        bool state;

    public:
        Relay() {}
        // Initialize the relay pin
        Relay(int pin, bool state = false);
        // Toggle the state of the pin
        void toggle();
        // Set state of the relay pin
        void update(bool state);
    };

} // namespace Actuators