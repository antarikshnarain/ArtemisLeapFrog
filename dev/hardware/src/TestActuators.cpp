#include "TestActuators.hpp"

namespace Actuators
{
    void digitalWrite(int pin, bool value)
    {
        printf("DigitalWrite %d set to %d\n", pin, value);
    }
} // namespace Actuators