/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG 
 * Author     : Antariksh Narain
 * Description: Library to control actuation of relay
----------------------------------------------------------------- */

#define CONSTRAIN(value, min, max) (value > max) ? max : ((value < min) ? min : value)
#define MIN(a,b) a<b?a:b
#define MAX(a,b) a>b?a:b

namespace Utility
{
    /* Class: PID Controller
     * The system computed the value to be fed in plant based on the
     * expected and actual state of the system.
     * The Gains are simplified using the time steps
    */
    class PIDController
    {
    private:
        double Kp, Ki, Kd;
        double P, I, D;
        double WindGuard;
        double old_r_t, error;

    public:
        // Set gains for PID controller
        PIDController(double KP, double KI, double KD, double time_step_ms, double windguard = 0);
        // Updated Gains
        double Update(double expected, double actual);
    }
} // namespace Utility