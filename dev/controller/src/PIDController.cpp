/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG 
 * Author     : Antariksh Narain
 * Description: Library to control actuation of relay
----------------------------------------------------------------- */

#include "../include/PIDController.h"

using namespace Utility;

PIDController::PIDController(double KP, double KI, double KD, double time_step_ms, double windguard = 0)
{
    time_step_ms /= 1000;
    this->Kp = KP;
    this->Ki = KI * time_step_ms;
    this->Kd = KD / time_step_ms;
    this->WindGuard = windguard;
}

double PIDController::Update(double r_t, double y_t)
{
    // Calculate error in values
    error = r_t - y_t;
    // Calculate values
    P = Kp * error;
    I += Ki * error;
    D = Kd * (r_t - old_r_t);
    // Store value
    old_r_t = r_t;
    // Return summing junction
    return CONSTRAIN(P + I - D, -this->WindGuard, this->WindGuard);
}