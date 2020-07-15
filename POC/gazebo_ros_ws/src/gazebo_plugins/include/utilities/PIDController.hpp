// Copyright 2020 Space Engineering Research Center
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
//
// Author: Antariksh Narain

#include <stdio.h>

#ifndef UTILITIES__PID_CONTROLLER_HPP
#define UTILITIES__PID_CONTROLLER_HPP

/*
Config KP, KI, KD, h, Tf, Tt
Working Value: Kp=30.0, KI=0.005, KD=10.0, h=1, Tf=10, Tt=10, Thr=(9.6, 12.0)
*/
namespace leapfrog_utilities
{
    class PIDController
    {
    private:
        const double KP = 30.0, KI = 0.005, KD = 10.0;
        double bi, ad, bd, br;
        double exp_val_old;
        double P, I, D;
        double ulow, uhigh;

    protected:
        double clamp(double val, double min, double max);

    public:
        PIDController(double min_val, double max_val, double h, double Tf, double Tt);
        double Update(double act_val, double exp_val);
    };
} // namespace leapfrog_utilities

#endif // UTILITIES__PID_CONTROLLER_HPP