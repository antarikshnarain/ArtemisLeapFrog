// Copyright 2020 Space Engineering Research Center
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
//
// Author: Antariksh Narain

#include <utilities/PIDController.hpp>
/*
Config KP, KI, KD, h, Tf, Tt
Working Value: Kp=30.0, KI=0.005, KD=10.0, h=1, Tf=10, Tt=10, Thr=(9.6, 12.0)
*/
namespace leapfrog_utilities
{
    double PIDController::clamp(double val, double min, double max)
    {
        if (val < min)
            return min;
        else if (val > max)
            return max;
        return val;
    }

    PIDController::PIDController(double min_val, double max_val, double h, double Tf, double Tt)
    {
        ulow = min_val;
        uhigh = max_val;
        bi = KI * h;
        ad = Tf / (Tf + h);
        bd = KD / (Tf + h);
        br = h / Tt;
        exp_val_old = 0;
    }

    double PIDController::Update(double act_val, double exp_val)
    {
        // Compute proportional part
        P = KP * (act_val - exp_val);
        D = ad * D - bd * (exp_val - exp_val_old);
        double v = P + I + D;
        printf("%f %f %f %f\n", P, I, D, v);
        //double u = sat(v, ulow, uhigh);
        double u = clamp(v, ulow, uhigh);
        I = I + bi * (act_val - exp_val) + br * (u - v);
        exp_val_old = exp_val;
        //sleep();
        return u;
    }
} // namespace leapfrog_utilities