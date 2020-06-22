#include<stdio.h>
/*
Config KP, KI, KD, h, Tf, Tt
1. 1.5, 0.003, 0.3, 0.1, 1, 10
1. 2.0, 0.003, 0.5, 0.1, 1, 10
1. 3.0, 0.0003, 0.5, 0.1, 1, 10 - slow ocillation rise
1. 4.0, 0.0003, 0.8, 0.1, 1, 10 - slow ocillation rise
1. 5.0, 0.0003, 0.8, 9.5, 11.0, 0.1, 1, 10 - lower hovering
1. 4.0, 0.0003, 10.0, 9.5, 11.0, 0.1, 1, 10 - slow rising
1. 10.0, 0.003, 10.0, 9.5, 11.0, 0.1, 1, 10 - better rising
*/
class PIDController
{
private:
    const double KP = 20.0, KI = 0.01, KD = 10.0;
    double bi, ad, bd, br;
    double exp_val_old;
    double P, I, D;
    double ulow, uhigh;
    double clamp(double val, double min, double max)
    {
        if (val < min)
            return min;
        else if (val > max)
            return max;
        return val;
    }

public:
    PIDController(double min_val, double max_val, double h, double Tf, double Tt)
    {
        ulow = min_val;
        uhigh = max_val;
        bi = KI * h;
        ad = Tf / (Tf + h);
        bd = KD / (Tf + h);
        br = h / Tt;
        exp_val_old = 0;
    }
    double Update(double act_val, double exp_val)
    {
        // Compute proportional part
        double P = KP * (act_val - exp_val);
        double D = ad * D - bd * (exp_val - exp_val_old);
        double v = P + I + D;
        printf("%f %f %f %f\n", P, I, D, v);
        //double u = sat(v, ulow, uhigh);
        double u = clamp(v, ulow, uhigh);
        I = I + bi * (act_val - exp_val) + br * (u - v);
        exp_val_old = exp_val;
        //sleep();
        return u;
    }
};