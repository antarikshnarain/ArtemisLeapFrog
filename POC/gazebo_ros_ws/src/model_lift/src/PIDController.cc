#include<stdio.h>
/*
Config KP, KI, KD, h, Tf, Tt
Working Value: Kp=30.0, KI=0.005, KD=10.0, h=1, Tf=10, Tt=10, Thr=(9.6, 12.0)
*/
class PIDController
{
private:
    const double KP = 30.0, KI = 0.005, KD = 10.0;
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