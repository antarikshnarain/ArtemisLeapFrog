
#ifndef _WPI2C_
#define _WPI2C_

int wiringPiI2CSetup(int v)
{
    return v;
}

void wiringPiI2CWriteReg8(int a, int b,int c)
{}

int wiringPiI2CReadReg8(int a, int b)
{
    return a+b;
}

#endif