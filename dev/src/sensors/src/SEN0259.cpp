/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG 
 * Author     : Kunal Singla, Antariksh Narain
 * Description: 
 * 
----------------------------------------------------------------- */

#include "sensors/SEN0259.hpp"
#include <string>
SEN0259::SEN0259(string port, int baudrate):Serial(port, baudrate, '\0', 1000, 9)
{
    this->distance = 0;
    this->strength = 0;
}

bool SEN0259::readData()
{
    this->sensor_data = this->Recv();
    auto sen_data = this->sensor_data.c_str();
    //int sen_data[9] = {0};
    // for(int i=0;i<9;i++)
    // {
    //     sen_data[i] = atoi(sen_data[i]);
    // }
    if(sen_data[0] != 0x59 || sen_data[1] != 0x59)
    {
        return false;
    }
    this->distance = sen_data[2] + (sen_data[3] << 8);
    this->strength = sen_data[4] + (sen_data[5] << 8);
    // Verify checksum
    int sum = 0;
    for(int i=0;i<8;i++)
    {
        sum += sen_data[i];
    }
    return sum == sen_data[8];
}

int SEN0259::GetDistance()
{
    if(!this->readData())
    {
        printf("Checksum failed!\n");
    }
    return this->distance;
}
int SEN0259::GetStrength()
{
    return this->strength;
}


#ifdef TEST
#include<bits/stdc++.h>
#include <unistd.h>
#include <time.h>
int main(int argc, char *argv[])
{
    if(argc != 3)
    {
        printf("Pass port and baudrate as parameters\n");
        return -1;
    }
    SEN0259 sen(string(argv[1]),atoi(argv[2]));
    int dist, sig;
    while(1)
    {
        dist = sen.GetDistance();
        sig = sen.GetStrength();
        printf("%dm --%d\n",dist,sig);
        usleep(10000);
    }
    return 0;
}
#endif