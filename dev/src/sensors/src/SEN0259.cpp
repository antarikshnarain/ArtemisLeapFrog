/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG 
 * Author     : Kunal Singla, Antariksh Narain
 * Description: 
 * 
----------------------------------------------------------------- */

#include "sensors/SEN0259.hpp"

SEN0259::SEN0259(string port, int baudrate):Serial(port, baudrate)
{
    this->distance = 0;
    this->strength = 0;
}

bool SEN0259::readData()
{
    this->sensor_data = this->Recv(MSG_SIZE);
    if(this->sensor_data[0] != 0x59 || this->sensor_data[1] != 0x59)
    {
        return false;
    }
    this->distance = this->sensor_data[2] | this->sensor_data[3] << 8;
    this->strength = this->sensor_data[4] | this->sensor_data[5] << 8;
    // Verify checksum
    int sum = 0;
    for(int i=0;i<MSG_SIZE-1;i++)
    {
        sum += this->sensor_data[i];
    }
    return sum == this->sensor_data[MSG_SIZE-1];
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
#include <unistd.h>
int main(int argc, char *argv[])
{
    if(argc != 3)
    {
        printf("Pass port and baudrate as parameters"\n);
        return -1;
    }
    SEN0259 sen(string(argv[1]),atoi(argv[2]));
    int dist, sig;
    while(1)
    {
        dist = sen.GetDistance();
        sig = sens.GetStrength();
        printf("%dm --%d\n",dist,sig);
        std::usleep(10000);
    }
    return 0;
}
#endif