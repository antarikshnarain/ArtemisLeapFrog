#include "../include/RangeSensor.h"

int main(int argc, char **argv) 
{
    SEN0259 RangeSensor1;
    RangeSensor1.begin("/dev/ttyUSB0");
    int iterations = 100;
    for(int i = 0; i < iterations; ++i)
    {
        RangeSensor1.measure();
        std::cout << RangeSensor1.getDistance() << std::endl;
        std::cout << RangeSensor1.getStrength() << std::endl;
    }

    RangeSensor1.close();
    
}