#include "../include/RangeSensor.h"

#define PORT "/dev/ttyUSB0"

int main(int argc, char **argv) 
{
    SEN0259 RangeSensor1;
    std::cout << "BEGINNING TO OPEN DEVICE" << std::endl;
    RangeSensor1.begin(PORT);

    int iterations = 100;

    std::cout << "BEGINNING TO OUTPUT MEASUREMENTS" << std::endl;
    for(int i = 0; i < iterations; ++i)
    {
        RangeSensor1.measure();
        std::cout << RangeSensor1.getDistance() << std::endl;
        std::cout << RangeSensor1.getStrength() << std::endl;
    }
    std::cout << "DONE WITH MEASUREMENTS" << std::endl;

    std::cout << "BEGINNING TO CLOSE DEVICE" << std::endl;
    RangeSensor1.close();
    std::cout << "CLOSED DEVICE" << std::endl;
    
}