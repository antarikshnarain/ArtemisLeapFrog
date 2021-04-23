#include <wiringPi.h>
#include <iostream>
#include <unistd.h>
using namespace std;

int main()
{
    wiringPiSetupGpio();
    pinMode(18, OUTPUT);
    digitalWrite(18, HIGH);
    sleep(3);
    digitalWrite(18, LOW);
    return 0;
}