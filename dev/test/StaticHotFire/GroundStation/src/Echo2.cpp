/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Main program for Ground station
 * Provides a terminal like interface to send and recv data.
----------------------------------------------------------------- */

// STL Libs
#include <iostream>
#include <functional>
#include <stdio.h>
#include <string>
#include <string.h>
#include <chrono>
#include <thread>
#include <future>
#include <map>
#include <mutex>
#include <random>
#include <chrono>
#include <thread>
// User Defined Libs
#include <Serial.hpp>

using namespace std;

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        printf("Pass Communication port name and baud rate as parameter.\n");
        return -1;
    }
    Serial serial(argv[1], atoi(argv[2]),'\n',1000,-1);
    printf("\rStarted Receiver...\n");
    while (1)
    {
        if(serial.IsAvailable())
        {
            printf("Serial buffer has data.\n");
            auto recv = serial.Recv();
            string data = serial.convert_to_string(recv);
            if(data == "exit")
            {
                break;
            }
            serial.Send(recv);
        }
        else
        {
            this_thread::sleep_for(chrono::milliseconds(200));
            printf("Sleeping\n");
        }
    }
    printf("Exiting Receiver!\n");
    printf("Closed primary threads!\n");
    return 0;
}