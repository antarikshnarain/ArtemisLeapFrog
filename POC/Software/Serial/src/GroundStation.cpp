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

// User Defined Libs
#include "Serial.hpp"

using namespace std;

Serial *serial;

void Receiver(std::future<void> fut)
{
    printf("Started Receiver...\n");
    while (fut.wait_for(chrono::milliseconds(1)) == std::future_status::timeout)
    {
        string recv;
        if(serial->IsAvailable())
        {
            recv = serial->Recv();
            printf("\r>>%s\n>", recv.c_str());
        }
    }
    printf("Exiting Receiver!\n");
}

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        printf("Pass Communication port name and baud rate as parameter.\n");
        return -1;
    }

    //Serial serial(argv[1], atoi(argv[2]),'\n',1000,-1);
    serial = new Serial(argv[1], atoi(argv[2]),'\n',1000,-1);

    // Promise to manage threads
    promise<void> exit_signal;
    future<void> exit_future = exit_signal.get_future();
    
    thread t2(&Receiver, move(exit_future));
    printf("Started Sender...\n");
    char data[100];
    while (1)
    {
        cout << "\n>";
        cin.getline(data, sizeof(data));
        //send_data = string(data);
        if (data == "exit")
        {
            exit_signal.set_value();
            break;
        }
        else
        {
            serial->Send(data);
        }
    }
    printf("Exiting Sender!\n");
    t2.join();
    return 0;
}