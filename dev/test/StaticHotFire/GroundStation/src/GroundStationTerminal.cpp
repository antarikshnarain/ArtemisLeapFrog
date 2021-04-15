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
// User Defined Libs
#include <Serial.hpp>

#include <bits/stdc++.h>

using namespace std;

Serial *serial;
mutex _mutex;

void Receiver(std::future<void> fut)
{
    printf("\rStarted Receiver...\n");
    while (fut.wait_for(chrono::milliseconds(1)) == std::future_status::timeout)
    {
        string recv;
        if(serial->IsAvailable())
        {
            recv = serial->convert_to_string(serial->Recv());
            printf("\r>>%s\n", recv.c_str());
        }
    }
    printf("Exiting Receiver!\n");
}

void Sender(promise<void> exit_promise)
{
    printf("Started Sender...\n");
    char data[100];
    while (1)
    {
        cin.getline(data, sizeof(data));
        int num = atoi(data);
        if (string(data) == "exit")
        {
            printf("Exiting ...\n");
            _mutex.lock();
            printf("Sending exit command to other node.\n");
            serial->Send("exit");
            _mutex.unlock();
            sleep(1);
            exit_promise.set_value();
            break;
        }
        else
        {
            _mutex.lock();
            serial->Send(data);
            _mutex.unlock();
        }
    }
    printf("Exiting Sender!\n");
}

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        printf("Pass Communication port name and baud rate as parameter.\n");
        //printf("Pass monitoring script name and test script name too.\n");
        return -1;
    }

    serial = new Serial(argv[1], atoi(argv[2]),'\n',1000,-1);

    promise<void> exit_signal;
    future<void> exit_future;
    exit_future = exit_signal.get_future();
    thread th_recv(&Receiver, move(exit_future));
    printf("Started Recevier Thread!\n");
    thread th_send(&Sender, move(exit_signal));
    th_send.join();
    th_recv.join();
    printf("Closed primary threads! ");
    printf("Closed all threads safely!\n");
    return 0;
}