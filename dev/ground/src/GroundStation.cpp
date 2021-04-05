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
#include "Serial.hpp"

using namespace std;

Serial *serial;
mutex _mutex;

// command, duration_ms
#define TOTAL_CMDS 2 // 1 + num in heart_beats
map<string,int> heart_beats = 
{
    {"t 1",500}
};

void Receiver(std::future<void> fut)
{
    printf("\rStarted Receiver...\n");
    while (fut.wait_for(chrono::milliseconds(1)) == std::future_status::timeout)
    {
        string recv;
        if(serial->IsAvailable())
        {
            recv = serial->Recv();
            printf("\r>>%s\n", recv.c_str());
        }
    }
    printf("Exiting Receiver!\n");
}

void Sender(std::promise<void> * prom)
{
    printf("Started Sender...\n");
    char data[100];
    while (1)
    {
        cin.getline(data, sizeof(data));
        if (string(data) == "exit")
        {
            printf("Exiting ...\n");
            for(int i=0;i<TOTAL_CMDS;i++)
            {
                prom[i].set_value();
            }
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

void HeartBeats(std::future<void> fut, string cmd, int sleep_time)
{
    while (fut.wait_for(chrono::milliseconds(sleep_time)) == std::future_status::timeout)
    {
        _mutex.lock();
        serial->Send(cmd);
        _mutex.unlock();
    }
}

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        printf("Pass Communication port name and baud rate as parameter.\n");
        return -1;
    }

    serial = new Serial(argv[1], atoi(argv[2]),3,1000,-1);

    // Promise to manage threads
    promise<void> exit_signals[TOTAL_CMDS];
    future<void> exit_futures[TOTAL_CMDS];
    for(int i=0;i<TOTAL_CMDS;i++)
    {
        exit_futures[i] = exit_signals[i].get_future();
    }
    
    thread t2(&Receiver, move(exit_futures[0]));
    thread t1(&Sender, move(exit_signals));
    sleep(1);
    thread pool[TOTAL_CMDS - 1];
    auto itr = heart_beats.begin();
    int i=0;
    while(itr != heart_beats.end())
    {
        pool[i++] = thread(&HeartBeats, move(exit_futures[i+1]), itr->first, itr->second);
        itr++;
    }

    t1.join();
    t2.join();
    printf("Closed primary threads! ");
    for(int i=0;i<TOTAL_CMDS - 1;i++)
    {
        pool[i].join();
    }
    printf("Closed all threads safely!\n");
    return 0;
}