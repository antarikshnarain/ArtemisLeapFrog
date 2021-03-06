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
<<<<<<< HEAD
=======
int enable_echo = 0;
typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::milliseconds milliseconds;
Clock::time_point t0;
Clock::time_point t1;

string random_string(std::size_t length)
{
    const std::string CHARACTERS = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

    std::random_device random_device;
    std::mt19937 generator(random_device());
    std::uniform_int_distribution<> distribution(0, CHARACTERS.size() - 1);

    std::string random_string;

    for (std::size_t i = 0; i < length; ++i)
    {
        random_string += CHARACTERS[distribution(generator)];
    }

    return random_string;
}
>>>>>>> Dev-Hardware

void Receiver(std::future<void> fut)
{
    printf("\rStarted Receiver...\n");
    while (fut.wait_for(chrono::milliseconds(1)) == std::future_status::timeout)
    {
        string recv;
        if(serial->IsAvailable())
        {
            recv = serial->convert_to_string(serial->Recv());
<<<<<<< HEAD
            printf("\r>>%s\n", recv.c_str());
=======
            if(enable_echo)
            {
                enable_echo--;
                if(enable_echo == 0)
                {
                    printf(">>Echo Disabled! - ");
                    t1 = Clock::now();
                    milliseconds ms = std::chrono::duration_cast<milliseconds>(t1 - t0);
                    std::cout << ms.count() << "ms\n";
                }
            }
            else
            {
                printf("\r>>%s\n", recv.c_str());
            }
>>>>>>> Dev-Hardware
        }
    }
    printf("Exiting Receiver!\n");
}

void Sender(vector<promise<void>> exit_promises, string filename)
{
    string input;
    ifstream file;
    file.open(filename.c_str(), ios::in);
    vector<string> commands;
    int cmd_size = 0;
    commands.push_back("");
    if(file.is_open())
    {
        while (getline(file, input))
        {
            commands.push_back(input);
            cmd_size++;
        }
        commands.push_back("");
        file.close();
    }
    else
    {
        printf("System was not able to find/open the file.\n");
        return;
    }

    int i=1;
    char data[100];
    while(i <= cmd_size)
    {
        printf("\n-------------------------------------------------\n");
        printf("Prev: %s\nCurr: %s\nFut: %s\n", commands[i-1].c_str(),commands[i].c_str(), commands[i+1].c_str());
        printf("Continue[y/CMD]:\n");
        cin.getline(data, sizeof(data));
        if (string(data) == "exit")
        {
            _mutex.lock();
            serial->Send(data);
            _mutex.unlock();
<<<<<<< HEAD
            this_thread::sleep_for(std::chrono::seconds(3));
            printf("Exiting ...\n");
            break;
        }
=======
            printf("Waiting for 5 seconds for the vehicle to shutdown!\n");
            this_thread::sleep_for(std::chrono::seconds(5));
            printf("Exiting ...\n");
            break;
        }
        else if(string(data) == "cmd echo 1")
        {
            enable_echo = 2;
            printf("Echo Enabled!");
             _mutex.lock();
            serial->Send(data);
            _mutex.unlock();
        }
        else if(enable_echo)
        {
            int pkt_size = atoi(data) * 1024;
            string packet = random_string(pkt_size);
            _mutex.lock();
            serial->Send(packet);
            t0 = Clock::now();
            //printf("Sent Time: %s", t0);
            _mutex.unlock();
        }
>>>>>>> Dev-Hardware
        else if(string(data) != "")
        {
            _mutex.lock();
            serial->Send(data);
            _mutex.unlock();
        }
        else
        {
            _mutex.lock();
            serial->Send(commands[i]);
            _mutex.unlock();
<<<<<<< HEAD
=======
            if(commands[i] == "exit")
            {
                printf("Waiting for 5 seconds for the vehicle to shutdown!\n");
                this_thread::sleep_for(std::chrono::seconds(5));
                printf("Exiting ...\n");
            }
>>>>>>> Dev-Hardware
            i++;
        }
    }
    for(int i=0;i<exit_promises.size();i++)
    {
        exit_promises[i].set_value();
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

struct monitor
{
    string command;
    int duration;
};

vector<monitor> LoadMonitoring(string filename)
{
    ifstream file;
    file.open(filename.c_str(), ios::in);
    string data;
    vector<string> datas;
    vector<monitor> results;
    if(file.is_open())
    {
        while(getline(file, data))
        {
            datas.push_back(data);
        }
    }
    for(int i=0;i<datas.size();i+=2)
    {
        results.push_back({datas[i], atoi(datas[i+1].c_str())});
    }
    file.close();
    return results;
}

int main(int argc, char *argv[])
{
    if (argc != 5)
    {
        printf("Pass Communication port name and baud rate as parameter.\n");
        printf("Pass monitoring script name and test script name too.\n");
        return -1;
    }

    serial = new Serial(argv[1], atoi(argv[2]),'\n',1000,-1);


    vector<monitor> monitoring = LoadMonitoring(argv[3]);
    // Promise to manage threads
    vector<promise<void>> exit_signals(monitoring.size()+1);
    vector<future<void>> exit_futures(monitoring.size()+1);
    for(int i=0;i<=monitoring.size();i++)
    {
        exit_futures[i] = exit_signals[i].get_future();
    }
    thread th_recv(&Receiver, move(exit_futures[0]));
    printf("Started Recevier Thread!\n");
    sleep(1);
    vector<thread> pool(monitoring.size());
    int i=0;
    for(const auto data:monitoring)
    {
        pool.push_back(thread(&HeartBeats, move(exit_futures[++i]), data.command, data.duration));
    }
    printf("Started Monitoring Threads!\n");
    thread th_send(&Sender, move(exit_signals), argv[4]);
    printf("Loaded Command script \n");
    th_send.join();
    th_recv.join();
    printf("Closed primary threads! ");
    for(int i=0;i<pool.size();i++)
    {
        if(pool[i].joinable())
        {
            pool[i].join();
        }
    }
    printf("Closed all threads safely!\n");
    return 0;
}