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
int enable_echo = 0;
typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::milliseconds milliseconds;
Clock::time_point t0;
Clock::time_point t1;

// Boost Libs
#define BOOST_LOG_DYN_LINK 1
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

namespace logging = boost::log;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;
namespace keywords = boost::log::keywords;
namespace severity = boost::log::trivial;

void initialize()
{
    //logging::add_file_log("sample.log");
    logging::add_file_log(
        keywords::file_name = "sample_%N.log",
        keywords::rotation_size = 10 * 1024 * 1024, // 10 MB
        keywords::time_based_rotation = sinks::file::rotation_at_time_point(0, 0, 0),
        keywords::format = "%LineID% [%TimeStamp%]: %Message%");

    logging::core::get()->set_filter(
        logging::trivial::severity >= logging::trivial::info);
}

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

void Receiver(std::future<void> fut)
{
    printf("\rStarted Receiver...\n");
    while (fut.wait_for(chrono::milliseconds(1)) == std::future_status::timeout)
    {
        string recv;
        if(serial->IsAvailable())
        {
            recv = serial->convert_to_string(serial->Recv());
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
        //int num = atoi(data);
        if (string(data) == "exit")
        {
            printf("Exiting ...\n");
            _mutex.lock();
            printf("Sending exit command to other node.\n");
            serial->Send("exit");
            _mutex.unlock();
            printf("Waiting for 5 seconds for the vehicle to shutdown!\n");
            sleep(5);
            exit_promise.set_value();
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
    initialize();
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