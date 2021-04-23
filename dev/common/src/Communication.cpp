/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Class definition for communication module for encoding and decoding data
----------------------------------------------------------------- */

#include "Communication.hpp"

using namespace std;

//#define TEST_COMMUNICATION

namespace Utilities
{
    Communication::Communication(string channel, int baud_rate):Serial(channel,baud_rate)
    {
        this->channel = channel;
        this->baud_rate = baud_rate;
        printf("Init Comm on %s at %d\n", this->channel.c_str(), this->baud_rate);
    }
    string Communication::encoder(string data)
    {
        // TODO: Add data encoding algorithm
        return data;
    }
    string Communication::decoder(string data)
    {
        // TODO: Add data decoding algorithm
        return data;
    }
    void Communication::SendSerial(string data)
    {
        if(this->Send(this->encoder(data)))
        {
            printf("%s Send: %s\n", this->channel.c_str(), data.c_str());
        }
    }
    string Communication::RecvSerial()
    {
        return this->decoder(this->Recv());
        //return this->decoder(this->Recv());
    }
}

#ifdef TEST_COMMUNICATION
#include <iostream>
#include <unistd.h>

int main(int argv, char *argc[])
{
    if(argv != 2)
    {
        cout<< "Pass port name";
        return 1;
    }
    Utilities::Communication comm(argc[1], B9600);
    comm.SendSerial("Test1 Send String from " + string(argc[1]));
    comm.SendSerial("\4Test2 Sending String from " + string(argc[1]));
    sleep(2);
    string recv = comm.RecvSerial();
    while(recv.length() == 0)
    {
        printf("Waiting %s\n", recv.c_str());
        recv = comm.RecvSerial();
    }
    cout << argc[1] << " Received: " << recv << endl;
    return 0;
}
#endif