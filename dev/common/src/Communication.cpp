/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Class definition for communication module for encoding and decoding data
----------------------------------------------------------------- */

#include "Communication.hpp"

using namespace std;

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
    }
}