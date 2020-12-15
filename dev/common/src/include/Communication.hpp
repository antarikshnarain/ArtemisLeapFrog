/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: RF Communication Library for encoding and decoding data
----------------------------------------------------------------- */

#include "Serial.hpp"
#include <string>
using namespace std;

namespace Utilities
{
    class Communication: private Serial
    {
    private:
        int baud_rate;
        string channel;

    protected:
        string encoder(string);
        string decoder(string);
    public:
        //Communication(){}
        Communication(string channel, int baud_rate);
        void SendSerial(string data);
        string RecvSerial();
    };
} // namespace GroundStation