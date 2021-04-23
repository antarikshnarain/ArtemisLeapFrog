/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: RF Communication Library for encoding and decoding data
----------------------------------------------------------------- */

#include "Serial.hpp"
#include <string>
#include <sstream>
using namespace std;

namespace Utilities
{
    class Communication : private Serial
    {
    private:
        const int ROT13 = 13;
        const int ROT7 = 7;
        int baud_rate;
        string channel;

    protected:
        // \brief Encode the data
        // \param string - data string
        // \return encoded string
        string encoder(string);
        // \brief Decode the data
        // \param string - data string
        // \return encoded string
        string decoder(string);

    public:
        Communication(){}
        // \brief Initialize communication channel
        // \param channel the channel name e.g. /dev/pts/1
        // \param baud_rate the channel baud rate
        Communication(string, int);
        // \brief Send data on the Serial port
        // \param data to be sent
        void SendSerial(string);
        // \brief Receive data from Serial port
        // \return data string
        string RecvSerial();
    };
} // namespace Utilities