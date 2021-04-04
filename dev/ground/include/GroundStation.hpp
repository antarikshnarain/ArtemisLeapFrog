/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Library containing functionality of Ground Station
----------------------------------------------------------------- */

#include <string>
#include "Serial.hpp"

class GroundStation : public Serial
{
    private:
    // \brief send data on serial port
    bool request(string data);
    // \brief receive data from serial port
    string response();
    public:
    // \brief initializes a terminal interface for user interaction
    void Terminal();
    // \brief display responses from the serial port
    void Display();
}