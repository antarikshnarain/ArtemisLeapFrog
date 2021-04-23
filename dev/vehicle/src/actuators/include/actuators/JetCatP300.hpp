/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Library to control JetCat P300 Engine
----------------------------------------------------------------- */

#include <Serial.hpp>
#include <wiringPi.h>
#include <vector>
#include <chrono>
#include <mutex>
#include <string>
#include <sstream>

using namespace std;

#define JETCAT_SAFE 25  // pin 22
#define JETCAT_POWER 11 // pin 23

struct RS232
{
    int ADR;
    string CMDCODE;
    int len;
    string params[7];
};

/* \brief Class to control JetCat P300 Engine using serial communication
    * This class acts as an interface between Controller and Engine.
    */
class JetCatP300 : public Serial
{
private:
    const char CR = 13;
    char buffer[100];
    mutex _mutex;
    enum
    {
        MAX_THROTTLE = 50000,
        MIN_THROTTLE = 100,
        MIN_BAUD_RATE = 4800,
        MAX_BAUD_RATE = 115000
    };

protected:
    // \brief split the csv data
    // \param string to split
    // \param delimitter
    // \return vector of values
    vector<string> split(string, char);

    // \brief process the response to RS232 structure
    // \param response string
    // \return RS232 structure object
    RS232 read_response(string);

public:
    // Default Destructor
    ~JetCatP300() {}
    // \brief Initialize engine class object
    // \param interface_port: Name of the communication port (string)
    // \param baud_rate: Rate at which the system should communicate with the engine
    JetCatP300(string interface_port, int baud_rate);

    // \brief Send command over serial port
    // \param command in RS232 format
    // \return status of command
    bool send_command(RS232);

    // \brief Receive response from the engine.
    // \return response in RS232 structure.
    RS232 receive_response();

    // \brief send and receive data in a single command
    // \param command in RS232 structure
    // \return response in RS232 structure
    RS232 execute(RS232);
};