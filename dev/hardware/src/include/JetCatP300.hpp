/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Library to control JetCat P300 Engine
----------------------------------------------------------------- */

//#include "Serialib.h"
#include <stdio.h>
#include <iostream>
#include <string>
#include <string.h>
#include <vector>
#include <unistd.h>
#include "wiringPi.h"
#include "Serial.hpp"

using namespace std;
using namespace Utilities;

// Pin 8 on JetCat
#define JETCAT_SAFE 9
// Pin 9 on JetCat
#define JETCAT_POWER 10
#define JETCAT_TX 11
#define JETCAT_RX 12

/* \brief Class to control JetCat P300 Engine using serial communication
     * This class acts as an interface between Controller and Engine.
     */
namespace Actuators
{
    class JetCatP300
    {
    private:
        bool CTRL_SIG = false;
        bool PWR_SIG = false;
        float THRUST_PER = 0.0f;
        float ALTITUDE = 0.0f;
        int BAUD = 9600;
        bool ENGINE_STATE = false;

        string comm_port;
        int SerialPort;
        const char CR = 13;
        char buffer[100];

        Serial serial;
        enum
        {
            MAX_THROTTLE = 50000,
            MIN_THROTTLE = 100,
            MIN_BAUD_RATE = 4800,
            MAX_BAUD_RATE = 115000
        };

        struct RS232
        {
            int ADR;
            char CMDCODE[4];
            int len;
            string params[7];
        };

        vector<string> split(string value, string delim)
        {
            vector<string> data;
            size_t pos = 0;
            string token;
            while ((pos = value.find(delim)) != string::npos)
            {
                token = value.substr(0, pos);
                data.push_back(token);
                value.erase(0, pos + delim.length());
            }
            return data;
        }

        bool send_command(RS232 data)
        {
            string command = to_string(data.ADR) + "," + data.CMDCODE;
            if (data.len)
            {
                for (int i = 0; i < data.len; i++)
                {
                    command += "," + data.params[i];
                }
            }
            else
            {
                // Dummy parameter
                command += ",1";
            }
            command += this->CR;
            return this->serial.Send(command);
        }
        RS232 receive_response()
        {
            return this->read_response(this->serial.Recv());
        }
        RS232 read_response(string response)
        {
            RS232 data;
            if (response == "")
            {
                return data;
            }
            vector<string> values = split(response, ",");
            data.ADR = atoi(values[0].c_str());
            memcpy(data.CMDCODE, values[2].c_str(), sizeof(char) * 3);
            data.len = 0;
            for (int i = 3; i < values.size(); i++)
            {
                data.params[data.len] = values[i];
                data.len++;
            }
            return data;
        }

    public:
        // Default Constructor
        JetCatP300() {}
        // Default Destructor
        ~JetCatP300(){}
        /* \brief Initialize engine class object
         * \param interface_port: Name of the communication port (string)
         * \param baud_rate: Rate at which the system should communicate with the engine
         */
        JetCatP300(string interface_port, int baud_rate);
        // Start the Engine
        void Start();
        // Stop the Engine
        void Stop();

        // \brief Initialize the Engine Module
        void Initialize();

        // \brief Set the CTRL_SIG and PWR_SIG
        void SetEngineMode();

        // \brief Set the engine thrust percentage
        bool SetEngineThrust(float percentage);

        // \brief Check Engine Health
        bool CheckHealth();

        // \brief Get the telemetry data from the engine
        void GetTelemetry();

        // \brief Get Fuel status
        float GetFuelStatus();

        // \brief Get Engine Altitude
        float GetAltitude();

        // \brief Estimate fuel consumption
        // \return Amount of fuel remaining in percentage
        float FuelConsumption();
    };
} // namespace Actuators