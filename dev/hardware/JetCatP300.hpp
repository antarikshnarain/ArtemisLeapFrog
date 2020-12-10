/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Library to control JetCat P300 Engine
 * Connection : XT60 pin -> 3s 3000mAh 22.2V
 * Pinout:
 * 1 - GND
 * 2 - Throttle PWM (PixHawk)
 * 7 - AUX PWM
 * 8 - GND (Allow engine to run, normal mode) | HIGH (Kill Sig)
 * 9 - Power on Signal for ECU (HIGH - On, LOW - Off)
 * 5 - TXD (COM1) 3.3V
 * 12- RXD (COM1) 3.3V
 * 14- TXD-cross check (COM2)
 * 6 - RXD-cross check (COM2)
----------------------------------------------------------------- */

// #include "Serialib.h"
#include "PiPins.hpp"
#include <stdio.h>
#include <string>
#include <sstream>
#include <bits/stdc++.h>
#include <unistd.h>

using namespace std;

namespace Actuators
{
    // \brief Structure to define AT command for RS232 communication port.
    struct RS232
    {
        int ADR;
        string CMDCODE;
        int len;
        double params[7];
        RS232() {}
        RS232(int adr, string cmd, int l, double argv[])
        {
            this->ADR = adr;
            CMDCODE = cmd;
            this->len = l;
            for (int i = 0; i < this->len; i++)
            {
                this->params[i] = argv[i];
            }
        }
    };

    /* \brief Class to control JetCat P300 Engine using serial communication
     * This class acts as an interface between Controller and Engine.
     */

    class JetCatP300
    {
    private:
        char *comm_port;
        int baud_rate;
        const char *format_command = "%d , %s %s %c";
        const char *format_response = "%d,\"HS\",%s,%s %c";
#ifdef __RASPBERRYPI__
        const char CR = 13;
#else
        const char CR = 0;
#endif
        char buffer[100];

        enum
        {
            MAX_THROTTLE = 50000,
            MIN_THROTTLE = 100,
            MIN_BAUD_RATE = 4800,
            MAX_BAUD_RATE = 115000
        };
        enum
        {
            B2400 = 1,
            B4800 = 2,
            B9600 = 3,
            B19200 = 4,
            B38400 = 5,
        };

        // \brief Creates command string
        // \param ADR : address of the ecu
        // \param CMDCODE: Command Code
        // \param PARAMLIST: list of parameters
        string string_request(RS232 data)
        {
            string params = "";
            if (data.len == 0)
            {
                params = ", 1";
            }
            else
            {
                for (int i = 0; i < data.len; i++)
                {
                    if ((double)(int)data.params[i] == data.params[i])
                    {
                        params += ", " + to_string((int)data.params[i]);
                    }
                    else
                    {
                        params += ", " + to_string(data.params[i]);
                    }
                }
            }
            //buffer = to_string(data.ADR) + data.CMDCODE + params + to_string(CR);
            int n = sprintf(buffer, format_command, data.ADR, data.CMDCODE.c_str(), params.c_str(), CR);
            printf("Sending Command: %s\n", buffer);
            //cout << buffer << endl;
            return buffer;
        }

        RS232 parse_response(char *response)
        {
            RS232 data;
            string s;
            istringstream str(response);
            getline(str, s, ',');
            data.ADR = stoi(s);
            getline(str, s, ',');
            getline(str, s, ',');
            data.CMDCODE = s;
            data.len = 0;
            while (getline(str, s, ','))
            {
                data.params[data.len] = stoi(s);
                data.len++;
            }

            // Check the return Code
            if (data.CMDCODE == "OK")
            {
                return data;
            }

            // Check for other return codes.
            data.ADR = -1;
            // switch (data.CMDCODE)
            // {
            // case "UC":
            //     printf("Unknown Command!\n");
            //     break;
            // case "PA":
            //     printf("Wrong parameter number!\n");
            //     break;
            // case "NA":
            //     printf("Command not allowed in actual operation mode!\n");
            //     break;
            // case "PR":
            //     printf("At least one parameter is out of range!\n");
            //     break;
            // case "PL":
            //     printf("At least one parameter is too long!\n");
            //     break;
            // case "DF":
            //     printf("Unknown data format!\n");
            //     break;
            // default:
            //     printf("Unkown Error Code!\n");
            //     break;
            // }
            return data;
        }

    public:
        // Default Constructor
        JetCatP300() {}
        // Default Destructor
        ~JetCatP300() {}
        /* \brief Initialize engine class object
         * \param interface_port: Name of the communication port (string)
         * \param baud_rate: Rate at which the system should communicate with the engine
         */
        JetCatP300(char *interface_port, int baud_rate);
        // \brief send command via serial port, dummy parameter automatically added
        // \param adr: slave address
        // \param adr: command code
        // \param adr: number of arguments
        // \param adr: array of params
        // \return response from the engine.
        RS232 SendCommand(int adr, string cmdcode, int l, double argv[] = NULL);
        // \brief Start the Engine
        void Start();
        // \brief Stop the Engine
        void Stop();
        // // \brief Set Engine Thrust (percentage)
        // // \param Percentage thrust (between 0-100)
        // void Thrust(int percentage);
        // \brief Set Engine Thrust (throttle value)
        // \param Throttle value of the engine [MIN_THROTTLE, MAX_THROTTLE]
        void Thrust(double throttle);
        // \brief Get Standard Telemetry data
        // \param verbose -> if we want the detailed data with timestamp
        string GetTelemetry(bool verbose = false);
        // \brief Check Health of the Engine
        void CheckHealth();
    };

} // namespace Actuators