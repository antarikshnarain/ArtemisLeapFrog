/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Library to control JetCat P300 Engine
----------------------------------------------------------------- */

namespace Actuators
{
    #include "Serialib.h"
    #include <stdio.h>
    #include <iostream>
    #include <string>
    /* \brief Class to control JetCat P300 Engine using serial communication
     * This class acts as an interface between Controller and Engine.
     */
    using namespace std;

    class JetCatP300
    {
        private:
        char *comm_port;
        int baud_rate;
        const char *format_command="%d , %s , %s %c";
        const char *format_response="%d,\"HS\",%s,%s %c";
        const char CR = 13;
        char buffer[100];
        enum
        {
            MAX_THROTTLE=50000,
            MIN_THROTTLE=100,
            MIN_BAUD_RATE=4800,
            MAX_BAUD_RATE=115000
        };
        
        struct RS232
        {
            int ADR;
            char CMDCODE[3];
            int len;
            int params[];
        };

        // \brief Creates command string
        // \param ADR : address of the ecu
        // \param CMDCODE: Command Code
        // \param PARAMLIST: list of parameters
        char *string_command(RS232 data)
        {
            
            int n = sprintf(buffer, format_command, data.ADR, data.CMDCODE, params, CR);
            return buffer;
        }

        void parse_response(char *response)
        {
            //sscanf(response, format_response, )
        }

        public:
        // Default Constructor
        JetCatP300(){}
        // Default Destructor
        ~JetCatP300();
        /* \brief Initialize engine class object
         * \param interface_port: Name of the communication port (string)
         * \param baud_rate: Rate at which the system should communicate with the engine
         */
        JetCatP300(char *interface_port, int baud_rate);
        // Start the Engine
        void Start();
        // Stop the Engine
        void Stop();
        // \brief Set Engine Thrust (percentage)
        // \param Percentage thrust (between 0-100)
        void Thrust(int percentage);
        // \brief Set Engine Thrust (throttle value)
        // \param Throttle value of the engine [MIN_THROTTLE, MAX_THROTTLE]
        void Thurst(double throttle);
    };
}