/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Library to communicate using Serial Port
----------------------------------------------------------------- */
// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

#include <string>
using namespace std;

namespace Utilities
{
    class Serial
    {
    private:
        string port_name;
        int baud_rate;
        int serial_port;
        termios tty;

    public:
        Serial(){};
        // \brief Destructor to close port
        ~Serial();
        // \brief Create a serial connection
        Serial(string portname, int baudrate);
        // \brief Send data on the port
        bool Send(string data);
        // \brief Read data on the port
        string Recv();
        // \brief Release port
        bool Close();
    };
} // namespace Utilities