/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Library to communicate using Serial Port
----------------------------------------------------------------- */

//#include "Connections.hpp"

#include <string.h>
#include <string>
#include <iostream>
using namespace std;

// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

#ifndef _SERIAL_h_
#define _SERIAL_h_

#define BUFFER_SIZE 8
class Serial
{
private:
    string port_name;
    int baud_rate;
    int serial_port;
    termios tty;

public:
    Serial(){}
    // \brief Destructor to close port
    ~Serial();
    // \brief Create a serial connection
    // \param portname name of the port to connect
    // \param baudrate rate at which to communicate
    Serial(string, int);
    // \brief Send data on the port
    // \param data to be send
    bool Send(string);
    // \brief Read data on the port
    // \return data string
    string Recv();
    // \brief Read predetermined set of bytes
    // \return string array
    string Recv(int);
    // \brief Release port
    bool Close();
};

#endif