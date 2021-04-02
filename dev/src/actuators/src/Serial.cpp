/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Library to communicate using Serial Port
 * Reference  : https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
 * Note:
 * tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
 * tty.c_cflag |= PARENB;  // Set parity bit, enabling parity
----------------------------------------------------------------- */

#include "actuators/Serial.hpp"

//#define TEST_SERIAL

using namespace std;

Serial::~Serial()
{
    this->Close();
}
Serial::Serial(string portname, int baudrate)
{
    this->port_name = portname;
    this->baud_rate = baudrate;
    // printf("Initializing %s at %d\n", this->port_name.c_str(), this->baud_rate);
    this->serial_port = open(this->port_name.c_str(), O_RDWR);
    if (tcgetattr(this->serial_port, &this->tty) != 0)
    {
        printf("tcgetattr(): failed! %i %s\n", errno, strerror(errno));
        return;
    }
    this->tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
    this->tty.c_cflag |= CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
    this->tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
    this->tty.c_cflag |= CS8;            // 8 bits per byte (most common)
    this->tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
    this->tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    this->tty.c_lflag &= ~ICANON;
    this->tty.c_lflag &= ~ECHO;                                                        // Disable echo
    this->tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
    this->tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
    this->tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
    this->tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
    this->tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

    this->tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    this->tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // this->tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // this->tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    this->tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    this->tty.c_cc[VMIN] = 0;

    // Set baud rate B9600
    cfsetispeed(&tty, this->baud_rate);
    cfsetospeed(&tty, this->baud_rate);

    // Save tty settings, also checking for error
    if (tcsetattr(this->serial_port, TCSANOW, &this->tty) != 0)
    {
        printf("tcgetattr(): failed! %i %s\n", errno, strerror(errno));
        return;
    }
}

bool Serial::Send(string data)
{
    data += '\0';
    printf("Writing: %s\n", data.c_str());
    if(write(this->serial_port, data.c_str(), data.size()) < 0)
    {
        return false;
    }
    return true;
}

string Serial::Recv()
{
    string data = "";
    int n;
    char buffer[BUFFER_SIZE];
    memset(&buffer, '\0', sizeof(buffer));
    if ((n = read(this->serial_port, &buffer, sizeof(buffer))) <= 0)
    {
        return data;
    }
    data = string(buffer);
    printf("--%d %s\n",n, buffer);
    if (n == BUFFER_SIZE)
    {
        do
        {
            memset(&buffer, '\0', sizeof(buffer));
            n = read(this->serial_port, &buffer, sizeof(buffer));
            printf("--%d %s\n",n, buffer);
            data += string(buffer);
        } while (n == BUFFER_SIZE);
    }
    //printf("Returning Good data %s\n",buffer);
    return data;
}

string Serial::Recv(int num_bytes)
{
    char buffer[BUFFER_SIZE];
    memset(&buffer, '\0', sizeof(buffer));
    read(this->serial_port, &buffer, sizeof(char)*num_bytes);
    return string(buffer);
}

bool Serial::Close()
{
    if (close(this->serial_port) < 0)
    {
        return false;
    }
    return true;
}

#ifdef TEST_SERIAL
#include <iostream>
#include <unistd.h>

int main(int argv, char *argc[])
{
    if(argv != 2)
    {
        cout<< "Pass port name";
        return 1;
    }
    Utilities::Serial serial(argc[1], B9600);
    serial.Send("Test1 Send String from " + string(argc[1]));
    serial.Send("\4Test2 Sending String from " + string(argc[1]));
    sleep(2);
    string recv = serial.Recv();
    while(recv.length() == 0)
    {
        printf("Waiting %s\n", recv.c_str());
        recv = serial.Recv();
    }
    cout << argc[1] << " Received: " << recv << endl;
    return 0;
}
#endif