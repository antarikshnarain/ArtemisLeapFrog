// Standard headers
#include <string.h>
#include <string>
#include <iostream>
#include <mutex>
#include <thread>
#include <chrono>
#include <future>
#include <functional>
#include <queue>

// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

using namespace std;

#ifndef _SERIAL_h_
#define _SERIAL_h_

#define BUFFER_SIZE 256
#define EOT 4 // End of transmission

/*
Note: The pkt size is a separate function because the byte value may be equal to
*/

class Serial
{
private:
    // Serial Properties
    string port_name;
    int baud_rate;
    char delimitter;
    int num_bytes;
    long unsigned int queue_size;

    int serial_port;
    termios tty;

    queue<string> recv_data;

    thread *monitor_thread;
    mutex _mutex;
    std::promise<void> exit_signal;
    std::future<void> exit_future;

protected:

    bool initializePort();
    // \brief Function to maintain the queue of data
    void manageQueue(std::future<void>);

    // \brief Release port
    bool closeSerial();

public:
    // \brief Destructor to close port
    ~Serial();
    // \brief Create a serial connection
    // \param portname name of the port to connect
    // \param baudrate rate at which to communicate
    // \param serial communication delimitter
    // \param queue size of data
    // \param packet size
    Serial(string, int, char, int, int);

    // \brief Send data on the port
    // \param data to be send
    bool Send(string);
    
    // \brief Read data on the port
    // \return data string
    string Recv();

    // \brief Check if values exist in the buffer queue
    bool IsAvailable();

};

#endif