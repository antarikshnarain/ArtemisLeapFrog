#include "Serial.hpp"

Serial::~Serial()
{
    this->closeSerial();
}

Serial::Serial(string portname, int baudrate, char delim='\n', int q_size=1000, int pkt_size=-1)
{
    this->port_name = portname;
    this->baud_rate = baudrate;
    this->delimitter = delim;
    this->queue_size = q_size;
    this->num_bytes = pkt_size;
    this->exit_future = this->exit_signal.get_future();
    if(!this->initializePort())
    {
        return;
    }
    // Initialize thread to monitor port
    this->monitor_thread = new thread(&Serial::manageQueue, this, std::move(this->exit_future));
}

bool Serial::initializePort()
{
    this->serial_port = open(this->port_name.c_str(), O_RDWR);

    if (tcgetattr(this->serial_port, &this->tty) != 0)
    {
        printf("tcgetattr(): failed! %i %s\n", errno, strerror(errno));
        return false;
    }
    tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
    tty.c_cflag |= CS8;            // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;                                                        // Disable echo
    tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
    tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
    tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set baud rate B9600
    cfsetispeed(&tty, this->baud_rate);
    cfsetospeed(&tty, this->baud_rate);

    // Save tty settings, also checking for error
    if (tcsetattr(this->serial_port, TCSANOW, &this->tty) != 0)
    {
        printf("tcgetattr(): failed! %i %s\n", errno, strerror(errno));
        return false;
    }
    return true;
}

void Serial::manageQueue(std::future<void> _future)
{
    char c;
    string data = "";
    int size = read(this->serial_port, &c, 1);
    while(c != EOT)
    {
        if(size == 0)
        {
            // buffer is empty sleep for a while
            if(_future.wait_for(chrono::microseconds(1)) != std::future_status::timeout)
            {
                break;
            }
            #ifdef TEST_SERIAL
            printf("Sleeping for a while!\n");
            #endif
            std::this_thread::sleep_for(chrono::milliseconds(1000));
        }
        else if((this->num_bytes == -1 && c == this->delimitter) 
            || (this->num_bytes == (int)data.size()))
        {
            this->_mutex.lock();
            this->recv_data.push(data);
            if(this->recv_data.size() == this->queue_size)
            {
                this->recv_data.pop();
            }
            this->_mutex.unlock();
            data = "";
        }
        else
        {
            data += c;
        }
        size = read(this->serial_port, &c, 1);
    }
    #ifdef TEST_SERIAL
    printf("Closing Queue Manager for %s\n", this->port_name.c_str());
    #endif
}

bool Serial::closeSerial()
{
    this->monitor_thread->join();
    this->exit_signal.set_value();
    if (close(this->serial_port) < 0)
    {
        return false;
    }
    return true;
}

bool Serial::Send(string data)
{
    data += this->delimitter;
    #ifdef TEST_SERIAL
    printf("Writing: %s\n", data.c_str());
    #endif
    if (write(this->serial_port, data.c_str(), data.size()) < 0)
    {
        return false;
    }
    return true;
}

string Serial::Recv()
{
    string data = "";
    this->_mutex.lock();
    data = this->recv_data.front();
    this->recv_data.pop();
    this->_mutex.unlock();
    #ifdef TEST_SERIAL
    printf("Received: %s\n", data.c_str());
    #endif
    return data;
}

bool Serial::IsAvailable()
{
    this->_mutex.lock();
    bool available = this->recv_data.size() > 0;
    this->_mutex.unlock();
    return available;
}

#ifdef TEST_SERIAL
int main(int argv, char *argc[])
{
    if (argv != 2)
    {
        cout << "Pass port name";
        return 1;
    }
    Serial serial(argc[1], 9600, '\n');
    //serial.Send("Test1 Send String from " + string(argc[1]));
    sleep(2);
    while (1)
    {
        if(serial.IsAvailable())
        {
            string recv = serial.Recv();
            cout << argc[1] << " Received: " << recv << endl;
        }
        sleep(1);
    }
    return 0;
}
#endif