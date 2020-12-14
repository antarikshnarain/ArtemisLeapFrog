/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: RF Communication Library
----------------------------------------------------------------- */

#include <string>
using namespace std;

class Communication
{
    private:
    int baud_rate;
    int channel;
    public:
    void Send(string data);
    string Recv();
};