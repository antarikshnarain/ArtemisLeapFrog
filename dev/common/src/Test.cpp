#include "Communication.hpp"
#include <iostream>
using namespace std;
using namespace Utilities;

int main(int argv, char *argc[])
{
    if(argv != 2)
    {
        cout<< "Pass port name";
        return 1;
    }
    cout<<argc[1];
    #ifdef TEST
    Communication comm(argc[1], B9600);
    #else
    Communication comm(argc[1],B9600);    
    #endif
    comm.SendSerial("Test Send String from " + string(argc[1]));
    string recv = comm.RecvSerial();
    while(recv.length() == 0)
    {
        printf("Waiting %s\n", recv.c_str());
        recv = comm.RecvSerial();
    }
    cout << "Received: " << recv;
    return 0;
}