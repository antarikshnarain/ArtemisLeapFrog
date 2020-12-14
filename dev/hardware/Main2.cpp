//# include "include/JetCatP300.hpp"
#include "include/JetCatP300.hpp"
#include <iostream>
#include <string>

using namespace std;
using namespace Actuators;

int main()
{
    //char port[] = "ddd";
    string port = "ddd";
    JetCatP300 jcp(port.c_str(), B9600);
    jcp.Initialize();
    jcp.CheckHealth();
}