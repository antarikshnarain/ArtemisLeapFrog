/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Main Code
----------------------------------------------------------------- */

//#include "JetCatP300.hpp"
//#include "src/JetCatP300.cpp"

using namespace Actuators;

int main()
{
    JetCatP300 jcp;
    // Test Commands
    printf("A1\n");
    jcp.Start();
    printf("A1\n");
    jcp.CheckHealth();
    printf("A1\n");
    jcp.GetTelemetry();
    printf("A1\n");
    jcp.Thrust(1.333);
    printf("A1\n");
    jcp.Stop();
    return 0;
}