/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG 
 * Author     : Antariksh Narain
 * Description: Header file defining all the connections. Property file
----------------------------------------------------------------- */

#define TEST

#pragma region JetCatP300
// Pin 8 on JetCat
#define JETCAT_SAFE 9
// Pin 9 on JetCat
#define JETCAT_POWER 10
#define JETCAT_TX 11
#define JETCAT_RX 12
#define JETCAT_COMM "/dev/ACM0"
#pragma endregion

#define MODE_LA 0
#define COUNT_ACS 6

#pragma region StandardHeaders
#include <stdio.h>
#include <iostream>
#include <string>
#include <string.h>
#include <vector>
#include <unistd.h>

using namespace std;
#pragma endregion
