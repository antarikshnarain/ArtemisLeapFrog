/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Class definition for Base properties class
----------------------------------------------------------------- */

#include "BaseProperties.hpp"

using namespace std;

namespace Properties
{
    void Base::Log(string data)
    {
        cout << "Logging [INFO]: " << data << endl;
    }
}