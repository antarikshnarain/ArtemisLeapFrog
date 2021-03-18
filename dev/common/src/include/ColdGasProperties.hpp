/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Cold Gas thrusters properties
----------------------------------------------------------------- */

#include "BaseProperties.hpp"

namespace Properties
{
#ifndef _PROPERTIES_COLDGAS_HPP
#define _PROPERTIES_COLDGAS_HPP
    class ColdGas : public Base
    {
        friend class boost::serialization::access;

        template <class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar &ColdGasEnabled;
            ar &durations;
        }

    public:
        bool ColdGasEnabled = false;
        float durations[NUM_THRUSTERS];

        void print()
        {
            printf("Thruster Activity\n");
            for (int i = 0; i < NUM_THRUSTERS; i++)
            {
                printf("%f, ", durations[i]);
            }
            printf("\n");
        }
    };
#endif
} // namespace Properties