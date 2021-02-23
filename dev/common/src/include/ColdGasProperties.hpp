/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Cold Gas thrusters properties
----------------------------------------------------------------- */

#include "BaseProperties.hpp"

namespace Properties
{
#define NUM_THRUSTERS 6
    class ColdGas : public Base
    {
        friend class boost::serialization::access;

        template <class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar &durations;
        }

    public:
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
    }
};
} // namespace Properties