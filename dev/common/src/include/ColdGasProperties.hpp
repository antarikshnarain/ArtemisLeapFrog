/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Engine Telemetry
 * Compilation: -lboost_system -lboost_serialization
----------------------------------------------------------------- */

#include "Properties.hpp"

namespace Properties
{
    class ColdGas
    {
        friend class boost::serialization::access;

        template <class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar &durations;
        }

    public:
        float durations[NUM_THRUSTERS];
    };
} // namespace Properties