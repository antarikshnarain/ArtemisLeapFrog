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
    class Engine
    {
        // Serializer for communication
        friend class boost::serialization::access;

        template <class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar &control_state;
            ar &power_state;
            ar &engine_thrust;
        }

    public:
        bool control_state = false;
        bool power_state = false;
        float engine_thrust;
    };
}
