/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Engine Telemetry
----------------------------------------------------------------- */

#include "BaseProperties.hpp"

namespace Properties
{
    class Engine: public Base
    {
        // Serializer for communication
        friend class boost::serialization::access;

        template <class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar &control_state;
            ar &power_state;
            ar &set_thrust;
            ar &engine_thrust;
        }

    public:
        bool control_state = false;
        bool power_state = false;
        bool set_thrust = false;
        float engine_thrust;
        
        void print()
        {
            printf("Engine Thrust: %f%%, Power: %d, Control: %d\n", engine_thrust, power_state, control_state);
        }
    };
}
