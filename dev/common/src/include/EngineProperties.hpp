/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Engine Telemetry
----------------------------------------------------------------- */

#include "BaseProperties.hpp"

namespace Properties
{
#ifndef _PROPERTIES_ENGINE_HPP
#define _PROPERTIES_ENGINE_HPP
    class Engine : public Base
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

    private:
        enum UpdatedObjects
        {
            CTRL_SIG = 0,
            PWR_SIG = 1,
            THRUST = 2
        };

    public:
        bool control_state = false;
        bool power_state = false;
        float engine_thrust;

        void print()
        {
            printf("Engine Thrust: %f%%, Power: %d, Control: %d\n", engine_thrust, power_state, control_state);
        }
    };
#endif
}
