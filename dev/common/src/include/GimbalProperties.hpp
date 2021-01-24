/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Gimbal Telemetry
 * Compilation: -lboost_system -lboost_serialization
----------------------------------------------------------------- */

#include "Properties.hpp"

namespace Properties
{
    class Gimbal
    {
        friend class boost::serialization::access;

        template <class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar &GimbalEnabled;
            ar &Angle;
            ar &Linear;
        }

    public:
        bool GimbalEnabled;
        float Angle[DIMS_GIMBAL];
        float Linear[DIMS_GIMBAL];
    };
} // namespace Properties
