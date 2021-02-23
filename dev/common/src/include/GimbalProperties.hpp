/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Gimbal Telemetry
----------------------------------------------------------------- */

#include "BaseProperties.hpp"

namespace Properties
{
    class Gimbal: public Base
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

        void print()
        {
            printf("Gimbal Position\n");
            for (int i = 0; i < DIMS_GIMBAL; i++)
            {
                printf("%f angle \t %fm position\n", Angle[i], Linear[i]);
            }
        }
    };
} // namespace Properties
