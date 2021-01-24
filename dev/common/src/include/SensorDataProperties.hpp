/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Sensor Data collection
 * Compilation: -lboost_system -lboost_serialization
----------------------------------------------------------------- */

#include "Properties.hpp"

namespace Properties
{
    class SensorDataProperties : public Properties
    {
        friend class boost::serialization::access;

        template <class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar &Base_Principal_axes;
            ar &Altitude;
        }

    public:
        float Base_Principal_axes[DIMS];
        float Altitude;

        void print()
        {
            printf("Orientation : %f \t %f \t %f\n", Base_Principal_axes[0], Base_Principal_axes[1], Base_Principal_axes[2]);
            printf("Altitude    : %f\n", Altitude);
        }
    };
} // namespace Properties