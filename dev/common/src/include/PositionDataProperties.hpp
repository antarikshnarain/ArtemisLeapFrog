/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Vehicle Position Data
----------------------------------------------------------------- */

#include "BaseProperties.hpp"

namespace Properties
{
#ifndef _PROPERTIES_POSITIONDATA_HPP
#define _PROPERTIES_POSITIONDATA_HPP
    class PositionData : public Base
    {
        friend class boost::serialization::access;

        template <class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar &Position;
            ar &Velocity;
            ar &Acceleration;
        }

    public:
        /*
    Sequence: X, Y, Z
    */
        float Position[DIMS];
        float Velocity[DIMS];
        float Acceleration[DIMS];

        void print()
        {
            printf("Position \t Velocity \t Accel\n");
            for (int i = 0; i < DIMS; i++)
            {
                printf("%fm \t %fm/s \t %fm/s.s\n", Position[i], Velocity[i], Acceleration[i]);
            }
        }
    };
#endif
} // namespace Properties