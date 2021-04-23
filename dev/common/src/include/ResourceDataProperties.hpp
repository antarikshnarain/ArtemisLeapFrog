/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Onboard vehicle resource status
----------------------------------------------------------------- */

#include "BaseProperties.hpp"

namespace Properties
{
#ifndef _PROPERTIES_RESOURCEDATA_HPP
#define _PROPERTIES_RESOURCEDATA_HPP
    class ResourceData : public Base
    {
        friend class boost::serialization::access;

        template <class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar &Battery;
            ar &ColdGas;
            ar &Fuel;
        }

    public:
        /*
    The values stored are in percentage
    */
        float Battery[NUM_BATTERY];
        float ColdGas[NUM_COLDGAS];
        float Fuel;

        void print()
        {
            // Resource Status
            printf("Gas         : %f%%\n", Fuel);
            printf("Battery     : ");
            for (float a : Battery)
            {
                printf("%fV, ", a);
            }
            printf("Tanks P     : ");
            for (float a : ColdGas)
            {
                printf("%f Psi, ", a);
            }
            printf("\n");
        }
    };
#endif
} // namespace Properties