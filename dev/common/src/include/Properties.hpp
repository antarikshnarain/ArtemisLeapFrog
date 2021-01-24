/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Vehicle Telemetry Properties
 * Compilation: -lboost_system -lboost_serialization
----------------------------------------------------------------- */

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

#include <vector>

#define DIMS 3
#define NUM_BATTERY 2
#define NUM_COLDGAS 4
#define NUM_THRUSTERS 6
#define DIMS_GIMBAL 2

namespace Properties
{
    class Properties
    {
    public:
        // \brief Pretty Print format for class
        // \param prop: Property Class object
        virtual void print() = 0;

        // TODO: Add Logging functionality
    };
} // namespace Properties