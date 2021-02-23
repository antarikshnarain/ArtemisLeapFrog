/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Vehicle Telemetry Properties
 * Compilation: -lboost_system -lboost_serialization
----------------------------------------------------------------- */

// Standard Libs

#include <vector>

// Boost Libs
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

// Custom Libs
#include "ColdGasProperties.hpp"
#include "EngineProperties.hpp"
#include "GimbalProperties.hpp"
#include "PositionDataProperties.hpp"
#include "ResourceDataProperties.hpp"
#include "SensorDataProperties.hpp"

// Pre-processors
#define DIMS 3
#define NUM_BATTERY 2
#define NUM_COLDGAS 4
#define NUM_THRUSTERS 6
#define DIMS_GIMBAL 2

namespace Properties
{
    class Properties
    {
        protected:
        ColdGas coldGas;
        Engine engine;
        Gimbal gimbal;
        PositionData positionData;
        ResourceData resourceData;
        SensorData sensorData;
    public:
        // \brief Pretty Print format for class
        // \param prop: Property Class object
        //virtual void print() = 0;

        // TODO: Add Logging functionality
    };
} // namespace Properties