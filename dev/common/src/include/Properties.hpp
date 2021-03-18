/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Vehicle Telemetry Properties
 * Compilation: -lboost_system -lboost_serialization
----------------------------------------------------------------- */

// Standard Libs

#include <vector>
#include <sstream>

// Boost Libs
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
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
#ifndef _PROPERTIES_PROPERTIES_HPP
#define _PROPERTIES_PROPERTIES_HPP
    class Properties
    {
    public:
        ColdGas coldGas;
        Engine engine;
        Gimbal gimbal;
        PositionData positionData;
        ResourceData resourceData;
        SensorData sensorData;
    };
#endif
} // namespace Properties