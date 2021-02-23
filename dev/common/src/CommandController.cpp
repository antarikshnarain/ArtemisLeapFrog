/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Library containing definition of Command Controller class
----------------------------------------------------------------- */

#include "CommandController.hpp"

bool CommandController::Call(int cmd_code)
{
    auto itr = functionMapping.find(cmd_code);
    if (itr != functionMapping.end())
    {
        (itr->second)();
        return true;
    }
    else
    {
        return false;
    }
}

bool CommandController::Call(char *cmd_string)
{
    // 1. Parse String
    char *token = strtok(cmd_string, " ");
    auto itr = keyMapping.find(string(token));
    this->points.clear();
    this->waypoints.clear();
    if (itr != keyMapping.end())
    {
        this->hash_code = itr->second.value;
        this->binary_oa << this->hash_code;
        if (token == "sw")
        {
            int count = atoi(strtok(NULL, " "));
            float way[3];
            for (int i = 0; i < count; i++)
            {
                cin >> way[0] >> way[1] >> way[2];
                waypoints.push_back(way);
            }
            // Send waypoints
        }
        else
        {
            // Fetch values
            token = strtok(NULL, " ");
            while (token != NULL)
            {
                points.push_back(atof(token));
                token = strtok(NULL, " ");
            }
        }
        // 2. Generate Hash + Properties object
        // Send Data
        return this->Call(this->hash_code);
    }
    else
    {
        cout << "Invalid Command!\n";
    }
    return false;

    // 1. Deserialize data
    this->binary_ia >> this->hash_code;
    // 2. Separate Hash Code and Properties
    auto itr = this->functionMapping.find(this->hash_code);
    if (itr != this->functionMapping.end())
    {
        // 3. Call associated Hash function
        (itr->second)();
        return true;
    }
}

#pragma region Manual Controls
bool CommandController::Engine()
{
    if (this->points.size() == 2)
    {
        int val = this->points[0];
        this->engine.control_state = val & 0x1;
        this->engine.power_state = val & 0x2;
        this->engine.set_thrust = val & 0x4;
        if (this->engine.set_thrust)
        {
            this->engine.engine_thrust = this->points[1];
        }
        this->binary_oa << this->engine;
        return true;
    }
    return false;

    this->binary_ia >> this->engine;
    this->engine.print();
}
bool CommandController::ACS()
{

    if (this->points.size() == NUM_THRUSTERS)
    {
        for (int i = 0; i < NUM_THRUSTERS; i++)
        {
            this->coldGas.durations[i] = this->points[i];
        }
        this->binary_oa << this->coldGas;
        return true;
    }
    return false;

    this->binary_ia >> this->coldGas;
    this->coldGas.print();
}
bool CommandController::Gimbal()
{

    if (this->points.size() == DIMS_GIMBAL)
    {
        for (int i = 0; i < DIMS_GIMBAL; i++)
        {
            this->gimbal.Angle[i] = this->points[i];
        }
        this->binary_oa << this->gimbal;
        return true;
    }
    return false;

    this->binary_ia >> this->gimbal;
    this->gimbal.print();
}
bool CommandController::SensorData()
{

    // Its a read operation

    this->binary_ia >> this->sensorData;
    this->sensorData.print();
}
bool CommandController::PositionData()
{
    // Its a read operation
    this->binary_ia >> this->positionData;
    this->positionData.print();
}
bool CommandController::ResourceData()
{

    // Its a read operation

    this->binary_ia >> this->resourceData;
    this->resourceData.print();
}
bool CommandController::ActuatorData()
{
}
#pragma endregion

#pragma region Flight Controls
bool CommandController::InitializeSystem()
{

    // Only the hash code is needed.
}
bool CommandController::SendWaypoints()
{

    this->binary_oa << this->waypoints;
}
bool CommandController::Hover()
{

    if (this->points.size() == 1)
    {
        this->binary_oa << this->points return true;
    }
    return false;
}
bool CommandController::InitializeLanding()
{

    // Only the hash code is needed.
}
bool CommandController::AbortCommand()
{

    // Only the hash code is needed.
}
bool CommandController::ShutdownSystem()
{

    // Only the hash code is needed.
}
#pragma endregion
