/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Class definition for Vehicle Command Controller
----------------------------------------------------------------- */

#include "CommandController.hpp"

#pragma region Protected Members
bool CommandController::parsePairs(float com_values[])
{
    char *token = strtok(this->value_string, " ");
    int index;
    do
    {
        index = this->getThrusterIndex(token);
        token = strtok(NULL, " ");
        if (token == NULL)
            return false;
        com_values[index] = atof(token);
    } while (token != NULL);
    return true;
}

int CommandController::getThrusterIndex(char *value)
{
    int index = 0;
    if (value[1] == 'r' || value[1] == 'R')
    {
        index = 0;
    }
    else if (value[1] == 'p' || value[1] == 'P')
    {
        index = 1;
    }
    else if (value[1] == 'y' || value[1] == 'Y')
    {
        index = 2;
    }
    else
    {
        printf("The type is not supported!\n");
        return -1;
    }

    return value[0] == '-' ? 2 * index + 1 : 2 * index;
}
#pragma endregion

#pragma region Public Members
bool CommandController::Call(char *cmd_string)
{
    // 1. Parse String
    sscanf(cmd_string, this->input_format, &this->code, &this->mode, this->value_string);
    auto itr = this->funcMapping.find(this->code);
    if (itr != this->funcMapping.end())
    {
        return (itr->second)();
        //return true;
    }
    else
    {
        printf("The entered code %c is invalid!\n", this->code);
        return false;
    }
}
bool CommandController::Engine()
{
    switch (this->mode)
    {
    case 0:
        this->engine.control_state = false;
        this->engine.power_state = false;
        break;
    case 1:
        this->engine.control_state = true;
        this->engine.power_state = false;
        break;
    case 2:
        this->engine.control_state = true;
        this->engine.power_state = true;
        this->engine.engine_thrust = atof(this->value_string);
        if (this->engine.engine_thrust <= 0 || this->engine.engine_thrust > 100.0)
        {
            printf("Please enter thrust value between 0-100%%\n");
            return false;
        }
        break;
    default:
        printf("Mentioned Mode %d is not supported!\n", this->mode);
        return false;
    }
    this->hashcode = (1 << 4) + this->mode;
    this->binary_oa << this->engine;
    return true;
}
bool CommandController::ACS()
{
    bool status = true;
    switch (this->mode)
    {
    case 0:
        this->coldGas.ColdGasEnabled = false;
        break;
    case 1:
        this->coldGas.ColdGasEnabled = true;
        break;
    case 2:
        // parse pairs type duration_ms
        status = this->parsePairs(this->coldGas.durations);
        break;
    default:
        break;
    }
    this->hashcode = (2 << 4) + this->mode;
    this->binary_oa << this->coldGas;
    return status;
}
bool CommandController::Gimbal()
{
    bool status = true;
    switch (this->mode)
    {
    case 0:
        this->gimbal.GimbalEnabled = false;
        break;
    case 1:
        this->gimbal.GimbalEnabled = true;
        break;
    case 2:
        // get linear position
        status = this->parsePairs(this->gimbal.Linear);
        break;
    case 3:
        // get linear position
        status = this->parsePairs(this->gimbal.Angle);
        break;
    default:
        break;
    }

    this->hashcode = (3 << 4) + this->mode;
    this->binary_oa << this->gimbal;
    return status;
}
bool CommandController::VehicleData()
{
    this->hashcode = (4 << 4) + this->mode;
    return true;
}
bool CommandController::Waypoints()
{
    char coordinate[100];
    int ctr = 0;
    this->waypoints.clear();
    for (int i = 0; i < this->mode; i++)
    {
        float ext_values[3] = {0};
        cin.getline(coordinate, 100, '\n');
        char *token = strtok(coordinate, ",");
        while (ctr < 3 && token != NULL)
        {
            ext_values[ctr] = atof(token);
            token = strtok(NULL, ",");
            ctr++;
        }
        this->waypoints.push_back(ext_values);
    }
    this->hashcode = (5 << 4) + this->mode;
    this->binary_oa << this->waypoints;
    return true;
}
bool CommandController::Hover()
{
    this->hashcode = (6 << 4) + this->mode;
    if (this->mode == 1)
    {
        this->binary_oa << atof(this->value_string);
    }
    return true;
}
bool CommandController::SpecialCommands()
{
    this->hashcode = (7 << 4) + this->mode;
    return true;
}
#pragma endregion
