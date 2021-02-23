/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Command translation
----------------------------------------------------------------- */

#include <iostream>
#include <functional>
#include <vector>
#include <unordered_map>
#include <string>
#include <stdarg.h>
#include <stdio.h>
#include <sstream>


// Boost Libs
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

// Custom Libs
#include "Properties.hpp"

// 0-> Flight software 1-> Ground station
#define GROUND_STATION 1

using namespace std;

class CommandController : public Properties::Properties
{
private:
    stringstream ss;
    boost::archive::binary_iarchive binary_ia;
    boost::archive::binary_oarchive binary_oa;
    short hash_code;
    string byte_stream;
    vector<float> points;
    vector<float[3]> waypoints;

    struct keyProp
    {
        short value;
        string example_string;
    };

    // Properties Class Objects
    
protected:
    const unordered_map<std::string, keyProp> keyMapping = {
        {"e", {1, "Cmd Engine: e <thrust_value>"}},
        {"a", {2, "Cmd ACS: a <thruster> <duration_ms>"}},
        {"g", {3, "Cmd Gimbal: g <gimbal> <angle>"}},
        {"sd", {4, "Request sensor data"}},
        {"pd", {5, "Request position data"}},
        {"rd", {6, "Request resource data"}},
        {"ad", {7, "Request actuator data"}},
        {"is", {8, "Initialize System"}},
        {"sw", {9, "Send Waypoints: sw <num_points>"}},
        //Add command to reset waypoints
        {"h", {10, "Hover: h <altitude>"}},
        {"il", {11, "Initialize Landing"}},
        {"aq", {12, "Abort Command"}},
        {"ss", {13, "Shutdown System"}}};

    const unordered_map<int, function<void()>> functionMapping = {
        {1, [this]() { this->Engine(); }},
        {2, [this]() { this->ACS(); }},
        {3, [this]() { this->Gimbal(); }},
        {4, [this]() { this->SensorData(); }},
        {5, [this]() { this->PositionData(); }},
        {6, [this]() { this->ResourceData(); }},
        {7, [this]() { this->ActuatorData(); }},
        {8, [this]() { this->InitializeSystem(); }},
        {9, [this]() { this->SendWaypoints(); }},
        {10, [this]() { this->Hover(); }},
        {11, [this]() { this->InitializeLanding(); }},
        {12, [this]() { this->AbortCommand(); }},
        {13, [this]() { this->ShutdownSystem(); }}};

    void encodeData()
    {
    }
    void decodeData()
    {
    }
#pragma region Manual Controls
    bool Engine();
    bool ACS();
    bool Gimbal();
    bool SensorData();
    bool PositionData();
    bool ResourceData();
    bool ActuatorData();
#pragma endregion

#pragma region Flight Controls
    bool InitializeSystem();
    bool SendWaypoints();
    bool Hover();
    bool InitializeLanding();
    bool AbortCommand();
    bool ShutdownSystem();
#pragma endregion

public:
    // \brief Default Constructor which also initializes binary input/output stream.
    CommandController():binary_ia(ss),binary_oa(ss){};
    // \brief Function to call the associated function
    // \param cmd_code: code associated to the function
    bool Call(int);

    // \brief Function to extract data from string and call associated function
    // \param cmd_string: data string
    // \return valid command or not
    bool Call(char*);

    // \brief Return string stream
    string GetData()
    {
        return ss.str();
    }
};