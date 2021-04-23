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
#include <boost/serialization/vector.hpp>

// Custom Libs
#include "Properties.hpp"

// 0-> Flight software 1-> Ground station
#define GROUND_STATION 1

#define INTERFACES 8

using namespace std;

class CommandController : public Properties::Properties
{
private:
    stringstream ss;
    boost::archive::binary_iarchive binary_ia;
    boost::archive::binary_oarchive binary_oa;

    const char *input_format = "%c %d %s";
    char value_string[100];
    char code;
    short mode;
    float value;
    //vector<float[3]> waypoints;

    int hashcode;

protected:
    // \brief Code to function mapping
    const unordered_map<char, function<bool()>> funcMapping = 
    {
        {'e', [this]() {return this->Engine();}},
        {'a', [this]() {return this->ACS();}},
        {'g', [this]() {return this->Gimbal();}},
        {'d', [this]() {return this->VehicleData();}},
        //{'w', [this]() {return this->Waypoints();}},
        {'h', [this]() {return this->Hover();}},
        {'s', [this]() {return this->SpecialCommands();}}
    };
    const function<void()> decodeMapping[INTERFACES] =
    {
        [this]() { cout << "Message from Vehicle: " << ss.str();},
        [this]() { this->binary_ia >> this->engine;},
        [this]() { this->binary_ia >> this->coldGas;},
        [this]() { this->binary_ia >> this->gimbal;},
        [this]() { this->decodeVehicleData();},
        [this]() {},
        [this]() {},
        [this]() {}
    };

    // \brief Function used to parse pairs(type, value)
    // \param array to set the values to
    // \return status
    bool parsePairs(float[]);

    // \brief Function to convert string type to index
    // +r=0,+p=2,+y=4;
    // \param string to extract data from
    // \return index value
    int getThrusterIndex(char*);

    // \brief Decode vehicle data
    bool decodeVehicleData();

#pragma region CommandManagers
    // \brief Perform Engine specific task.
    bool Engine();

    // \brief Perform ACS specific task.
    bool ACS();

    // \brief Perform Gimbal specific task.
    bool Gimbal();

    // \brief Perform Vehicle Data specific task.
    bool VehicleData();

    // \brief Perform Waypoint specific task.
    bool Waypoints();

    // \brief Perform Hover specific task.
    bool Hover();

    // \brief Perform Special Commands task.
    bool SpecialCommands();
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
    bool DecodeCmd(char*);

    // \brief Function to read the data stream received and extract information
    // \param stream : in char pointer
    // \return status
    bool DecodeStream(char*);

    // \brief Return string stream for sending data
    // \return stream as string
    string GetData();
};