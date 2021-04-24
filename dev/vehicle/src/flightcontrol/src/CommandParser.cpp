#include "flightcontrol/CommandParser.hpp"

vector<string> CommandParser::split(string data, char delim)
{
    vector<string> tokens;
    stringstream check(data);
    string temp;
    while (getline(check, temp, delim))
    {
        tokens.push_back(temp);
    }
    return tokens;
}

string CommandParser::engineParser(string cmd, string values)
{
    float value = atof(values.c_str());

    if (cmd == "ctrl")
    {
        // service
        return this->engine_ctrl(value);
    }

    else if (cmd == "power")
    { // service
        return this->engine_power(value);
    }
    else if (cmd == "enable")
    { // service
        return this->engine_enable(value);
    }
    else if (cmd == "telem")
    {
        if (value == 0)
        {
            // service x2
            return this->engine_telem_0();
        }
        else if (value == 1)
        {
            // Parameters
            return this->engine_telem_1();
        }
        else if (value == 2)
        {
            // Publisher
            return this->engine_telem_2();
        }
        else if (value == 3)
        {
            // Publisher
            return this->engine_telem_3();
        }
        else if (value == 4)
        {
            // Publisher
            return this->engine_telem_4();
        }
        else
        {
            return INVALID_COMMAND;
        }
    }
    else if (cmd == "thrust")
    {
        // Action service
        return this->engine_thrust(value);
    }
    else
        return INVALID_COMMAND;
}
string CommandParser::acsParser(string cmd, string values)
{
    if (cmd == "enable")
    {
        // software flag update
        int value = atoi(values.c_str());
        return this->acs_enable(value);
    }
    else if (cmd == "fire")
    {
        // service
        int durations[6] = {0};
        vector<string> tokens = this->split(values, ',');
        for (string token : tokens)
        {
            vector<string> thrust = this->split(token, '=');
            if (thrust.size() != 2)
            {
                return INVALID_COMMAND;
            }
            auto pos = this->cg_map.find(thrust[0]);
            if (pos == this->cg_map.end())
            {
                return INVALID_COMMAND;
            }
            else
            {
                durations[pos->second] = atoi(thrust[1].c_str());
            }
        }
        // // If successful activate the thrusters
        return this->acs_fire(durations);
    }
    else
        return INVALID_COMMAND;
}
string CommandParser::sensorsParser(string cmd, string values)
{
    int value = atoi(values.c_str());
    if (cmd == "enable")
    { // software flag update
        return this->sensor_enable(value);
    }
    else if (cmd == "telem")
    {
        if (value == 0)
        {
            // Subscriber
            return this->sensor_telem_0();
        }
        else if (value == 1)
        {
            // subscriber
            return this->sensor_telem_1();
        }
        else if (value == 2)
        {
            // subscriber
            return this->sensor_telem_2();
        }
        else
        {
            return INVALID_COMMAND;
        }
    }
    else
        return INVALID_COMMAND;
}
string CommandParser::cmdParser(string cmd, string values)
{
    int value = atoi(values.c_str());
    if (cmd == "echo")
    { // software flag logic
        return this->cmd_echo(value);
    }
    else if (cmd == "script")
    { // complex software logic
        // value -> name of script if value == stop -> cancel all script exec
        // runs as a thread
        return this->cmd_script(value);
    }
    else
        return INVALID_COMMAND;
}

string CommandParser::Parser(string cmd)
{
    vector<string> tokens = this->split(cmd, ' ');
    if (tokens.size() != 3)
    {
        return INVALID_COMMAND;
    }
    if (tokens[0] == "engine")
        return this->engineParser(tokens[1], tokens[2]);
    else if (tokens[0] == "acs")
        return this->acsParser(tokens[1], tokens[2]);
    else if (tokens[0] == "sensors")
        return this->sensorsParser(tokens[1], tokens[2]);
    else if (tokens[0] == "cmd")
        return this->cmdParser(tokens[1], tokens[2]);
    else
        return this->INVALID_COMMAND;
}

#ifdef TEST_CMDPARSER
#include <iostream>
int main()
{
    char data[100];
    CommandParser cp;
    while (1)
    {
        cin.getline(data, sizeof(data));
        if (string(data) == "exit")
        {
            break;
        }
        cout << cp.Parser(data) << endl;
    }
}
#endif