#ifndef _CMDPARSER_h_
#define _CMDPARSER_h_

#include <string>
#include <map>
#include <vector>
#include <sstream>

using namespace std;

class CommandParser
{
private:
    const string INVALID_COMMAND = "Invalid Command.";
    map<string, int> cg_map = {{"r", 0}, {"R", 1}, {"p", 2}, {"P", 3}, {"y", 4}, {"Y", 5}};

    // \brief Split string by the delimitter
    // \param string data
    // \param character delimitter
    // \return vector of split strings
    vector<string> split(string data, char delim);

protected:
    // \brief Command Parser for Engine
    // \param command to execute
    // \param additional params
    // \return status
    string engineParser(string cmd, string values);

    // \brief Command Parser for ACS
    // \param command to execute
    // \param additional params
    // \return status
    string acsParser(string cmd, string values);

    // \brief Command Parser for Sensors
    // \param command to execute
    // \param additional params
    // \return status
    string sensorsParser(string cmd, string values);

    // \brief Command Parser for Miscellaneous commands
    // \param command to execute
    // \param additional params
    // \return status
    string cmdParser(string cmd, string values);

    // \brief Command Parser for Gimbal
    // \param command to execute
    // \param additional params
    // \return status
    string gimbalParser(string cmd, string values);

public:
    // \brief Primary Command Parser.
    // \param command to execute
    // \return status
    string Parser(string cmd);

    virtual string engine_ctrl(int value)
    {
        return value == 1 ? "Control enabled" : "Control disabled";
    }
    virtual string engine_power(int value)
    {
        return value == 1 ? "Power enabled" : "Power disabled";
    }
    virtual string engine_enable(int value)
    {
        return value == 1 ? "Engine started" : "Engine stopped";
    }
    virtual string engine_telem_0()
    {
        return "Telemetry of Health Check";
    }
    virtual string engine_telem_1()
    {
        return "Telemetry of System Info";
    }
    virtual string engine_telem_2()
    {
        return "Engine Telemetry";
    }
     virtual string engine_telem_3()
    {
        return "Fuel Telemetry";
    }
    virtual string engine_thrust(float value)
    {
        return "Setting thrust to " + to_string(value);
    }

    virtual string acs_enable(int value)
    {
        return value == 1 ? "acs enabled" : "acs disabled";
    }
    virtual string acs_fire(int durations[6])
    {
        char temp[] = {'r', 'R', 'p', 'P', 'y', 'Y'};
        string send_string = "";
        for (int i = 0; i < 6; i++)
        {
            send_string += "Activating " + to_string(temp[i]) + " for " + to_string(durations[i]) + '\n';
        }
        return send_string;
    }

    virtual string sensor_enable(int value)
    {
        return value == 1 ? "Sensors started" : "Sensors stopped";
    }
    virtual string sensor_telem_0()
    {
        return "Telemetry of IMU";
    }
    virtual string sensor_telem_1()
    {
        return "Telemetry of Laser";
    }
    virtual string sensor_telem_2()
    {
        return "Battery Telemetry";
    }

    virtual string cmd_echo(int value)
    {
        return value == 1 ? "Echo enabled" : "Echo disabled";
    }
    virtual string cmd_script(int value)
    {
        return value == 1 ? "Script enabled" : "Script disabled";
    }

    virtual string gimbal_enable(int value)
    {
        return value == 1 ? "gimbal enabled" : "gimbal disabled";
    }
    virtual string gimbal_move(float angles[2])
    {
        char temp[] = {'r', 'p'};
        string send_string = "";
        for (int i = 0; i < 2; i++)
        {
            send_string += "Activating " + to_string(temp[i]) + " for " + to_string(angles[i]) + '\n';
        }
        return send_string;
    }
};

#endif