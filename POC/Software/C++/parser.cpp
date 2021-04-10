#include <bits/stdc++.h>
using namespace std;

class CommandParser
{
private:
    const string INVALID_COMMAND = "Invalid Command.";
    map<string, int> cg_map = {{"r",0},{"R",1},{"p",2},{"P",3},{"y",4},{"Y",5}};
    vector<string> split(string data, char delim)
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

protected:
    string engineParser(string cmd, string values)
    {
        float value = atof(values.c_str());
        
        if(cmd == "ctrl")
            // service
            return value == 1 ? "Control enabled" : "Control disabled";
        else if(cmd == "power")
            // service
            return value == 1 ? "Power enabled" : "Power disabled";
        else if(cmd == "enable")
            // service
            return value == 1 ? "Engine started" : "Engine stopped";
        else if(cmd == "telem")
        {
            if(value == 0)
            {
                // service x2
                return "Telemetry of Health Check";
            }
            else if(value == 1)
            {
                // Parameters
                return "Telemetry of System Info";
            }
            else if(value == 2)
            {
                // Publisher
                return "Engine Telemetry";
            }
            else
            {
                return INVALID_COMMAND;
            }
        }
        else if(cmd == "thrust")
            // Action service
            return "Setting thrust to " + to_string(value);
        else
            return INVALID_COMMAND;
    }
    string acsParser(string cmd, string values)
    {
        if(cmd == "enable")
        {
            // software flag update
            int value = atoi(values.c_str());
            if(value == 1)
            {
                return "enabling acs";
            }
            else if(value == 0)
            {
                return "disabling acs";
            }
            else
            {
                return INVALID_COMMAND;
            }
        }
        else if(cmd == "fire")
        {
            // service
            int durations[6] = {0};
            vector<string> tokens = this->split(values, ',');
            for(string token:tokens)
            {
                vector<string> thrust = this->split(token, '=');
                if(thrust.size() != 2)
                {
                    return INVALID_COMMAND;
                }
                auto pos = this->cg_map.find(thrust[0]);
                if (pos == this->cg_map.end()) {
                    return INVALID_COMMAND;
                } else {
                    durations[pos->second] = atoi(thrust[1].c_str());
                }
            }
            // If successful activate the thrusters
            char temp[] = {'r','R', 'p', 'P', 'y', 'Y'};
            string send_string = "";
            for(int i=0;i<6;i++)
            {
                send_string += "Activating " + to_string(temp[i]) + " for " + to_string(durations[i]) + '\n';
            }
            return send_string;
        }
        else
            return INVALID_COMMAND;
    }
    string sensorsParser(string cmd, string values)
    {
        int value = atoi(values.c_str());
        if(cmd == "enable")
            // software flag update
            return value == 1 ? "Sensors started" : "Sensors stopped";
        else if(cmd == "telem")
        {
            if(value == 0)
            {
                // Subscriber
                return "Telemetry of IMU";
            }
            else if(value == 1)
            {
                // subscriber
                return "Telemetry of Laser";
            }
            else if(value == 2)
            {
                // subscriber
                return "Battery Telemetry";
            }
            else
            {
                return INVALID_COMMAND;
            }
        }
        else
            return INVALID_COMMAND;

    }
    string cmdParser(string cmd, string values)
    {
        int value = atoi(values.c_str());
        if(cmd == "echo")
            // software flag logic 
            return value == 1 ? "Echo enabled" : "Echo disabled";
        else if(cmd == "script")
            // complex software logic
            // value -> name of script if value == stop -> cancel all script exec
            // runs as a thread
            return value == 1 ? "Script enabled" : "Script disabled";
        else
            return INVALID_COMMAND;
    }

public:
    string Parser(string cmd)
    {
        vector<string> tokens = this->split(cmd, ' ');
        if (tokens.size() != 3)
        {
            return INVALID_COMMAND;
        }
        if(tokens[0] == "engine")
            return this->engineParser(tokens[1], tokens[2]);
        else if(tokens[0] == "acs")
            return this->acsParser(tokens[1], tokens[2]);
        else if(tokens[0] == "sensors")
            return this->sensorsParser(tokens[1], tokens[2]);
        else if(tokens[0] == "cmd")
            return this->cmdParser(tokens[1], tokens[2]);
        else
            return this->INVALID_COMMAND;
    }
};

int main()
{
    char data[100];
    CommandParser cp;
    while(1)
    {
        cin.getline(data, sizeof(data));
        if(string(data) == "exit")
        {
            break;
        }
        cout << cp.Parser(data) << endl;
    }
}