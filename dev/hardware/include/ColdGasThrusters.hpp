/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG 
 * Author     : Antariksh Narain
 * Description: Library to control cold gas thrusters simultaneously
----------------------------------------------------------------- */

#include<map>
#include<vector>
#include<thread>
using namespace std;

namespace Actuators
{
    class ColdGasThrusters
    {
        private:
        map<int,float> solenoid_map;
        vector<float> pressure;
        vector<thread> acs_threads;
        enum
        {
            ROLL_P = 10,
            ROLL_N = 11,
            PITCH_P = 12,
            PITCH_N = 13,
            YAW_P = 14,
            YAW_N = 15
        };
        public:
        // \brief Constructor to initialize the class
        ColdGasThrusters(int count);
        // \bried Destroy all the threads
        ~ColdGasThrusters();
        // \brief Fire the selected thruster
        // \param Thurster-index
        // \param Duration
        void FireThrusters(int,float);
        vector<double> GetDurations();
        // \brief Get Tank pressure
        vector<float> GetPressure();
    };
}