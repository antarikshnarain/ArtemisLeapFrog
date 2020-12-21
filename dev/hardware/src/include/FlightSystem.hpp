#include "Properties.hpp"

#pragma region Actuators
#include "JetCatP300.hpp"
#include "LinearActuator.hpp"
using namespace Actuators;
#pragma endregion

#pragma region Sensors
#include "MPU6050.hpp"
#include "SEN0259.h"
using namespace Sensors;
#pragma endregion

#include <vector>
#include <string>

using namespace std;

class FlightSystem: public Properties
{
    private:
    bool heartBeat;
    int current_waypoint;

    protected:
    string ProcessCommand(string);

    public:
    
    FlightSystem();
    ~FlightSystem();
    void InitializeVehicleSystem();
    void CalibrateSensors();
    void VehicleTestSequence();
    void GetVehicleTelemetry();
    void StartACS();
    vector<double> GetCurrentPosition();
};