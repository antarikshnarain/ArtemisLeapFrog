/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Vehicle Telemetry Properties
----------------------------------------------------------------- */

#include <vector>

using namespace std;
class Properties
{
    protected:
    float Base_Principal_axes[3];
    float CurrentWaypoint[3];
    float Altitude;
    vector<float> BatteryStatus;
    float FuelStatus;
    vector<float> ColdGasPressure;
    float VehiclePosition[3];
    float VehicleVelocity[3];
    float VehicleAcceleration[3];
}