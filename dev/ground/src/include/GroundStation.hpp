/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Library containing functionality of Ground Station
----------------------------------------------------------------- */

#include "Properties.hpp"
#include "EngineController.hpp"
#include "GimbalController.hpp"
#include "ThrusterController.hpp"

class GroundStation : public Properties, public EngineController, public GimbalController, public ThrusterController
{
private:
public:
    void GetVehicleTelemetry();
    void EngineControl();
    void GimbalControl();
    void ThrustControl();
    void SetWaypoints();
}