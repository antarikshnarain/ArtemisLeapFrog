/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Engine Control command for Ground Station
----------------------------------------------------------------- */

namespace GroundStation
{
    class EngineController
    {
    private:
        bool control_state;
        bool power_state;
        float engine_thrust;

    public:
        bool EnableEnginePin();
        bool EnablePower();
        void ChangeThrust(float value);
    };
} // namespace GroundStation