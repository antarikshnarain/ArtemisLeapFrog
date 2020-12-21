#include "BatteryMeter.hpp"

namespace Sensors
{
    BatteryMeter::BatteryMeter(int cells, int start_pin_number)
    {
        this->no_of_cells = cells;
    }
    float * BatteryMeter::GetVoltage()
    {
        sample();
        float v[BATTERY_CELLS];
        for (int i = 0; i < BATTERY_CELLS; i++)
        {
            v[i] = 5 * (total[i] / (float)MOVING_AVERAGE_COUNT) / 1023.0f;
            v[i] *= (resis[i][0] + resis[i][1]) / resis[i][0];
            v[i] *= adjus[i];
        }

        // Calculate potential difference
        for (int i = this->no_of_cells - 1; i > 0; i--)
        {
            v[i] -= v[i - 1];
        }
        return v;
    }
    double BatteryMeter::GetPercentage()
    {
        double pert = 0.0;
        float * data = this->GetVoltage();
        for(int i=0;i<this->no_of_cells;i++)
        {
            pert += (data[i] / this->max_voltage) * 100.0;
        }
        return pert / double(this->no_of_cells); 
    }
}