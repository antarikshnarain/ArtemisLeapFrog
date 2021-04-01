#include <vector>
#ifdef __RASPBERY_PI__
#include <wiringPi.h>
#else
double analogRead(int pin)
{
    return 511;
}
#endif
using namespace std;

#define BATTERY_CELLS 6
#define MOVING_AVERAGE_COUNT 24

namespace Sensors
{

    class BatteryMeter
    {
    private:
        const double max_voltage = 3.70;
        int no_of_cells;
        int analogPinA0;

        const float resis[][2] = {{1.0, 0.0}, {1.006, 0.997}, {0.997, 2.011}, {0.997, 3.360}, {0.991, 4.69}, {0.991, 5.06}, {0.998, 6.76}};
        const float adjus[] = {0.9974, 0.9947, 1.0105, 1.00, 1.0000, 1.0000, 1.0000};
        int total[BATTERY_CELLS];
        int maIndex[BATTERY_CELLS];
        int mai;
        unsigned int samples[BATTERY_CELLS][MOVING_AVERAGE_COUNT];
    protected:
        void MovingAverage(int idx)
        {
            mai = maIndex[idx];
            total[idx] -= samples[idx][mai];
            samples[idx][mai] = analogRead(analogPinA0 + idx);
            total[idx] += samples[idx][mai];
            maIndex[idx] = (mai + 1) % MOVING_AVERAGE_COUNT;
        }
        void sample()
        {
            for (int i = 0; i < BATTERY_CELLS; i++)
            {
                MovingAverage(i);
            }
        }

    public:
        BatteryMeter(int cells, int start_pin);
        float * GetVoltage();
        double GetPercentage();
    };
} // namespace Sensors