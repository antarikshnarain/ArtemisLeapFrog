#ifndef KalmanFilter_h
#define KalmanFilter_h

#include <chrono>

using namespace std::chrono;

class KalmanFilter
{
    public:

	KalmanFilter(double angle = 0.001, double bias = 0.003, double measure = 0.03);
	double update(double newValue, double newRate);

    protected:
    double micros();
    private:

	double Q_angle, Q_bias, R_measure;
	double K_angle, K_bias, K_rate;
	double P[2][2], K[2];
	double S, y;
	double dt, kt;
};

#endif