#include <chrono>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "actuators/srv/actuator_cold_gas_fire_thruster.hpp"
#include "actuators/msg/actuator_cold_gas_pressure.hpp"
#include "sensors/msg/sensor_imu.hpp"
#include <wiringPi.h>
#include <PID.hpp>

using namespace std::chrono_literals;

#define PIN_ROLL_P 27
#define PIN_ROLL_N 22
#define PIN_PITCH_P 16
#define PIN_PITCH_N 19
#define PIN_YAW_P 27
#define PIN_YAW_N 22

class ACSManager : public rclcpp::Node
{
private:
    // ROS variables
    rclcpp::Service<actuators::srv::ActuatorColdGasFireThruster>::SharedPtr coldgas_service_;
    rclcpp::Publisher<actuators::msg::ActuatorColdGasPressure>::SharedPtr pressure_publisher_;
    rclcpp::Subscription<sensors::msg::SensorImu>::SharedPtr imu_subscriber_;
	rclcpp::TimerBase::SharedPtr timer_;

    int get_pin[6] = {PIN_ROLL_P, PIN_ROLL_N, PIN_PITCH_P, PIN_PITCH_N, PIN_YAW_P, PIN_YAW_N};
    int valve_to_tank[6] = {0,1,2,3,2,3};
    int pressure_available[4] = {5000}; // 5000ms for each tank

    std::mutex _mutex;

    PID pid_roll = PID(0.1, 1000, 0, 1.5, 1.0, 0);
    PID pid_pitch = PID(0.1, 1000, 0, 1.5, 1.0, 0);

protected:
    bool updateCapacity(int pin, int duration)
    {
        // Equation: F = -0.7t + 3.5; -> Assumption tanks are full in capacity.
        _mutex.lock();
        if(pressure_available[valve_to_tank[pin]] - duration < 0)
        {
            _mutex.unlock();
            return false;
        }
        pressure_available[valve_to_tank[pin]] -= duration;
        _mutex.unlock();
        if(pressure_available[valve_to_tank[pin]] < 1500)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Low pressure in tank %d, please initialize landing!", valve_to_tank[pin]);
        }
        return true;
    }

public:
    ACSManager() : Node("ColdGasSystem")
    {
        // Create Publishers
		this->pressure_publisher_ = this->create_publisher<actuators::msg::ActuatorColdGasPressure>("pressure", 10);
		this->timer_ = this->create_wall_timer(100ms, std::bind(&ACSManager::ColdGasPressure, this));
        // Create Services
        for(int i=0;i<6;i++)
        {
            pinMode(get_pin[i], OUTPUT);
        }
        this->coldgas_service_ = this->create_service<actuators::srv::ActuatorColdGasFireThruster>("cold_gas", [this](const std::shared_ptr<actuators::srv::ActuatorColdGasFireThruster::Request> request, std::shared_ptr<actuators::srv::ActuatorColdGasFireThruster::Response> response) -> void {
            std::thread threads[6];
            for (int i = 0; i < 6; i++)
            {
                threads[i] = std::thread([this](int pin, int duration) {
                    if(this->updateCapacity(pin, duration))
                    {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Firing thruster %d for %dms",pin, duration);
                        digitalWrite(pin, 1);
                        std::this_thread::sleep_for(std::chrono::milliseconds(duration));
                        digitalWrite(pin, 0);
                    }
                    else
                    {
                        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Low on capacity, not firing thruster %d for %dms", pin, duration);
                    }
                    
                },
                                         get_pin[i], request->durations[i]);
            }
            for (int i = 0; i < 6; i++)
            {
                threads[i].join();
            }
            response->status = true;
        });

        this->imu_subscriber_ = this->create_subscription<sensors::msg::SensorImu>("/sensors/imu_data", 10, [this](const sensors::msg::SensorImu::SharedPtr msg) -> void {
            
        // Logic to actuator valves
        // Read sensor data
        // Process data for actuation
        /* Algorithm:-
         * Constants - stable angle range 
         * Input: current vehicle angle
         * Output: thruster to fire and the duration
         * rpy = read_sensor_data;
         * for each RPY:
         *      calculate the error
         *      pass to PID controller
         *      get fire duration as output in ms
        */
        int durations[2] = {(int) pid_roll.calculate(0.0, msg->roll), (int) pid_pitch.calculate(0.0, msg->pitch)};
        std::thread threads[2];
        for (int i = 0; i < 6; i++)
        {
            int pin = durations[i] > 0 ? 2*i : 2*i + 1;
            threads[i] = std::thread([this](int pin, int duration) {
                if(this->updateCapacity(pin, duration))
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Firing thruster %d for %dms",pin, duration);
                    digitalWrite(pin, 1);
                    std::this_thread::sleep_for(std::chrono::milliseconds(duration));
                    digitalWrite(pin, 0);
                }
                else
                {
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Low on capacity, not firing thruster %d for %dms", pin, duration);
                }
                
            },
                                        get_pin[pin], durations[i]);
        }
        for (int i = 0; i < 6; i++)
        {
            threads[i].join();
        }

        // current_rpy = 
        // if(!this->enable_engine)
		// {
		// 	return;
		// }
		// char buffer[256];
		// if(sprintf(buffer, "%d,%d,%.4f,%d,%.4f,%.4f", msg->turbine_rpm, msg->egt_temp, msg->pump_voltage, msg->turbine_state, msg->throttle_position, msg->engine_current) > 0)
		// {
		// 	this->sub_engine_telemetry = std::string(buffer);
		// }
		// RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JCP300-Engine-Telem:%d,%d,%.4f,%d,%.4f,%.4f", 
		// 	msg->turbine_rpm, msg->egt_temp, msg->pump_voltage, msg->turbine_state, msg->throttle_position, msg->engine_current);
	});
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ACS service Initialized!");

    }
    void ColdGasPressure()
    {
        auto message = actuators::msg::ActuatorColdGasPressure();
        for(int i=0;i<4;i++)
        {
            message.pressure_available[i] = this->pressure_available[i];
        }
		// Publish
		this->pressure_publisher_->publish(message);
    }
};

int main(int argc, char *argv[])
{
    wiringPiSetupGpio();
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ACSManager>());
    rclcpp::shutdown();
    return 0;
}