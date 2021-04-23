#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "actuators/srv/actuator_cold_gas_fire_thruster.hpp"
#include <wiringPi.h>

using namespace std::chrono_literals;

#define PIN_ROLL_P 18
#define PIN_ROLL_N 23
#define PIN_PITCH_P 24
#define PIN_PITCH_N 17
#define PIN_YAW_P 27
#define PIN_YAW_N 22

class ACSManager : public rclcpp::Node
{
private:
    // ROS variables
    rclcpp::Service<actuators::srv::ActuatorColdGasFireThruster>::SharedPtr coldgas_service_;

    int get_pin[6] = {PIN_ROLL_P, PIN_ROLL_N, PIN_PITCH_P, PIN_PITCH_N, PIN_YAW_P, PIN_YAW_N};

public:
    ACSManager() : Node("ColdGasSystem")
    {
        // Create Services
        for(int i=0;i<6;i++)
        {
            pinMode(get_pin[i], OUTPUT);
        }
        this->coldgas_service_ = this->create_service<actuators::srv::ActuatorColdGasFireThruster>("cold_gas", [this](const std::shared_ptr<actuators::srv::ActuatorColdGasFireThruster::Request> request, std::shared_ptr<actuators::srv::ActuatorColdGasFireThruster::Response> response) -> void {
            std::thread threads[6];
            for (int i = 0; i < 6; i++)
            {
                threads[i] = std::thread([](int pin, int duration) {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Firing thruster %d for %dms",pin, duration);
                    digitalWrite(pin, 1);
                    std::this_thread::sleep_for(std::chrono::milliseconds(duration));
                    digitalWrite(pin, 0);
                },
                                         get_pin[i], request->durations[i]);
            }
            for (int i = 0; i < 6; i++)
            {
                threads[i].join();
            }
            response->status = true;
        });
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ACS service Initialized!");

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