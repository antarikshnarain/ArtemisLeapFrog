#include <chrono>
#include <memory>
#include <cstdio>
#include <string>
#include <Serial.hpp>
#include <future>

#include "rclcpp/rclcpp.hpp"
// JetCatP300
#include "actuators/msg/actuator_jcp300_info.hpp"
#include "actuators/msg/actuator_jcp300_telemetry.hpp"
#include "actuators/srv/actuator_jcp300_thrust.hpp"
#include "actuators/srv/actuator_jcp300_params.hpp"
#include "actuators/srv/actuator_jcp300_health_check.hpp"
#include "actuators/srv/actuator_jcp300_status.hpp"
// ACS
#include "actuators/srv/actuator_cold_gas_fire_thruster.hpp"
// Sensors
#include "sensors/msg/sensor_imu.hpp"
#include "sensors/msg/sensor_laser.hpp"

#include "flightcontrol/CommandParser.hpp"

#define HEARTBEAT_DURATION 500

using namespace std::chrono_literals;

class FlightManager : public rclcpp::Node, public Serial, public CommandParser
{
private:
    // ROS variables
    // JetCatP300
    rclcpp::Subscription<actuators::msg::ActuatorJCP300Info>::SharedPtr info_subscriber_;
	rclcpp::Subscription<actuators::msg::ActuatorJCP300Telemetry>::SharedPtr telemetry_subscriber_;
	rclcpp::Client<actuators::srv::ActuatorJCP300Thrust>::SharedPtr thrust_client_;
	rclcpp::Client<actuators::srv::ActuatorJCP300Params>::SharedPtr params_client_;
	rclcpp::Client<actuators::srv::ActuatorJCP300HealthCheck>::SharedPtr healthcheck_client_;
	rclcpp::Client<actuators::srv::ActuatorJCP300Status>::SharedPtr status_client_;
    // ACS
    rclcpp::Client<actuators::srv::ActuatorColdGasFireThruster>::SharedPtr coldgas_client_;
    // Sensors
	rclcpp::Subscription<sensors::msg::SensorImu>::SharedPtr imu_subscriber_;
	rclcpp::Subscription<sensors::msg::SensorLaser>::SharedPtr laser_subscriber_;

    // Local variables
    bool enable_engine = false;
    bool enable_acs = false;
    bool enable_sensors = false;
    bool enable_echo = false;
    int enable_script = 0;

    string sub_info;
    string sub_telemetry;
    string sub_imu;
    string sub_laser;

public:
    FlightManager(string, int, std::future<void>);

    void SerialMonitor(std::future<void>);

    void InitializeSequence();

    void ShutdownSequence();

    virtual string engine_ctrl(int value);
    virtual string engine_power(int value);
    virtual string engine_enable(int value);
    virtual string engine_telem_0();
    virtual string engine_telem_1();
    virtual string engine_telem_2();
    virtual string engine_thrust(float value);

    virtual string acs_enable(int value);
    virtual string acs_fire(float durations[6]);

    virtual string sensor_enable(int value);
    virtual string sensor_telem_0();
    virtual string sensor_telem_1();
    virtual string sensor_telem_2();

    virtual string cmd_echo(int value);
    virtual string cmd_script(int value);
};