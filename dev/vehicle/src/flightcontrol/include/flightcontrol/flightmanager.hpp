#include <chrono>
#include <memory>
#include <cstdio>
#include <string>
#include <Serial.hpp>
#include <future>
#include <thread>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
// JetCatP300
#include "actuators/msg/actuator_jcp300_info.hpp"
#include "actuators/msg/actuator_jcp300_engine_telemetry.hpp"
#include "actuators/msg/actuator_jcp300_fuel_telemetry.hpp"
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

#define HEARTBEAT_DURATION 2000

using namespace std::chrono_literals;

class FlightManager : public rclcpp::Node, public Serial, public CommandParser
{
private:
    // ROS variables
    // JetCatP300
    rclcpp::Subscription<actuators::msg::ActuatorJCP300Info>::SharedPtr info_subscriber_;
	rclcpp::Subscription<actuators::msg::ActuatorJCP300EngineTelemetry>::SharedPtr engine_telemetry_subscriber_;
	rclcpp::Subscription<actuators::msg::ActuatorJCP300FuelTelemetry>::SharedPtr fuel_telemetry_subscriber_;
	rclcpp::Client<actuators::srv::ActuatorJCP300Thrust>::SharedPtr thrust_client_;
	rclcpp::Client<actuators::srv::ActuatorJCP300Params>::SharedPtr params_client_;
	rclcpp::Client<actuators::srv::ActuatorJCP300HealthCheck>::SharedPtr healthcheck_client_;
	rclcpp::Client<actuators::srv::ActuatorJCP300Status>::SharedPtr status_client_;
    // ACS
    rclcpp::Client<actuators::srv::ActuatorColdGasFireThruster>::SharedPtr coldgas_client_;
    // Sensors
	rclcpp::Subscription<sensors::msg::SensorImu>::SharedPtr imu_subscriber_;
	rclcpp::Subscription<sensors::msg::SensorLaser>::SharedPtr laser_subscriber_;
    // Gimbal
    rclcpp::Client<actuators::srv::ActuatorMoveGimbal>::SharedPtr gimbal_client_;

    // Local variables
    bool enable_engine = false;
    bool enable_acs = false;
    bool enable_sensors = false;
    bool enable_echo = false;
    bool enable_gimbal = false;
    int enable_script = 0;

    string sub_info;
    string sub_engine_telemetry;
    string sub_fuel_telemetry;
    string sub_imu;
    string sub_laser;

    //promise<void> exit_script_thread_promise;
    promise<void> *exit_script_thread_promise = NULL;
    //future<void> exit_script_thread_future;
    future<void> *exit_script_thread_future = NULL;
    thread script_thread;

    const string path_to_files = "/usr/local/share/script/";
    const string custom_script_file = "custom_scripts.list";
    vector<string> script_map;
public:
    FlightManager(string, int, std::future<void>);

    void SerialMonitor(std::future<void>);

    void InitializeSequence();

    void ShutdownSequence();

    void ScriptRunner(string filename, std::future<void>);

    virtual string engine_ctrl(int value);
    virtual string engine_power(int value);
    virtual string engine_enable(int value);
    virtual string engine_telem_0();
    virtual string engine_telem_1();
    virtual string engine_telem_2();
    virtual string engine_telem_3();
    virtual string engine_thrust(float value);

    virtual string acs_enable(int value);
    virtual string acs_fire(int durations[6]);

    virtual string sensor_enable(int value);
    virtual string sensor_telem_0();
    virtual string sensor_telem_1();
    virtual string sensor_telem_2();

    virtual string cmd_echo(int value);
    virtual string cmd_script(int value);

    virtual string gimbal_enable(int value);
    virtual string gimbal_move(float angles[2]);
};