#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "actuators/msg/actuator_jcp300_info.hpp"
#include "actuators/msg/actuator_jcp300_telemetry.hpp"
#include "sensors/msg/sensor_imu.hpp"
#include "sensors/msg/sensor_laser.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class HeartBeat : public rclcpp::Node
{
private:
	// ROS variables
	// Subscriptions
	rclcpp::Subscription<actuators::msg::ActuatorJCP300Info>::SharedPtr info_subscriber_;
	rclcpp::Subscription<actuators::msg::ActuatorJCP300Telemetry>::SharedPtr telemetry_subscriber_;
	rclcpp::Subscription<sensors::msg::SensorImu>::SharedPtr imu_subscriber_;
	rclcpp::Subscription<sensors::msg::SensorLaser>::SharedPtr laser_subscriber_;
	// Add Subscription to system temperature

	rclcpp::TimerBase::SharedPtr timer_[3];

	// Local variables

public:
	HeartBeat() : Node("HeartBeat")
	{
		// Create Subscribers
		this->info_subscriber_ = this->create_subscription<actuators::msg::ActuatorJCP300Info>("info", 10, [this](const actuators::msg::ActuatorJCP300Info::SharedPtr msg) -> void {
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JCP300-Info:%s,%s,%d,%d,%s,%s",
						msg->firmware_version, msg->version_number, msg->last_time_run, msg->total_operation_time, msg->serial_number, msg->turbine_type);
		});

		this->telemetry_subscriber_ = this->create_subscription<actuators::msg::ActuatorJCP300Telemetry>("telemetry", 10, [this](const actuators::msg::ActuatorJCP300Telemetry::SharedPtr msg) -> void {
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JCP300-Telem:%d,%d,%d,%.4f,%d,%d",
						msg->actual_fuel, msg->rest_fuel, msg->rpm, msg->battery_voltage, msg->last_run, msg->fuel_actual_run);
		});
		this->imu_subscriber_ = this->create_subscription<sensors::msg::SensorImu>("imu", 10, [this](const sensors::msg::SensorImu::SharedPtr msg) -> void {
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sensors-IMU:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
						msg->raw_linear_acc[0], msg->raw_linear_acc[1], msg->raw_linear_acc[2], msg->raw_angular_acc[0], msg->raw_angular_acc[1], msg->raw_angular_acc[2],
						msg->roll, msg->pitch, msg->temp, msg->linear_acc[0], msg->linear_acc[1], msg->linear_acc[2], msg->angular_acc[0], msg->angular_acc[1], msg->angular_acc[2]);
		});
		this->laser_subscriber_ = this->create_subscription<sensors::msg::SensorLaser>("laser", 10, [this](const sensors::msg::SensorLaser::SharedPtr msg) -> void {
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sensors-Laser:%d,%d", msg->distance, msg->sig_strength);
		});
		//this->telemetry_publisher_ = this->create_publisher<actuators::msg::ActuatorJCP300Telemetry>("telemetry", 10);
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<HeartBeat>());
	rclcpp::shutdown();
	return 0;
}