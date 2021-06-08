#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensors/msg/sensor_linear_actuator.hpp"
#include "sensors/ADS1015.hpp"


using namespace std::chrono_literals;

class LinearActuatorPublisher : public rclcpp::Node, public ADS1015
{
private:
	rclcpp::Publisher<sensors::msg::SensorLinearActuator>::SharedPtr la_publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

public:
	LinearActuatorPublisher(int device_id) : Node("LinearActuator"), ADS1015(device_id)
	{
		// Create Publisher
		la_publisher_ = this->create_publisher<sensors::msg::SensorLinearActuator>("linear_position", 10);
		// Create lambda function to publish data
		auto publish_msg = [this]() -> void
		{

			auto message = sensors::msg::SensorLinearActuator();
			message.position[0] = this->analog_value[0];
			message.position[1] = this->analog_value[1];
			this->la_publisher_->publish(message);
		};
		// register publisher with timer
		this->timer_ = this->create_wall_timer(100ms, publish_msg);
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	if(argc < 2)
	{
		printf("Pass device id");
		return -1;
	}
	int dev_addr = atoi(argv[1]);
	if(dev_addr == 0)
	{
		printf("Pass value as integer and not hex.\n");
		return -1;
	}
	rclcpp::spin(std::make_shared<LinearActuatorPublisher>(atoi(argv[1])));
	rclcpp::shutdown();
	return 0;
}