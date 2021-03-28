#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensors/msg/sensor_laser.hpp"

using namespace std::chrono_literals;

class LaserPublisher : public rclcpp::Node
{
private:
	rclcpp::Publisher<sensors::msg::SensorLaser>::SharedPtr sensor_publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

public:
	LaserPublisher() : Node("SEN0259")
	{
		// Create Publisher
		this->sensor_publisher_ = this->create_publisher<sensors::msg::SensorLaser>("laser", 10);
		// Create lambda function to publish data
		auto publish_msg = [this]() -> void
		{
			auto message = sensors::msg::SensorLaser();
			message.distance = 1;
			// process data and update message
			// LOG Message
			// Publish
			this->sensor_publisher_->publish(message);
		};
		// register publisher with timer
		this->timer_ = this->create_wall_timer(100ms, publish_msg);
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LaserPublisher>());
	rclcpp::shutdown();
	return 0;
}