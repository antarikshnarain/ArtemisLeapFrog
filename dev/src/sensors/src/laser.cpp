#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensors/msg/sensor_laser.hpp"
#include "sensors/SEN0259.hpp"

using namespace std::chrono_literals;

class LaserPublisher : public rclcpp::Node, public SEN0259
{
private:
	rclcpp::Publisher<sensors::msg::SensorLaser>::SharedPtr sensor_publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

public:
	LaserPublisher(string port, int baud) : Node("SEN0259"), SEN0259(port, baud)
	{
		// Create Publisher
		this->sensor_publisher_ = this->create_publisher<sensors::msg::SensorLaser>("laser", 10);
		// Create lambda function to publish data
		auto publish_msg = [this]() -> void {
			auto message = sensors::msg::SensorLaser();
			// process data and update message
			message.distance = this->GetDistance();
			message.sig_strength = this->GetStrength();
			// Publish
			this->sensor_publisher_->publish(message);
		};
		// register publisher with timer
		this->timer_ = this->create_wall_timer(10ms, publish_msg);
	}
};

int main(int argc, char *argv[])
{
	printf("No Args: %d\n", argc);
	rclcpp::init(argc, argv);
	printf("No Args: %d %s %s\n", argc, argv[1], argv[2]);
	for(int i=0;i<argc;i++)
	{
		printf("%s ", argv[i]);
	}
	printf("\n");
	if (argc < 3)
	{
		printf("Pass port and baudrate as parameters");
		return -1;
	}
	rclcpp::spin(std::make_shared<LaserPublisher>(string(argv[1]), atoi(argv[2])));
	rclcpp::shutdown();
	return 0;
}