#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensors/msg/sensor_imu.hpp"

using namespace std::chrono_literals;

class IMUPublisher : public rclcpp::Node
{
private:
	rclcpp::Publisher<sensors::msg::SensorImu>::SharedPtr imu_publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

public:
	IMUPublisher() : Node("MPU6050")
	{
		// Create Publisher
		imu_publisher_ = this->create_publisher<sensors::msg::SensorImu>("imu_data", 10);
		// Create lambda function to publish data
		auto publish_msg = [this]() -> void
		{
			auto message = sensors::msg::SensorImu();
			message.raw_linear_acc[0] = 1;
			message.raw_linear_acc[1] = 2;
			message.raw_linear_acc[2] = 3;
			message.raw_angular_acc[0] = 4;
			message.raw_angular_acc[1] = 5;
			message.raw_angular_acc[2] = 6;
			// process data and update message
			// LOG Message
			// Publish
			this->imu_publisher_->publish(message);
		};
		// register publisher with timer
		this->timer_ = this->create_wall_timer(100ms, publish_msg);
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<IMUPublisher>());
	rclcpp::shutdown();
	return 0;
}