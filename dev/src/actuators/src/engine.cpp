#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensors/msg/comm_rf.hpp"

using namespace std::chrono_literals;

class JetCatP300Manager : public rclcpp::Node
{
private:
	rclcpp::Publisher<communication::msg::CommRF>::SharedPtr comm_publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

public:
	JetCatP300Manager() : Node("JetCatP300")
	{
		// Create Publisher
		this->comm_publisher_ = this->create_publisher<communication::msg::CommRF>("telemetry", 10);
		// Create lambda function to publish data
		auto publish_msg = [this]() -> void
		{
			auto message = sensors::msg::ActuatorJCP300Telemetry();
			message.distance = 1;
			// process data and update message
			// LOG Message
			// Publish
			this->comm_publisher_->publish(message);
		};
		// register publisher with timer
		this->timer_ = this->create_wall_timer(100ms, publish_msg);
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<JetCatP300Manager>());
	rclcpp::shutdown();
	return 0;
}