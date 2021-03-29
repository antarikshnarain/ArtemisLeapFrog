#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensors/msg/comm_rf.hpp"

using namespace std::chrono_literals;

class RFD900xManager : public rclcpp::Node
{
private:
	rclcpp::Publisher<communication::msg::CommRF>::SharedPtr comm_publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

public:
	RFD900xManager() : Node("RFD900X")
	{
		// Create Publisher
		this->comm_publisher_ = this->create_publisher<communication::msg::CommRF>("laser", 10);
		// Create lambda function to publish data
		auto publish_msg = [this]() -> void
		{
			auto message = sensors::msg::CommRF();
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
	rclcpp::spin(std::make_shared<RFD900xManager>());
	rclcpp::shutdown();
	return 0;
}