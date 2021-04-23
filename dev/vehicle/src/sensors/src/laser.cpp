#include <chrono>
#include <memory>

#include <Serial.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensors/msg/sensor_laser.hpp"

using namespace std::chrono_literals;

class LaserPublisher : public rclcpp::Node, public Serial
{
private:
	rclcpp::Publisher<sensors::msg::SensorLaser>::SharedPtr sensor_publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

public:
	LaserPublisher(string port, int baud) : Node("SEN0259"), Serial(port, baud, 0, 1, 9)
	{
		// Create Publisher
		this->sensor_publisher_ = this->create_publisher<sensors::msg::SensorLaser>("laser", 10);
		// Create lambda function to publish data
		auto publish_msg = [this]() -> void {
			if(this->IsAvailable())
			{
				auto message = sensors::msg::SensorLaser();
				vector<uint8_t> data = this->Recv();
				//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Laser data: %d %d %d+%d %d+%d", data[0], data[1], data[3], data[2], data[5], data[4]);
				int sum = 0;
				// process data and update message
				message.distance = (data[3] << 8) | data[2];
				message.sig_strength = (data[5] << 8) | data[4];
				uint8_t checksum = (uint8_t)data[8];
				for(int i=0;i<8;i++)
				{
					sum += (int8_t)data[i];
				}
				sum &= 0x00FF;
				message.checksum = (sum == checksum);
				// Publish
				this->sensor_publisher_->publish(message);
			}
		};
		// register publisher with timer changed from 10ms to 100ms
		this->timer_ = this->create_wall_timer(100ms, publish_msg);
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	if (argc < 3)
	{
		printf("Pass port and baudrate as parameters");
		return -1;
	}
	rclcpp::spin(std::make_shared<LaserPublisher>(string(argv[1]), atoi(argv[2])));
	printf("Waiting for Serial port to close.\n");
	std::this_thread::sleep_for(chrono::milliseconds(1000));
	rclcpp::shutdown();
	return 0;
}