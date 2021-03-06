#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensors/msg/sensor_imu.hpp"
#include "sensors/MPU6050.hpp"

//#include <KalmanFilter.hpp>

using namespace std::chrono_literals;

class IMUPublisher : public rclcpp::Node, public MPU6050
{
private:
	rclcpp::Publisher<sensors::msg::SensorImu>::SharedPtr imu_publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

public:
	IMUPublisher(int device_id) : Node("MPU6050"), MPU6050(device_id, 1000)
	{
		// Create Publisher
		imu_publisher_ = this->create_publisher<sensors::msg::SensorImu>("imu_data", 10);
		// Create lambda function to publish data
		auto publish_msg = [this]() -> void
		{
			vector<float> raw_values = this->RawValues();
			vector<float> rpy_values = this->PrincipalAxisValues();
			auto message = sensors::msg::SensorImu();
			message.raw_linear_acc[0] = raw_values[0];
			message.raw_linear_acc[1] = raw_values[1];
			message.raw_linear_acc[2] = raw_values[2];
			message.raw_angular_acc[0] = raw_values[3];
			message.raw_angular_acc[1] = raw_values[4];
			message.raw_angular_acc[2] = raw_values[5];
			message.temp = raw_values[6];
			message.roll = rpy_values[0];
			message.pitch = rpy_values[1];
			// process data and update message
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
	rclcpp::spin(std::make_shared<IMUPublisher>(atoi(argv[1])));
	rclcpp::shutdown();
	return 0;
}