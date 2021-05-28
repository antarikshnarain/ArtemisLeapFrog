#include <chrono>
#include <memory>

#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "sensors/msg/sensor_linear_actuator.hpp"
#include <Serial.hpp>


using namespace std::chrono_literals;

class LinearActuatorPublisher : public rclcpp::Node, public Serial
{
private:
	rclcpp::Publisher<sensors::msg::SensorLinearActuator>::SharedPtr la_publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

protected:
	vector<string> split(string data, char delim)
	{
		vector<string> tokens;
		stringstream check(data);
    	string temp;
    	while (getline(check, temp, delim))
    	{
        	tokens.push_back(temp);
    	}
    	return tokens;
	}

public:
	LinearActuatorPublisher(string port, int baud_rate) : Node("LinearActuator"), Serial(port, baud_rate, '\n',1,-1)
	{
		// Create Publisher
		la_publisher_ = this->create_publisher<sensors::msg::SensorLinearActuator>("linear_position", 10);
		// Create lambda function to publish data
		auto publish_msg = [this]() -> void
		{
			if(this->IsAvailable())
			{
				auto message = sensors::msg::SensorLinearActuator();
				string data = this->convert_to_string(this->Recv());
				vector<string> values = this->split(data,',');
				message.position[0] = atoi(values[0].c_str());
				message.position[1] = atoi(values[1].c_str());
				this->la_publisher_->publish(message);
			}

		};
		// register publisher with timer
		this->timer_ = this->create_wall_timer(1ms, publish_msg);
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
	rclcpp::spin(std::make_shared<LinearActuatorPublisher>(string(argv[1]), atoi(argv[2])));
	rclcpp::shutdown();
	return 0;
}