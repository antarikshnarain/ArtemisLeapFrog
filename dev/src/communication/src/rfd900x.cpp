#include <chrono>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "communication/srv/comm_rf_send.hpp"
#include "communication/srv/comm_rf_recv.hpp"
#include "communication/Serial.hpp"

using namespace std::chrono_literals;

class RFD900xManager : public rclcpp::Node, public Serial
{
private:
	// ROS variables
	rclcpp::Service<communication::srv::CommRFSend>::SharedPtr send_service_;
	rclcpp::Service<communication::srv::CommRFRecv>::SharedPtr recv_service_;

	// Local variables
	bool ctrl_sig = false;
	bool pwr_sig = false;

public:
	RFD900xManager(string port, int baud) : Node("RFD900X"), Serial(port, baud)
	{
		// Create Services
		this->send_service_ = this->create_service<communication::srv::CommRFSend>("SendData", [this](const std::shared_ptr<communication::srv::CommRFSend::Request> request, std::shared_ptr<communication::srv::CommRFSend::Response> response) -> void {
			response->status = this->Send(request->data);
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending data");
		});
		this->recv_service_ = this->create_service<communication::srv::CommRFRecv>("RecvData", [this](const std::shared_ptr<communication::srv::CommRFRecv::Request> request, std::shared_ptr<communication::srv::CommRFRecv::Response> response) -> void {
			//response->data = this->Recv();
			if(request->size == 0)
			{
				response->data = this->Recv();
			}
			else
			{
				response->data = this->Recv(request->size);
			}
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Receiving data");
		});

		RCLCPP_INFO(this->get_logger(), "RFD900x Send and Receive Server ready.");
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	if (argc != 3)
	{
		printf("Pass port and baud rate as parameters.\n");
		return -1;
	}
	rclcpp::spin(std::make_shared<RFD900xManager>(string(argv[1]), atoi(argv[2])));
	rclcpp::shutdown();
	return 0;
}