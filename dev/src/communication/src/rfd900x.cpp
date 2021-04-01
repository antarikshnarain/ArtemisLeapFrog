#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "communication/srv/comm_rf_send.hpp"
#include "communication/srv/comm_rf_recv.hpp"
#include "communication/Serial.hpp"

using namespace std::chrono_literals;

std::unique_ptr<Serial> serial;

void RFDSend(const std::shared_ptr<communication::srv::CommRFSend::Request> request, 
	std::shared_ptr<communication::srv::CommRFSend::Response> response)
{
	response->status = serial->Send(request->data);
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending data");
}
void RFDRecv(const std::shared_ptr<communication::srv::CommRFRecv::Request> request, 
	std::shared_ptr<communication::srv::CommRFRecv::Response> response)
{
	response->data = serial->Recv();
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Receiving data");
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	if(argc != 3)
	{
		printf("Pass port and baud rate as parameters.\n");
		return -1;
	}
	serial = std::make_unique<Serial>(string(argv[1]), atoi(argv[2]));
	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("rfd900x");
	rclcpp::Service<communication::srv::CommRFSend>::SharedPtr serviceSend = 
		node->create_service<communication::srv::CommRFSend>("SendData", &RFDSend);
	rclcpp::Service<communication::srv::CommRFRecv>::SharedPtr serviceRecv = 
		node->create_service<communication::srv::CommRFRecv>("RecvData", &RFDRecv);

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RFD900x Send and Receive Server ready.");
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}