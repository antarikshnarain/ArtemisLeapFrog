#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "actuators/msg/actuator_jcp300_info.hpp"
#include "actuators/msg/actuator_jcp300_telemetry.hpp"
#include "actuators/srv/actuator_jcp300_thrust.hpp"
#include "actuators/srv/actuator_jcp300_params.hpp"
#include "actuators/srv/actuator_jcp300_health_check.hpp"
#include "actuators/srv/actuator_jcp300_status.hpp"
#include "actuators/JetCatP300.hpp"

using namespace std::chrono_literals;

class JetCatP300Manager : public rclcpp::Node, public JetCatP300
{
private:
	// ROS variables
	// Subscriptions
	rclcpp::Subscription<actuators::msg::ActuatorJCP300Info>::SharedPtr info_subscriber_;
	rclcpp::Subscription<actuators::msg::ActuatorJCP300Telemetry>::SharedPtr telemetry_subscriber_;
	rclcpp::Subscription<sensors::msg::SensorImu>::SharedPtr imu_subscriber_;
	rclcpp::Subscription<sensors::msg::SensorLaser>::SharedPtr sensor_subscriber_;
	// Add Subscription to system temperature

	rclcpp::Client<actuators::srv::ActuatorJCP300Thrust>::SharedPtr thrust_client_;
	rclcpp::Client<actuators::srv::ActuatorJCP300Params>::SharedPtr params_client_;
	rclcpp::Client<actuators::srv::ActuatorJCP300HealthCheck>::SharedPtr healthcheck_client_;
	rclcpp::Client<actuators::srv::ActuatorJCP300Status>::SharedPtr status_client_;
	rclcpp::TimerBase::SharedPtr timer_[3];

	// Local variables
	bool ctrl_sig = false;
	bool pwr_sig = false;
	bool engine_started = false;
	float current_thrust = 0.0;

public:
	JetCatP300Manager(string port, int baudrate) : Node("JetCatP300"), JetCatP300(port, baudrate)
	{
		// Create Publishers
		this->info_publisher_ = this->create_publisher<actuators::msg::ActuatorJCP300Info>("info", 10);
		this->telemetry_publisher_ = this->create_publisher<actuators::msg::ActuatorJCP300Telemetry>("telemetry", 10);

		this->timer_[1] = this->create_wall_timer(100ms, std::bind(&JetCatP300Manager::GetEngineInfo, this));
		this->timer_[2] = this->create_wall_timer(100ms, std::bind(&JetCatP300Manager::GetEngineTelemetry, this));

		// Create Services
		this->thrust_service_ = this->create_service<actuators::srv::ActuatorJCP300Thrust>("thrust", [this](const std::shared_ptr<actuators::srv::ActuatorJCP300Thrust::Request> request, std::shared_ptr<actuators::srv::ActuatorJCP300Thrust::Response> response) -> void {
			float new_value = request->thrust_value;
			if (new_value < 0.0)
			{
				new_value = 0.0;
			}
			if (new_value > 100.0)
			{
				new_value = 100.0;
			}
			if (!this->send_command(RS232{1, "TRR", 1, to_string(new_value)}))
			{
				fprintf(stderr, "SetEngineThrust(): failed!");
				response->status = false;
				return;
			}
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Updating thrust from %.4f to %.4f", this->current_thrust, new_value);
			this->current_thrust = new_value;
			response->status = true;
		});

		this->params_service_ = this->create_service<actuators::srv::ActuatorJCP300Params>("parameters", [this](const std::shared_ptr<actuators::srv::ActuatorJCP300Params::Request> request, std::shared_ptr<actuators::srv::ActuatorJCP300Params::Response> response) -> void {
			response->status = 0;
			if (this->ctrl_sig != request->ctrl_sig)
			{
				this->ctrl_sig = request->ctrl_sig;
				digitalWrite(JETCAT_SAFE, this->ctrl_sig);
				response->status |= int(this->ctrl_sig);
			}
			if (this->pwr_sig != request->pwr_sig)
			{
				this->pwr_sig = request->pwr_sig;
				digitalWrite(JETCAT_POWER, this->pwr_sig);
				response->status |= (int(this->ctrl_sig) << 1);
			}
		});

		this->healthcheck_service_ = this->create_service<actuators::srv::ActuatorJCP300HealthCheck>("health_check", [this](const std::shared_ptr<actuators::srv::ActuatorJCP300HealthCheck::Request> request, std::shared_ptr<actuators::srv::ActuatorJCP300HealthCheck::Response> response) -> void {
			if(!request->check_health)
			{
				return;
			}
			printf("Initializing Check Health!\n");
			if (!this->send_command(RS232{1, "DHC", 0, "1"}))
			{
				fprintf(stderr, "CheckHealth(): failed!");
			}
			// Force wait for 10 seconds
			for (int i = 10; i >= 0; i--)
			{
				printf("Waiting %d...\r", i);
				usleep(1000000);
			}
			printf("Analyzing Results\n");
			if (!this->send_command(RS232{1, "RHC", 0, "1"}))
			{
				fprintf(stderr, "CheckHealth():RHC failed!");
			}

			RS232 rs232_response = this->receive_response();
			// Logic to process flags
			const string health_params[7] = {"Starter", "Main Valve", "Starter Valve", "RPM Sensor", "Pump", "GlowPlug", "EGT Sensor"};
			string msg = "";
			if (rs232_response.len == 7)
			{
				// Verify Bits
				for (int i = 0; i < rs232_response.len; i++)
				{
					msg += "Testing " + health_params[i] + " -- ";
					if (atoi(rs232_response.params[i].c_str()) & 0x01)
					{
						msg += "OK. ";
					}
					if (atoi(rs232_response.params[i].c_str()) & 0x02)
					{
						msg += "Driver error. ";
					}
					if (atoi(rs232_response.params[i].c_str()) & 0x04)
					{
						msg += "No Current. ";
					}
					if (atoi(rs232_response.params[i].c_str()) & 0x08)
					{
						msg += "Engine Failure, refer manual! ";
					}
					msg += "\n";
				}
			}
			response->status_message = msg;
		});

		this->status_service_ = this->create_service<actuators::srv::ActuatorJCP300Status>("engine_status", [this](const std::shared_ptr<actuators::srv::ActuatorJCP300Status::Request> request, std::shared_ptr<actuators::srv::ActuatorJCP300Status::Response> response) -> void {
			response->status = false;
			if (this->engine_started != request->state)
			{
				this->engine_started = request->state;
				if (!this->send_command(RS232{1, "TCO", 1, (this->engine_started ? "1" : "0")}))
				{
					fprintf(stderr, "Engine Status change failed!");
					return;
				}
				response->status = true;
			}
		});
	}
	void GetEngineInfo()
	{
		auto message = actuators::msg::ActuatorJCP300Info();
		printf("Fetching Jet Engine information!\n");
		if (!this->send_command(RS232{1, "RTY", 1, "1"}))
		{
			fprintf(stderr, "EngineInformation(): failed!\n");
		}
		// process data and update message
		RS232 response = this->receive_response();
		if (response.len < 6)
		{
			return;
		}
		message.firmware_version = response.params[0];
		message.version_number = response.params[1];
		message.last_time_run = response.params[2];
		message.total_operation_time = response.params[3];
		message.serial_number = response.params[4];
		message.turbine_type = response.params[5];
		// Publish
		this->info_publisher_->publish(message);
	}
	void GetEngineTelemetry()
	{
		auto message = actuators::msg::ActuatorJCP300Telemetry();
		printf("Getting Engine Telemetry!\n");
		if (!this->send_command(RS232{1, "RFI", 1, "1"}))
		{
			fprintf(stderr, "GetTelemetry(): falied!\n");
		}
		RS232 response = this->receive_response();
		if (response.len < 6)
		{
			printf("Response does not follow the format.\n");
			return;
		}
		message.actual_fuel = atoi(response.params[0].c_str());
		message.rest_fuel = atoi(response.params[1].c_str());
		message.rpm = atoi(response.params[2].c_str());
		message.battery_voltage = atof(response.params[3].c_str());
		message.last_run = atoi(response.params[4].c_str());
		message.fuel_actual_run = atoi(response.params[5].c_str());
		// Publish
		this->telemetry_publisher_->publish(message);
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<JetCatP300Manager>(string(argv[1]), atoi(argv[2])));
	rclcpp::shutdown();
	return 0;
}