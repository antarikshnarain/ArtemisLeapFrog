#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "actuators/msg/actuator_jcp300_info.hpp"
#include "actuators/msg/actuator_jcp300_engine_telemetry.hpp"
#include "actuators/msg/actuator_jcp300_fuel_telemetry.hpp"
#include "actuators/srv/actuator_jcp300_thrust.hpp"
#include "actuators/srv/actuator_jcp300_thrust2.hpp"
#include "actuators/msg/actuator_jcp300_system_status.hpp"
#include "actuators/srv/actuator_jcp300_params.hpp"
#include "actuators/srv/actuator_jcp300_health_check.hpp"
#include "actuators/srv/actuator_jcp300_status.hpp"
#include "actuators/JetCatP300.hpp"

using namespace std::chrono_literals;

class JetCatP300Manager : public rclcpp::Node, public JetCatP300
{
private:
	// ROS variables
	rclcpp::Publisher<actuators::msg::ActuatorJCP300Info>::SharedPtr info_publisher_;
	rclcpp::Publisher<actuators::msg::ActuatorJCP300EngineTelemetry>::SharedPtr engine_telemetry_publisher_;
	rclcpp::Publisher<actuators::msg::ActuatorJCP300FuelTelemetry>::SharedPtr fuel_telemetry_publisher_;
	rclcpp::Service<actuators::srv::ActuatorJCP300Thrust>::SharedPtr thrust_service_;
	rclcpp::Service<actuators::srv::ActuatorJCP300Thrust2>::SharedPtr thrust_service_2;
	rclcpp::Publisher<actuators::msg::ActuatorJCP300SystemStatus>::SharedPtr system_status_publisher_;
	rclcpp::Service<actuators::srv::ActuatorJCP300Params>::SharedPtr params_service_;
	rclcpp::Service<actuators::srv::ActuatorJCP300HealthCheck>::SharedPtr healthcheck_service_;
	rclcpp::Service<actuators::srv::ActuatorJCP300Status>::SharedPtr status_service_;
	rclcpp::TimerBase::SharedPtr timer_[3];

	// Local variables
	bool ctrl_sig = false;
	bool pwr_sig = false;
	bool engine_started = false;
	float current_thrust = 0.0;
	int current_thrust_2 = 0;
	const string ENGINE_DISABLED = "Please enable engine, before proceeding.";

public:
	JetCatP300Manager(string port, int baudrate) : Node("JetCatP300"), JetCatP300(port, baudrate)
	{
		// Create Publishers
		this->info_publisher_ = this->create_publisher<actuators::msg::ActuatorJCP300Info>("info", 10);
		this->engine_telemetry_publisher_ = this->create_publisher<actuators::msg::ActuatorJCP300EngineTelemetry>("engine_telemetry", 10);
		this->fuel_telemetry_publisher_ = this->create_publisher<actuators::msg::ActuatorJCP300FuelTelemetry>("fuel_telemetry", 10);
		this->system_status_publisher_ = this->create_publisher<actuators::msg::ActuatorJCP300SystemStatus>("system_status", 10);

		//this->timer_[1] = this->create_wall_timer(7s, std::bind(&JetCatP300Manager::GetEngineInfo, this));
		this->timer_[0] = this->create_wall_timer(100ms, std::bind(&JetCatP300Manager::GetEngineTelemetry, this));
		this->timer_[1] = this->create_wall_timer(150ms, std::bind(&JetCatP300Manager::GetFuelTelemetry, this));
		this->timer_[2] = this->create_wall_timer(125ms, std::bind(&JetCatP300Manager::GetSystemStatus, this));


		// Create Services
		this->thrust_service_ = this->create_service<actuators::srv::ActuatorJCP300Thrust>("thrust", [this](const std::shared_ptr<actuators::srv::ActuatorJCP300Thrust::Request> request, std::shared_ptr<actuators::srv::ActuatorJCP300Thrust::Response> response) -> void {
			if(this->ctrl_sig || !this->pwr_sig)
			{
				response->status = ENGINE_DISABLED;
				return;
			}
			float new_value = request->thrust_value;
			if (new_value < 0.0)
			{
				new_value = 0.0;
			}
			if (new_value > 100.0)
			{
				new_value = 100.0;
			}
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Updating engine thrust.");
			//RS232 rs232_response = this->execute(RS232{1, "TRR", 1, to_string(int(new_value))});
			RS232 rs232_response = this->execute(RS232{1, "WPE", 1, to_string(new_value)});
			if(rs232_response.CMDCODE != "OK")
			{
				response->status = rs232_response.CMDCODE;
				return;
			}
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Updating thrust from %.4f to %.4f.", this->current_thrust, new_value);
			this->current_thrust = new_value;
			response->status = rs232_response.CMDCODE;
		});

		this->thrust_service_2 = this->create_service<actuators::srv::ActuatorJCP300Thrust2>("thrust", [this](const std::shared_ptr<actuators::srv::ActuatorJCP300Thrust2::Request> request, std::shared_ptr<actuators::srv::ActuatorJCP300Thrust2::Response> response) -> void {
			if(this->ctrl_sig || !this->pwr_sig)
			{
				response->status = ENGINE_DISABLED;
				return;
			}
			int new_value = request->thrust_value;
			if (new_value < 0)
			{
				new_value = 0;
			}
			if (new_value > 100)
			{
				new_value = 100;
			}
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Updating engine thrust.");
			RS232 rs232_response = this->execute(RS232{1, "WPE", 1, to_string(new_value)});
			if(rs232_response.CMDCODE != "OK")
			{
				response->status = rs232_response.CMDCODE;
				return;
			}
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Updating thrust from %.4f to %.4f.", this->current_thrust_2, new_value);
			this->current_thrust_2 = new_value;
			response->status = rs232_response.CMDCODE;
		});

		this->params_service_ = this->create_service<actuators::srv::ActuatorJCP300Params>("parameters", [this](const std::shared_ptr<actuators::srv::ActuatorJCP300Params::Request> request, std::shared_ptr<actuators::srv::ActuatorJCP300Params::Response> response) -> void {
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Jet engine, updating parameters %d %d.", request->ctrl_sig, request->pwr_sig);
			response->status = "Not OK";
			if (this->ctrl_sig != request->ctrl_sig)
			{
				this->ctrl_sig = request->ctrl_sig;
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Digital Write started ctrl");
				//wiringPiSetupGpio();
				//pinMode(JETCAT_SAFE, OUTPUT);
				digitalWrite(JETCAT_SAFE, this->ctrl_sig);
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Digital Write completed");
				response->status = "OK";
			}
			if (this->pwr_sig != request->pwr_sig)
			{
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Digital Write started pwr");
				//wiringPiSetupGpio();
				//pinMode(JETCAT_POWER, OUTPUT);
				digitalWrite(JETCAT_POWER, request->pwr_sig);
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Digital Write completed");
				response->status = "OK";
				this->pwr_sig = request->pwr_sig;
			}
			if(!this->ctrl_sig && this->pwr_sig)
			{
				//GetEngineInfo();
				//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
				//this->timer_[0] = this->create_wall_timer(100ms, std::bind(&JetCatP300Manager::GetEngineTelemetry, this));
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Jet engine ECU started.");
			}
		});

		this->healthcheck_service_ = this->create_service<actuators::srv::ActuatorJCP300HealthCheck>("health_check", [this](const std::shared_ptr<actuators::srv::ActuatorJCP300HealthCheck::Request> request, std::shared_ptr<actuators::srv::ActuatorJCP300HealthCheck::Response> response) -> void {
			if(this->ctrl_sig || !this->pwr_sig)
			{
				response->status_message = ENGINE_DISABLED;
				return;
			}
			if(!request->check_health)
			{
				return;
			}
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initializing engine health check.");
			RS232 rs232_response = this->execute(RS232{1, "DHC", 0, "1"});
			if(rs232_response.CMDCODE != "OK")
			{
				response->status_message = "Health Check failed, " + rs232_response.CMDCODE;
				return;
			}
			// Wait for 15 seconds.
			std::this_thread::sleep_for(std::chrono::milliseconds(15000));
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reading engine health check response.");
			rs232_response = this->execute(RS232{1, "RHC", 0, "1"});
			if(rs232_response.CMDCODE != "OK")
			{
				response->status_message = "Health Check Response failed, " + rs232_response.CMDCODE;
				return;
			}
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
					msg += "|";
				}
			}
			else
			{
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Read Health check. Not enough parameters in response %d.", rs232_response.len);
			}
			response->status_message = msg;
		});

		this->status_service_ = this->create_service<actuators::srv::ActuatorJCP300Status>("engine_status", [this](const std::shared_ptr<actuators::srv::ActuatorJCP300Status::Request> request, std::shared_ptr<actuators::srv::ActuatorJCP300Status::Response> response) -> void {
			if(this->ctrl_sig || !this->pwr_sig)
			{
				response->status = ENGINE_DISABLED;
				return;
			}
			response->status = false;
			if (this->engine_started != request->state)
			{
				if(request->state)
				{
					RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting engine.");
					RS232 rs232_response = this->execute(RS232{1, "TCO", 1, "1"});
					if(rs232_response.CMDCODE != "OK")
					{
						RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting engine failed, %s", rs232_response.CMDCODE);
						return;
					}
					response->status = "Engine Started " + rs232_response.CMDCODE;
				}
				else
				{
					RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stopping engine.");
					RS232 rs232_response = this->execute(RS232{1, "TCO", 1, "0"});
					if(rs232_response.CMDCODE != "OK")
					{
						RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stopping engine failed, %s", rs232_response.CMDCODE);
						return;
					}
					response->status = "Engine Stopped " + rs232_response.CMDCODE;
				}
				this->engine_started = request->state;
			}
		});

		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Jet engine services initialized.");
		//this->GetEngineInfo();

		// Start the ECU
		pinMode(JETCAT_SAFE, OUTPUT);
		pinMode(JETCAT_POWER, OUTPUT);
		this->ctrl_sig = false;
		this->pwr_sig = true;
		digitalWrite(JETCAT_SAFE, this->ctrl_sig);
		digitalWrite(JETCAT_POWER, this->pwr_sig);
		
		// digitalWrite(JETCAT_SAFE, LOW);
		// digitalWrite(JETCAT_POWER, HIGH);
		// RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Jet engine ECU started.");

	}
	void GetEngineInfo()
	{
		if(this->ctrl_sig || !this->pwr_sig)
		{
			//response->status_message = ENGINE_DISABLED;
			return;
		}
		auto message = actuators::msg::ActuatorJCP300Info();
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Fetching Jet Engine information.");
		RS232 response = this->execute(RS232{1, "RTY", 1, "1"});
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Info Received data");
		if (response.len < 5)
		{
			return;
		}
		message.firmware_version = response.params[0];
		message.version_number = response.params[1];
		message.last_time_run = atoi(response.params[2].c_str());
		message.total_operation_time = atoi(response.params[3].c_str());
		message.serial_number = response.params[4];
		message.turbine_type = response.params[5];
		// Publish
		this->info_publisher_->publish(message);
	}
	void GetTelemetry()
	{
		this->GetEngineTelemetry();
		this->GetFuelTelemetry();
	}
	void GetFuelTelemetry()
	{
		if(this->ctrl_sig || !this->pwr_sig)
		{
			//response->status_message = ENGINE_DISABLED;
			return;
		}
		auto message = actuators::msg::ActuatorJCP300FuelTelemetry();
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Getting fuel telemetry.");
		RS232 response = this->execute(RS232{1, "RFI", 1, "1"});
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Fuel telemetry received.");
		if (response.len < 5)
		{
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Fuel telemetry, response format invalid %d %s.", response.len, response.CMDCODE.c_str());
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
		this->fuel_telemetry_publisher_->publish(message);
	}
	void GetEngineTelemetry()
	{
		if(this->ctrl_sig || !this->pwr_sig)
		{
			//response->status_message = ENGINE_DISABLED;
			return;
		}
		auto message = actuators::msg::ActuatorJCP300EngineTelemetry();
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Getting engine telemetry.");
		RS232 response = this->execute(RS232{1, "RAC", 1, "1"});
		// this->send_command(RS232{1, "RAC", 1, "1"});
		// // The engine sends 2 packets : 1. command send 2. response
		// RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for part 1.");
		// while(!this->IsAvailable())
		// {
		// 	std::this_thread::sleep_for(std::chrono::milliseconds(5));
		// }
		// auto resp = this->Recv();
		// RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for part 2.");
		// while(!this->IsAvailable())
		// {
		// 	std::this_thread::sleep_for(std::chrono::milliseconds(5));
		// }
		// RS232 response = this->receive_response();
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Engine telemetry received.");
		if (response.len < 5)
		{
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Engine telemetry, response format invalid %d %s.", response.len, response.CMDCODE.c_str());
			printf("Response does not follow the format.\n");
			return;
		}
		message.turbine_rpm = atoi(response.params[0].c_str());
		message.egt_temp = atoi(response.params[1].c_str());
		message.pump_voltage = atof(response.params[2].c_str());
		message.turbine_state = atoi(response.params[3].c_str());
		message.throttle_position = atof(response.params[4].c_str());
		message.engine_current = atof(response.params[5].c_str());
		// Publish
		this->engine_telemetry_publisher_->publish(message);
	}
	void GetSystemStatus()
	{
		if(this->ctrl_sig || !this->pwr_sig)
		{
			//response->status_message = ENGINE_DISABLED;
			return;
		}
		auto message = actuators::msg::ActuatorJCP300SystemStatus();
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Getting fuel telemetry.");
		RS232 response = this->execute(RS232{1, "RSS", 1, "1"});
		//RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Fuel telemetry received.");
		if (response.len < 5)
		{
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Engine system status, response format invalid %d %s.", response.len, response.CMDCODE.c_str());
			printf("Response does not follow the format.\n");
			return;
		}
		message.off_condition = atoi(response.params[1].c_str());
		message.flight_speed = atoi(response.params[2].c_str());
		// Publish
		this->system_status_publisher_->publish(message);
	}
};

int main(int argc, char *argv[])
{
	wiringPiSetupGpio();
	rclcpp::init(argc, argv);
	if (argc < 3)
	{
		printf("Pass port and baudrate as parameters");
		return -1;
	}
	rclcpp::spin(std::make_shared<JetCatP300Manager>(string(argv[1]), atoi(argv[2])));
	rclcpp::shutdown();
	return 0;
}