#include "flightcontrol/flightmanager.hpp"

FlightManager::FlightManager(string port, int baudrate, std::future<void> fut) : Node("FlightManager"), Serial(port, baudrate, '\n', 1000, -1)
{
	this->InitializeSequence();
	thread(&FlightManager::SerialMonitor, this, move(fut)).detach();
	// Send ready message
	this->Send(string("Vehicle is Ready!"));
}

void FlightManager::SerialMonitor(std::future<void> fut)
{
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Started Serial Monitor.");
	int heartbeat_counter = HEARTBEAT_DURATION;
    while (fut.wait_for(chrono::milliseconds(200)) == std::future_status::timeout && heartbeat_counter > 0)
    {
        if(this->IsAvailable())
        {
            auto recv = this->Recv();
            string data = this->convert_to_string(recv);
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Command Received: %s", data.c_str());
            if(data == "exit")
            {
                break;
            }
			if(this->enable_echo)
			{
	            this->Send(recv);
				this->enable_echo = false;
			}
			else
			{
				this->Send(this->Parser(data));
			}
			heartbeat_counter = HEARTBEAT_DURATION;
        }
		else
		{
			heartbeat_counter --;
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting %d", heartbeat_counter);
		}
        //this_thread::sleep_for(chrono::milliseconds(200));
    }
	this->ShutdownSequence();
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stopped Serial Monitor. %d", heartbeat_counter);
}

void FlightManager::InitializeSequence()
{
	// Initialize client services
	this->thrust_client_ = this->create_client<actuators::srv::ActuatorJCP300Thrust>("/actuators/thrust");
	while (!this->thrust_client_->wait_for_service(1s))
	{
		if (!rclcpp::ok())
		{
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
			return;
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JCP300-Thrust service not available, waiting again...");
	}
	this->params_client_ = this->create_client<actuators::srv::ActuatorJCP300Params>("/actuators/parameters");
	while (!this->params_client_->wait_for_service(1s))
	{
		if (!rclcpp::ok())
		{
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
			return;
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JCP300-Parameter service not available, waiting again...");
	}
	this->healthcheck_client_ = this->create_client<actuators::srv::ActuatorJCP300HealthCheck>("/actuators/health_check");
	while (!this->healthcheck_client_->wait_for_service(1s))
	{
		if (!rclcpp::ok())
		{
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
			return;
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JCP300-HealthCheck service not available, waiting again...");
	}
	this->status_client_ = this->create_client<actuators::srv::ActuatorJCP300Status>("/actuators/engine_status");
	while (!this->status_client_->wait_for_service(1s))
	{
		if (!rclcpp::ok())
		{
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
			return;
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JCP300-Status service not available, waiting again...");
	}
	this->coldgas_client_ = this->create_client<actuators::srv::ActuatorColdGasFireThruster>("/actuators/cold_gas");
	while (!this->coldgas_client_->wait_for_service(1s))
	{
		if (!rclcpp::ok())
		{
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
			return;
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ACS-Thruster service not available, waiting again...");
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialized Clients");
	// Initialize Subscribers
	// this->info_subscriber_ = this->create_subscription<actuators::msg::ActuatorJCP300Info>("JCP300-Info", )
	// this->telemetry_subscriber_
	// this->imu_subscriber_
	// this->sensor_subscriber_
	this->info_subscriber_ = this->create_subscription<actuators::msg::ActuatorJCP300Info>("/actuators/info", 10, [this](const actuators::msg::ActuatorJCP300Info::SharedPtr msg) -> void {
		char buffer[256];
		if(sprintf(buffer, "%s,%s,%d,%d,%s,%s", msg->firmware_version.c_str(), msg->version_number.c_str(), msg->last_time_run, msg->total_operation_time, msg->serial_number.c_str(), msg->turbine_type.c_str()) > 0)
		{
			this->sub_info = std::string(buffer);
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JCP300-Info:%s,%s,%d,%d,%s,%s",
			msg->firmware_version.c_str(), msg->version_number.c_str(), msg->last_time_run, msg->total_operation_time, msg->serial_number.c_str(), msg->turbine_type.c_str());
		
	});

	this->telemetry_subscriber_ = this->create_subscription<actuators::msg::ActuatorJCP300Telemetry>("/actuators/telemetry", 10, [this](const actuators::msg::ActuatorJCP300Telemetry::SharedPtr msg) -> void {
		char buffer[256];
		if(sprintf(buffer, "%d,%d,%d,%.4f,%d,%d", msg->actual_fuel, msg->rest_fuel, msg->rpm, msg->battery_voltage, msg->last_run, msg->fuel_actual_run) > 0)
		{
			this->sub_telemetry = std::string(buffer);
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JCP300-Telem:%d,%d,%d,%.4f,%d,%d",
			msg->actual_fuel, msg->rest_fuel, msg->rpm, msg->battery_voltage, msg->last_run, msg->fuel_actual_run);
	});
	this->imu_subscriber_ = this->create_subscription<sensors::msg::SensorImu>("/sensors/imu_data", 10, [this](const sensors::msg::SensorImu::SharedPtr msg) -> void {
		char buffer[512];
		if(sprintf(buffer, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f", 
			msg->raw_linear_acc[0], msg->raw_linear_acc[1], msg->raw_linear_acc[2], msg->raw_angular_acc[0], msg->raw_angular_acc[1], msg->raw_angular_acc[2],
			msg->roll, msg->pitch, msg->temp, msg->linear_acc[0], msg->linear_acc[1], msg->linear_acc[2], msg->angular_acc[0], msg->angular_acc[1], msg->angular_acc[2]) > 0)
		{
			this->sub_imu = std::string(buffer);
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sensors-IMU:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
					msg->raw_linear_acc[0], msg->raw_linear_acc[1], msg->raw_linear_acc[2], msg->raw_angular_acc[0], msg->raw_angular_acc[1], msg->raw_angular_acc[2],
					msg->roll, msg->pitch, msg->temp, msg->linear_acc[0], msg->linear_acc[1], msg->linear_acc[2], msg->angular_acc[0], msg->angular_acc[1], msg->angular_acc[2]);
	});
	this->laser_subscriber_ = this->create_subscription<sensors::msg::SensorLaser>("/sensors/laser", 10, [this](const sensors::msg::SensorLaser::SharedPtr msg) -> void {
		char buffer[128];
		if(sprintf(buffer, "%d,%d,%d", msg->distance, msg->sig_strength, msg->checksum) > 0)
		{
			this->sub_laser = std::string(buffer);
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sensors-Laser:%d,%d,%d", msg->distance, msg->sig_strength, msg->checksum);
	});
}

void FlightManager::ShutdownSequence()
{
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutdown Sequence: Started");
	// Power off engine
	this->engine_power(0);
	// Disable sensor data
	this->sensor_enable(0);
	// Disable ACS system
	this->acs_enable(0);
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutdown Sequence: Complete");
}
string FlightManager::engine_ctrl(int value)
{
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Engine control updated %d", value);
	bool ctrl_val = (bool)value;
	this->enable_engine = !ctrl_val;
	auto request = std::make_shared<actuators::srv::ActuatorJCP300Params::Request>();
	request->ctrl_sig = ctrl_val;
	request->pwr_sig = false;
	auto result = this->params_client_->async_send_request(request);
	if (result.wait_for(15s) == std::future_status::ready)
	{
		//Use the result
		return "OK";
	}
	else
	{
		//Something went wrong
		return "Something went wrong!, engine_ctrl";
	}
}
string FlightManager::engine_power(int value)
{
	if (!this->enable_engine)
	{
		return "Please enable engine before proceeding...";
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Engine power updated %d", value);
	bool pwr_val = (bool)value;
	auto request = std::make_shared<actuators::srv::ActuatorJCP300Params::Request>();
	request->ctrl_sig = true;
	request->pwr_sig = pwr_val;
	auto result = this->params_client_->async_send_request(request);
	if (result.wait_for(15s) == std::future_status::ready)
	{
		//Use the result
		return "OK";
	}
	else
	{
		//Something went wrong
		return "Something went wrong!, engine_power";
	}
}
string FlightManager::engine_enable(int value)
{
	if (!this->enable_engine)
	{
		return "Please enable engine before proceeding...";
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Engine state updated %d", value);
	auto request = std::make_shared<actuators::srv::ActuatorJCP300Status::Request>();
	request->state = (bool)value;
	auto result = this->status_client_->async_send_request(request);
	if (result.wait_for(15s) == std::future_status::ready)
	{
		//Use the result
		return result.get()->status ? "OK" : "Not OK";
	}
	else
	{
		//Something went wrong
		return "Something went wrong!, engine_enable";
	}
}
string FlightManager::engine_telem_0()
{
	if (!this->enable_engine)
	{
		return "Please enable engine before proceeding...";
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Engine Health check requested.");
	auto request = std::make_shared<actuators::srv::ActuatorJCP300HealthCheck::Request>();
	request->check_health = true;
	auto result = this->healthcheck_client_->async_send_request(request);
	if (result.wait_for(15s) == std::future_status::ready)
	{
		//Use the result
		return result.get()->status_message;
	}
	else
	{
		//Something went wrong
		return "Something went wrong!, engine_telem_0";
	}
}
string FlightManager::engine_telem_1()
{
	if (!this->enable_engine)
	{
		return "Please enable engine before proceeding...";
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Engine Information requested.");
	return this->sub_info;
}
string FlightManager::engine_telem_2()
{
	if (!this->enable_engine)
	{
		return "Please enable engine before proceeding...";
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Engine Telemetry requested.");
	return this->sub_telemetry;
}
string FlightManager::engine_thrust(float value)
{
	if (!this->enable_engine)
	{
		return "Please enable engine before proceeding...";
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Engine thrust updated %.4f", value);
	auto request = std::make_shared<actuators::srv::ActuatorJCP300Thrust::Request>();
	request->thrust_value = value;
	auto result = this->thrust_client_->async_send_request(request);
	if (result.wait_for(15s) == std::future_status::ready)
	{
		//Use the result
		return result.get()->status ? "OK" : "Not OK";
	}
	else
	{
		//Something went wrong
		return "Something went wrong!, engine_thrust";
	}
}
string FlightManager::acs_enable(int value)
{
	this->enable_acs = (bool)value;
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ACS state updated %d", value);
	return "OK";
}
string FlightManager::acs_fire(float durations[6])
{
	if (!this->enable_acs)
	{
		return "Please enable ACS before proceeding...";
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ACS fire sequence requested %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", 
		durations[0],durations[1],durations[2],durations[3],durations[4],durations[5]);
	auto request = std::make_shared<actuators::srv::ActuatorColdGasFireThruster::Request>();
	for(int i=0;i<6;i++)
	{
		request->durations[i] = durations[i];
	}
	auto result = this->coldgas_client_->async_send_request(request);
	if (result.wait_for(15s) == std::future_status::ready)
	{
		//Use the result
		return result.get()->status ? "OK" : "Not OK";
	}
	else
	{
		//Something went wrong
		return "Something went wrong!, engine_thrust";
	}
}
string FlightManager::sensor_enable(int value)
{
	this->enable_sensors = (bool)value;
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sensor state updated %d", value);
	return "OK";
}
string FlightManager::sensor_telem_0()
{
	if (!this->enable_sensors)
	{
		return "Please enable Sensors before proceeding...";
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "IMU Data requested");
	return this->sub_imu;
}
string FlightManager::sensor_telem_1()
{
	if (!this->enable_sensors)
	{
		return "Please enable Sensors before proceeding...";
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Altitude Data requested");
	return this->sub_laser;
}
string FlightManager::sensor_telem_2()
{
	if (!this->enable_sensors)
	{
		return "Please enable Sensors before proceeding...";
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Battery Data requested");
	return "NOT IMPLEMENTED.";
}

string FlightManager::cmd_echo(int value)
{
	this->enable_echo = (bool)value;
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Command Echo state updated %d", value);
	return "OK";
}
string FlightManager::cmd_script(int value)
{
	this->enable_script = value;
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Command Script state updated %d", value);
	return "OK";
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	if (argc < 3)
	{
		printf("Pass port and baudrate as parameters");
		return -1;
	}
	promise<void> exit_signal;
	future<void> exit_future = exit_signal.get_future();
	rclcpp::spin(std::make_shared<FlightManager>(std::string(argv[1]), atoi(argv[2]), move(exit_future)));
	exit_signal.set_value();
	printf("Flight Manager: Sleeping for 5 seconds for things to shut down.");
	this_thread::sleep_for(chrono::seconds(5));
	printf("LEAPFROG Shutting down.");
	rclcpp::shutdown();
	return 0;
}