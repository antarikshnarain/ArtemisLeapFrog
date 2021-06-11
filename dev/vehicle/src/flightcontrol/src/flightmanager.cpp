#include "flightcontrol/flightmanager.hpp"

FlightManager::FlightManager(string port, int baudrate, std::future<void> fut) : Node("FlightManager"), Serial(port, baudrate, '\n', 1000, -1)
{
	// Load Script files
	string filename = this->path_to_files + this->custom_script_file;
	ifstream file;
	file.open(filename.c_str(), ios::in);
	string data;
	string script_names = "";
	int num = 1;
	if (file.is_open())
	{
		while (getline(file, data))
		{
			this->script_map.push_back(this->path_to_files + data);
			script_names += "| " + to_string(num++) + " -> " + this->path_to_files + data + " |";
		}
	}
	else
	{
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Script file not loaded.");
	}

	this->InitializeSequence();
	thread(&FlightManager::SerialMonitor, this, move(fut)).detach();
	// Send ready message
	string temp_msg = string("Vehicle is Ready!") + string(" Loaded Scripts ") + to_string(this->script_map.size()) + script_names;
	this->Send(temp_msg);
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", temp_msg.c_str());
}

int FlightManager::CopyLogs()
{
	int i;
	printf("Checking if processor is available...");
	// if (system(NULL)) puts ("Ok");
	// else exit (EXIT_FAILURE);
	printf("Executing command DIR...\n");
	// TODO: Remove hardcoded command.
	i = system("~/vehicle/copylogs.sh");
	printf("The value returned was: %d.\n", i);
	return i;
}

void FlightManager::SerialMonitor(std::future<void> fut)
{
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Started Serial Monitor.");
	int heartbeat_counter = HEARTBEAT_DURATION;
	while (fut.wait_for(chrono::milliseconds(200)) == std::future_status::timeout && heartbeat_counter > 0)
	{
		if (this->IsAvailable())
		{
			auto recv = this->Recv();
			string data = this->convert_to_string(recv);
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Command Received: %s", data.c_str());
			if (data == "exit")
			{
				break;
			}
			if (this->enable_echo)
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
			heartbeat_counter--;
			if (heartbeat_counter % 100 == 0)
			{
				RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting %d", heartbeat_counter);
			}
			if (heartbeat_counter == 500)
			{
				this->Send("Vehicle will shutdown in 2 minutes.");
			}
		}
		//this_thread::sleep_for(chrono::milliseconds(200));
	}
	int datacopied = this->CopyLogs();
	this->Send("Vehicle Shutting down !!! " + to_string(datacopied));
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
	this->thrust_client_2 = this->create_client<actuators::srv::ActuatorJCP300Thrust2>("/actuators/thrust");
	while (!this->thrust_client_2->wait_for_service(1s))
	{
		if (!rclcpp::ok())
		{
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
			return;
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JCP300-Thrust2 service not available, waiting again...");
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
	this->gimbal_client_ = this->create_client<actuators::srv::ActuatorMoveGimbal>("/actuators/move_gimbal");
	while (!this->gimbal_client_->wait_for_service(5s))
	{
		if (!rclcpp::ok())
		{
			RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
			return;
		}
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gimbal service not available, waiting again...");
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialized Clients");

	// Initialize Subscribers
	// -- Engine Subscribers
	this->info_subscriber_ = this->create_subscription<actuators::msg::ActuatorJCP300Info>("/actuators/info", 10, [this](const actuators::msg::ActuatorJCP300Info::SharedPtr msg) -> void
																						   {
																							   if (!this->enable_engine)
																							   {
																								   return;
																							   }
																							   char buffer[256];
																							   if (sprintf(buffer, "%s,%s,%d,%d,%s,%s", msg->firmware_version.c_str(), msg->version_number.c_str(), msg->last_time_run, msg->total_operation_time, msg->serial_number.c_str(), msg->turbine_type.c_str()) > 0)
																							   {
																								   this->sub_info = std::string(buffer);
																							   }
																							   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JCP300-Info:%s,%s,%d,%d,%s,%s",
																										   msg->firmware_version.c_str(), msg->version_number.c_str(), msg->last_time_run, msg->total_operation_time, msg->serial_number.c_str(), msg->turbine_type.c_str());
																						   });
	this->fuel_telemetry_subscriber_ = this->create_subscription<actuators::msg::ActuatorJCP300FuelTelemetry>("/actuators/fuel_telemetry", 10, [this](const actuators::msg::ActuatorJCP300FuelTelemetry::SharedPtr msg) -> void
																											  {
																												  if (!this->enable_engine)
																												  {
																													  return;
																												  }
																												  char buffer[256];
																												  if (sprintf(buffer, "%d,%d,%d,%.4f,%d,%d", msg->actual_fuel, msg->rest_fuel, msg->rpm, msg->battery_voltage, msg->last_run, msg->fuel_actual_run) > 0)
																												  {
																													  this->sub_fuel_telemetry = std::string(buffer);
																												  }
																												  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JCP300-Fuel-Telem:%d,%d,%d,%.4f,%d,%d",
																															  msg->actual_fuel, msg->rest_fuel, msg->rpm, msg->battery_voltage, msg->last_run, msg->fuel_actual_run);
																											  });
	this->engine_telemetry_subscriber_ = this->create_subscription<actuators::msg::ActuatorJCP300EngineTelemetry>("/actuators/engine_telemetry", 10, [this](const actuators::msg::ActuatorJCP300EngineTelemetry::SharedPtr msg) -> void
																												  {
																													  if (!this->enable_engine)
																													  {
																														  return;
																													  }
																													  char buffer[256];
																													  if (sprintf(buffer, "%d,%d,%.4f,%d,%.4f,%.4f", msg->turbine_rpm, msg->egt_temp, msg->pump_voltage, msg->turbine_state, msg->throttle_position, msg->engine_current) > 0)
																													  {
																														  this->sub_engine_telemetry = std::string(buffer);
																													  }
																													  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JCP300-Engine-Telem:%d,%d,%.4f,%d,%.4f,%.4f",
																																  msg->turbine_rpm, msg->egt_temp, msg->pump_voltage, msg->turbine_state, msg->throttle_position, msg->engine_current);
																												  });
	this->system_status_subscriber_ = this->create_subscription<actuators::msg::ActuatorJCP300SystemStatus>("/actuators/system_status", 10, [this](const actuators::msg::ActuatorJCP300SystemStatus::SharedPtr msg) -> void
																											{
																												if (!this->enable_engine)
																												{
																													return;
																												}
																												char buffer[256];
																												if (sprintf(buffer, "%d,%d", msg->off_condition, msg->flight_speed) > 0)
																												{
																													this->sub_system_status = std::string(buffer);
																												}
																												RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "JCP300-System-Status:%d,%d", msg->off_condition, msg->flight_speed);
																											});
	// --Sensors Subscribers
	this->imu_subscriber_ = this->create_subscription<sensors::msg::SensorImu>("/sensors/imu_data", 10, [this](const sensors::msg::SensorImu::SharedPtr msg) -> void
																			   {
																				   if (!this->enable_sensors)
																				   {
																					   return;
																				   }
																				   char buffer[512];
																				   if (sprintf(buffer, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
																							   msg->raw_linear_acc[0], msg->raw_linear_acc[1], msg->raw_linear_acc[2], msg->raw_angular_acc[0], msg->raw_angular_acc[1], msg->raw_angular_acc[2],
																							   msg->roll, msg->pitch, msg->temp, msg->linear_acc[0], msg->linear_acc[1], msg->linear_acc[2], msg->angular_acc[0], msg->angular_acc[1], msg->angular_acc[2]) > 0)
																				   {
																					   this->sub_imu = std::string(buffer);
																				   }
																				   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sensors-IMU:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
																							   msg->raw_linear_acc[0], msg->raw_linear_acc[1], msg->raw_linear_acc[2], msg->raw_angular_acc[0], msg->raw_angular_acc[1], msg->raw_angular_acc[2],
																							   msg->roll, msg->pitch, msg->temp, msg->linear_acc[0], msg->linear_acc[1], msg->linear_acc[2], msg->angular_acc[0], msg->angular_acc[1], msg->angular_acc[2]);
																			   });
	this->laser_subscriber_ = this->create_subscription<sensors::msg::SensorLaser>("/sensors/laser", 10, [this](const sensors::msg::SensorLaser::SharedPtr msg) -> void
																				   {
																					   if (!this->enable_sensors)
																					   {
																						   return;
																					   }
																					   char buffer[128];
																					   if (sprintf(buffer, "%d,%d,%d", msg->distance, msg->sig_strength, msg->checksum) > 0)
																					   {
																						   this->sub_laser = std::string(buffer);
																					   }
																					   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sensors-Laser:%d,%d,%d", msg->distance, msg->sig_strength, msg->checksum);
																				   });
	this->actuator_subscriber_ = this->create_subscription<sensors::msg::SensorLinearActuator>("/sensors/linear_position", 10, [this](const sensors::msg::SensorLinearActuator::SharedPtr msg) -> void
																				   {
																					   if (!this->enable_sensors)
																					   {
																						   return;
																					   }
																					   char buffer[128];
																					   if (sprintf(buffer, "%d,%d,%d", msg->position) > 0)
																					   {
																						   this->sub_actuator = std::string(buffer);
																					   }
																					   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sensors-Linear Actuator:%d", msg->position);
																				   });
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialized Subscriptions.");
}

void FlightManager::ShutdownSequence()
{
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutdown Sequence: Started");
	// Power off engine
	this->engine_power(0);
	this->enable_engine = false;
	this->enable_sensors = false;
	// Disable sensor data
	this->sensor_enable(0);
	// Disable ACS system
	this->enable_acs = false;
	this->acs_enable(0);
	// Disable Gimbal system
	this->enable_gimbal = false;
	this->gimbal_enable(0);

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutdown Sequence: Complete");
}

string FlightManager::label_run(string name)
{
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Labeling name: %d", name);
	return "OK";
}

string FlightManager::engine_ctrl(int value)
{
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Engine control updated %d", value);
	bool ctrl_val = (bool)value;
	this->enable_engine = ctrl_val;
	// auto request = std::make_shared<actuators::srv::ActuatorJCP300Params::Request>();
	// request->ctrl_sig = !ctrl_val;
	// request->pwr_sig = false;
	// auto result = this->params_client_->async_send_request(request);
	// if (result.wait_for(15s) == std::future_status::ready)
	// {
	// 	//Use the result
	// 	return "OK";
	// }
	// else
	// {
	// 	//Something went wrong
	// 	return "Something went wrong!, engine_ctrl";
	// }
	return "OK";
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
	request->ctrl_sig = false;
	request->pwr_sig = pwr_val;
	auto result = this->params_client_->async_send_request(request);
	if (result.wait_for(15s) == std::future_status::ready)
	{
		//Use the result
		return result.get()->status;
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
		return result.get()->status;
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
	if (result.wait_for(25s) == std::future_status::ready)
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
	return this->sub_engine_telemetry;
}
string FlightManager::engine_telem_3()
{
	if (!this->enable_engine)
	{
		return "Please enable engine before proceeding...";
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Fuel Telemetry requested.");
	return this->sub_fuel_telemetry;
}
string FlightManager::engine_telem_4()
{
	if (!this->enable_engine)
	{
		return "Please enable engine before proceeding...";
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Engine System Status requested.");
	return this->sub_system_status;
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
		return result.get()->status;
	}
	else
	{
		//Something went wrong
		return "Something went wrong!, engine_thrust";
	}
}
string FlightManager::engine_thrust2(int value)
{
	if (!this->enable_engine)
	{
		return "Please enable engine before proceeding...";
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Engine thrust updated %.4f", value);
	auto request = std::make_shared<actuators::srv::ActuatorJCP300Thrust2::Request>();
	request->thrust_value = value;
	auto result = this->thrust_client_2->async_send_request(request);
	if (result.wait_for(15s) == std::future_status::ready)
	{
		//Use the result
		return result.get()->status;
	}
	else
	{
		//Something went wrong
		return "Something went wrong!, engine_thrust2";
	}
}

string FlightManager::acs_enable(int value)
{
	this->enable_acs = (bool)value;
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ACS state updated %d", value);
	return "OK";
}
string FlightManager::acs_fire(int durations[6])
{
	if (!this->enable_acs)
	{
		return "Please enable ACS before proceeding...";
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ACS fire sequence requested %.2f, %.2f, %.2f, %.2f, %.2f, %.2f",
				durations[0], durations[1], durations[2], durations[3], durations[4], durations[5]);
	auto request = std::make_shared<actuators::srv::ActuatorColdGasFireThruster::Request>();
	for (int i = 0; i < 6; i++)
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
string FlightManager::sensor_telem_3()
{
	if (!this->enable_sensors)
	{
		return "Please enable Sensors before proceeding...";
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Linear Actuator Position requested");
	return this->sub_actuator;
}

string FlightManager::cmd_echo(int value)
{
	this->enable_echo = (bool)value;
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Command Echo state updated %d", value);
	return "OK";
}

string FlightManager::gimbal_enable(int value)
{
	this->enable_gimbal = (bool)value;
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gimbal state updated %d", value);
	return "OK";
}
string FlightManager::gimbal_move(float angles[2])
{
	if (!this->enable_gimbal)
	{
		return "Please enable Gimbal before proceeding...";
	}
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gimbal fire sequence requested %.2f, %.2f",
				angles[0], angles[1]);
	auto request = std::make_shared<actuators::srv::ActuatorMoveGimbal::Request>();
	for (int i = 0; i < 2; i++)
	{
		request->angles[i] = angles[i];
	}
	auto result = this->gimbal_client_->async_send_request(request);
	if (result.wait_for(15s) == std::future_status::ready)
	{
		//Use the result
		return result.get()->status ? "OK" : "Not OK";
	}
	else
	{
		//Something went wrong
		return "Something went wrong!, gimbal_move failed";
	}
}

void FlightManager::ScriptRunner(string filename, future<void> script_future)
{
	// load commands to memory
	ifstream file;
	file.open(filename.c_str(), ios::in);
	vector<string> commands;
	vector<int> cmd_delays;
	string data;
	if (file.is_open())
	{
		bool toggle = true;
		while (getline(file, data))
		{
			if (toggle)
			{
				commands.push_back(data);
			}
			else
			{
				cmd_delays.push_back(atoi(data.c_str()));
			}
			toggle = !toggle;
		}
	}
	else
	{
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Custom script %s does not exist.", filename.c_str());
		return;
	}
	if (commands.size() != cmd_delays.size() || commands.size() == 0)
	{
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Script is not valid. Commands: %d -> Delays: %d.", commands.size(), cmd_delays.size());
		return;
	}
	string notify = "Script Started: " + filename;
	this->Send(notify);
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Started custom script %s.", filename.c_str());
	int i = 0;
	do
	{
		string resp = this->Parser(commands[i]);
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executed %s with response: %s.", commands[i].c_str(), resp.c_str());
		this->Send(resp);
		//std::this_thread.sleep_for(std::chrono::milliseconds(cmd_delays[i]));
		i++;
	} while (i < (int)commands.size() && script_future.wait_for(chrono::milliseconds(cmd_delays[i])) == std::future_status::timeout);
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Completed custom script %s. %d / %d.", filename.c_str(), i, commands.size());
	notify = "Script completed: " + filename;
	this->Send(notify);
	this->enable_script = 0;
}

string FlightManager::cmd_script(int value)
{
	// Command to interrupt script
	if (value == 0)
	{
		this->enable_script = 0;
		if (exit_script_thread_promise != NULL)
		{
			exit_script_thread_promise->set_value();
			exit_script_thread_promise = NULL;
			return "OK - Stopping Running Scripts.";
		}
		return "OK - No script running.";
	}
	else
	{
		if (this->enable_script > 0)
		{
			return "Script already running.";
		}
		if (value < 0 || value > (int)this->script_map.size())
		{
			return "Invalid Script number " + to_string(value);
		}
		for (string files : this->script_map)
		{
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Available Scripts %s", files.c_str());
		}
		exit_script_thread_promise = new promise<void>();
		future<void> exit_script_thread_future2 = exit_script_thread_promise->get_future();
		thread(&FlightManager::ScriptRunner, this, this->script_map[value - 1], move(exit_script_thread_future2)).detach();
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting script %s for %d.", this->script_map[value - 1], value);
	}
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