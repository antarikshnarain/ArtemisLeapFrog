/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Library to control JetCat P300 engine
----------------------------------------------------------------- */

#include "JetCatP300.hpp"

using namespace std;
using namespace Utilities;

namespace Actuators
{
	JetCatP300::JetCatP300(string communication_port, int baudrate):Serial(communication_port, baudrate)
	{
		this->comm_port = communication_port;
		this->BAUD = baudrate;
	}
	void JetCatP300::Initialize()
	{
		// Enable Control Pin
		this->CTRL_SIG = false;
		// Power On the engine
		this->PWR_SIG = true;
		this->SetEngineMode();

		// Initialize Communication
		this->serial = Serial(this->comm_port, this->BAUD);

		fprintf(stdout, "Engine is setup and ready to roll!");
	}

	void JetCatP300::Start()
	{
		if (!this->send_command(RS232{1, "TCO", 1, "1"}))
		{
			fprintf(stderr, "EngineStart(): failed!");
		}
		this->ENGINE_STATE = true;
	}

	void JetCatP300::Stop()
	{
		if (!this->send_command(RS232{1, "TCO", 1, "0"}))
		{
			fprintf(stderr, "EngineStop(): failed!");
			// TODO: Add failsafe, stop power to ECU
		}
		this->ENGINE_STATE = false;
	}

	bool JetCatP300::SetEngineThrust(float percentage)
	{
		this->THRUST_PER = percentage;
		// Send Command
		if (!this->send_command(RS232{1, "TRR", 1, to_string(this->THRUST_PER)}))
		{
			fprintf(stderr, "SetEngineThrust(): failed!");
			return false;
		}
		return true;
	}

	bool JetCatP300::CheckHealth()
	{
		printf("Initializing Check Health!\n");
		if (!this->send_command(RS232{1, "DHC", 0}))
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
		if (!this->send_command(RS232{1, "RHC", 0}))
		{
			fprintf(stderr, "CheckHealth():RHC failed!");
		}

		RS232 response = this->receive_response();
		// Logic to process flags
		const string health_params[7] = {"Starter", "Main Valve", "Starter Valve", "RPM Sensor", "Pump", "GlowPlug", "EGT Sensor"};
		if (response.len == 7)
		{
			// Verify Bits
			for (int i = 0; i < response.len; i++)
			{
				printf("Testing %s\r", health_params[i].c_str());
				if (atoi(response.params[i].c_str()) & 0x01)
				{
					printf("%s ok!", health_params[i].c_str());
				}
				if (atoi(response.params[i].c_str()) & 0x02)
				{
					printf("%s Driver Error!", health_params[i].c_str());
				}
				if (atoi(response.params[i].c_str()) & 0x04)
				{
					printf("%s No Current!", health_params[i].c_str());
				}
				if (atoi(response.params[i].c_str()) & 0x08)
				{
					printf("%s Engine Failure, refer manual!", health_params[i].c_str());
					return false;
				}
			}
		}
		return true;
	}

	void JetCatP300::GetTelemetry()
	{
		printf("Getting Engine Telemetry!\n");
		if(!this->send_command(RS232{1,"RFI",1,"1"}))
		{
			fprintf(stderr, "GetTelemetry(): falied!\n");
		}
		RS232 response = this->receive_response();
		this->prop_actual_fuel = atoi(response.params[0].c_str());
		this->prop_rest_fuel = atoi(response.params[1].c_str());
		this->prop_rpm = atoi(response.params[2].c_str());
		this->prop_batt_volt = atof(response.params[3].c_str());
		this->prop_last_run = atoi(response.params[4].c_str());
		this->prop_fuel_actual_run = atoi(response.params[5].c_str());

	}

	void JetCatP300::EngineInformation()
	{
		printf("Fetching Jet Engine information!\n");
		if(!this->send_command(RS232{1,"RTY",1,"1"}))
		{
			fprintf(stderr, "EngineInformation(): failed!\n");
		}
		RS232 response = this->receive_response();
		if(response.len < 6)
		{
			return;
		}
		this->_firmware_version = response.params[0];
		this->_version_num = response.params[1];
		this->_last_time_run = response.params[2];
		this->_total_operation_time = response.params[3];
		this->_serial_no = response.params[4];
		this->_turbine_type = response.params[5];
	}

	void JetCatP300::SetEngineMode()
	{
		digitalWrite(JETCAT_SAFE, this->CTRL_SIG);
		digitalWrite(JETCAT_POWER, this->PWR_SIG);
	}

} // namespace Actuators