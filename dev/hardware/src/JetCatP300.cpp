/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Library to control JetCat P300 engine
----------------------------------------------------------------- */

#include "../include/JetCatP300.hpp"

using namespace std;
using namespace Utilities;

//using Actuators::JetCatP300;

//class JetCatP300
//{

	void digitalWrite(int pin,bool value)
	{

	}
	Actuators::JetCatP300::JetCatP300(string communication_port, int baudrate)
	{
		this->comm_port = communication_port;
		this->BAUD = baudrate;
	}
	void Actuators::JetCatP300::Initialize()
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

	void Actuators::JetCatP300::Start()
	{
		if (!this->send_command(RS232{1, "TCO", 1, "1"}))
		{
			fprintf(stderr, "EngineStart(): failed!");
		}
		this->ENGINE_STATE = true;
	}

	void Actuators::JetCatP300::Stop()
	{
		if (!this->send_command(RS232{1, "TCO", 1, "0"}))
		{
			fprintf(stderr, "EngineStop(): failed!");
			// TODO: Add failsafe, stop power to ECU
		}
		this->ENGINE_STATE = false;
	}

	bool Actuators::JetCatP300::SetEngineThrust(float percentage)
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

	bool Actuators::JetCatP300::CheckHealth()
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

	void Actuators::JetCatP300::GetTelemetry()
	{
	}

	void Actuators::JetCatP300::SetEngineMode()
	{
		digitalWrite(JETCAT_SAFE, this->CTRL_SIG);
		digitalWrite(JETCAT_POWER, this->PWR_SIG);
	}
//};
//} // namespace Actuators