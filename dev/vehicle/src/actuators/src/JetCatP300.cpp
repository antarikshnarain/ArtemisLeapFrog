/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Library to control JetCat P300 engine
----------------------------------------------------------------- */

#include "actuators/JetCatP300.hpp"

using namespace std;

JetCatP300::JetCatP300(string port, int baudrate) : Serial(port, baudrate, 13, 100, -1)
{
	printf("Intialized JetCatP300!\n");
	std::this_thread::sleep_for(std::chrono::milliseconds(1500));
}

bool JetCatP300::send_command(RS232 data)
{
	string command = to_string(data.ADR) + "," + data.CMDCODE;
	if (data.len)
	{
		for (int i = 0; i < data.len; i++)
		{
			command += "," + data.params[i];
		}
	}
	else
	{
		// Dummy parameter
		command += ",1";
	}
	printf("Sending Command to engine: %s\n", command.c_str());
	command += this->CR;
	return this->Send(command);
}

RS232 JetCatP300::receive_response()
{
	return this->read_response(this->convert_to_string(this->Recv()));
}

vector<string> JetCatP300::split(string value, string delim)
{
	vector<string> data;
	size_t pos = 0;
	string token;
	while ((pos = value.find(delim)) != string::npos)
	{
		token = value.substr(0, pos);
		data.push_back(token);
		value.erase(0, pos + delim.length());
	}
	return data;
}

RS232 JetCatP300::read_response(string response)
{
	RS232 data;
	if (response == "")
	{
		return data;
	}
	vector<string> values = split(response, ",");
	data.ADR = atoi(values[0].c_str());
	memcpy(data.CMDCODE, values[2].c_str(), sizeof(char) * 3);
	data.len = 0;
	for (auto i = 3; i < (int)values.size(); i++)
	{
		data.params[data.len] = values[i];
		data.len++;
	}
	return data;
}