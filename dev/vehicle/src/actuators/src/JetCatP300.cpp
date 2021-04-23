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
	//command += this->CR;
	return this->Send(command);
}

RS232 JetCatP300::execute(RS232 data)
{
	this->_mutex.lock();
	this->send_command(data);
	// The engine sends 2 packets : 1. command send 2. response
	while(this->IsAvailable() < 2)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
	this->Recv();
	RS232 resp = this->receive_response();
	this->_mutex.unlock();
	return resp;
}

RS232 JetCatP300::receive_response()
{
	//auto sent_command = this->Recv();
	return this->read_response(this->convert_to_string(this->Recv()));
}

vector<string> JetCatP300::split(string value, char delim)
{
	vector<string> data;
	stringstream check(value);
    string temp;
    while (getline(check, temp, delim))
    {
        data.push_back(temp);
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
	printf("Received from engine: %s\n", response.c_str());
	vector<string> values = split(response, ',');
	data.ADR = atoi(values[0].c_str());
	data.CMDCODE = values[2];
	//memcpy(data.CMDCODE, values[2].c_str(), sizeof(char) * 3);
	data.len = 0;
	for (auto i = 3; i < (int)values.size(); i++)
	{
		data.params[data.len] = values[i];
		data.len++;
	}
	return data;
}