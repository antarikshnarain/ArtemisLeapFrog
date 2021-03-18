/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Main program for Vehicle Flight software
----------------------------------------------------------------- */

// User Defined Libs
//#include "GroundStation.hpp"
#include "Communication.hpp"
#include "Properties.hpp"

// Boost Libs
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

// STL Libs
#include <iostream>
#include <functional>
#include <stdio.h>
#include <string>
#include <string.h>
#include <mpi.h>
#include <map>
#include <sstream>

using namespace std;
using namespace Utilities;

typedef void strfunc_t (const string&);
typedef map<string,strfunc_t*> strfuncmap_t; 

bool recv_flag = true;

Properties vehicle_prop;

struct mapper
{
    std::string print_string;
    std::function func_ptr;
};

map<std::string, std::string> keyMapping = {
    {"e","Setting Engine thrust to %.2f%%\n"},
    {"S e","Starting Engine\n"},
    {"S a","Starting ACS System\n"},
    {"S g","Starting Gimbal System\n"},
    {"s e","Starting Engine\n"},
    {"s a","Starting ACS System\n"},
    {"s g","Starting Gimbal System\n"},
    {"q","Stopping Engine\n"},
    {"h","Hovering at an altitude of %.2fm\n"},
    {"l","Initializing Landing sequence\n"},
    {"t","Requesting Telemetry data"}
};

// map<std::string, std::function> funcMapping = {
//     {""}
// }

void Sender(Communication comm)
{
    strfuncmap_t command_map;
    char data[100];
    string send_data;
    float value;
    while(1)
    {
        cout << "\n>";
        cin.getline(data, sizeof(data));
        send_data = string(data);
        if(send_data == "exit")
        {
            recv_flag = false;
            break;
        }
        else
        {
            if(keyMapping.find(string(data)) != keyMapping.end())
            {
                printf(keyMapping[data].c_str());
                comm.SendSerial(send_data);
            }
            else
            {
                string token = strtok(data, " ");

                if(keyMapping.find(token) != keyMapping.end())
                {
                    printf(keyMapping[token].c_str(), atof(strtok(NULL, " ")));
                    comm.SendSerial(send_data);
                }
                else
                {
                    cout << "Invalid Command " << token;
                }
            }
        }
        //sscanf(data.c_str(), format, &Mode, values[0],values[1],values[2]);
    }

}

void Receiver(Communication comm)
{
    while(recv_flag)
    {
        string recv = comm.RecvSerial();
        while(recv.length() == 0)
        {
            //printf("Waiting %s\n", recv.c_str());
            recv = comm.RecvSerial();
        }
        cout << "Received: " << recv << endl;
        // Process received data
        {
            std::istringstream iss(recv);
            boost::archive::text_iarchive ia(iss);
            ia >> vehicle_prop;
            vehicle_prop.print();
        }
    }
}

int main(int argc, char *argv[])
{
    int process_Rank, size_Of_Cluster, message_Item;
    MPI_Init(&argc, &argv);
    MPI_Comm_size(MPI_COMM_WORLD, &size_Of_Cluster);
    MPI_Comm_rank(MPI_COMM_WORLD, &process_Rank);

    if(argc != 3)
    {
        printf("Pass Communication port name and baud rate as parameter.\n");
        return -1;
    }
    
    Communication comm(string(argv[1]), atoi(argv[2]));
   
    if(process_Rank == 0)
    {
        Sender(comm);
    }
    else
    {
        Receiver(comm);
    }
    MPI_Barrier(MPI_COMM_WORLD);
    MPI_Finalize();
    return 0;
}