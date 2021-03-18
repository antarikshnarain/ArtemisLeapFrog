/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Main program for Ground station
 * Provides a terminal like interface to send and recv data.
----------------------------------------------------------------- */

// User Defined Libs
//#include "GroundStation.hpp"
#include "Communication.hpp"
#include "Properties.hpp"
#include "CommandController.hpp"

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
#include <unordered_map>
#include <sstream>

using namespace std;
using namespace Utilities;

bool recv_flag = true;
// Global Command Controller - Properties
CommandController controller;
// Global Communication Manager - Communication
Communication *comm;

void Sender()
{
    char data[100];
    //string send_data;
    while (1)
    {
        cout << "\n>";
        cin.getline(data, sizeof(data));
        //send_data = string(data);
        if (data == "exit")
        {
            recv_flag = false;
            break;
        }
        else
        {
            if(!controller.DecodeCmd(data))
            {
                // 4. Send Data
                comm->SendSerial(controller.GetData());
            }
        }
    }
}

void Receiver()
{
    CommandController controller;
    while (recv_flag)
    {
        string recv = comm->RecvSerial();
        while (recv.length() == 0)
        {
            //printf("Waiting %s\n", recv.c_str());
            recv = comm->RecvSerial();
        }
        cout << "Received: " << recv << endl;
        // Process received data
        // 1. Deserialize data
        char *data_str = (char *)recv.c_str();
        controller.DecodeStream(data_str);
    }
}

int main(int argc, char *argv[])
{
    int process_Rank, size_Of_Cluster, message_Item;
    MPI_Init(&argc, &argv);
    MPI_Comm_size(MPI_COMM_WORLD, &size_Of_Cluster);
    MPI_Comm_rank(MPI_COMM_WORLD, &process_Rank);

    if (argc != 3)
    {
        printf("Pass Communication port name and baud rate as parameter.\n");
        return -1;
    }

    comm = new Communication(string(argv[1]), atoi(argv[2]));

    if (process_Rank == 0)
    {
        Sender();
    }
    else
    {
        Receiver();
    }
    MPI_Barrier(MPI_COMM_WORLD);
    MPI_Finalize();
    delete(comm);
    return 0;
}