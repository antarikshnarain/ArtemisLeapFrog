/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG 
 * Author     : Antariksh Narain
 * Description: Independent code to control cold gas thrusters simultaneously
----------------------------------------------------------------- */

#include <iostream>
#include <mpi.h>
#include <unistd.h>
#include <vector>
#include <wiringPi.h>
#include "MPU6050.hpp"

using namespace std;

MPU6050 mpu(0x68);

void digitalWrite(int pin, bool value)
{
    printf("Digitial pin %d to value %d\n", pin, value);
}

void FireThrusterThread(int index)
{
    printf("Starting thread %d\n", index);
    int duration;
    MPI_Recv(&duration, 1, MPI_INT, 0, 1, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
    printf("Running %d for duration %d\n", index, duration);
    while(duration > 0)
    {
        digitalWrite(index, true);
        usleep(duration * 1000000.0);
        digitalWrite(index, false);
        MPI_Recv(&duration, 1, MPI_INT, 0, 1, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
        printf("Running %d for duration %d\n", index, duration);
    }

    printf("Exiting thread %d\n", index);
}

volatile vector<double>GetDurations()
{
    vector<double> data = mpu.PrincipalAxisValues();
    float e_roll = Kp*(initial_averages[0]-averages[0])-Kd*averages[6];
    float e_pitch = Kp*(initial_averages[1]-averages[1])-Kd*averages[7];
    float e_yaw = Kp*(initial_averages[2]-averages[2])-Kd*averages[8];

    float t_roll;
    float t_pitch;
    float t_yaw;

	fprintf(stdout, "E roll : %f\n\n", e_roll);
	fprintf(stdout, "E pitch : %f\n\n", e_pitch);
	fprintf(stdout, "E yaw : %f\n\n", e_yaw);

    // if e is over maximum, set time to 100%
    if(abs(e_roll) > Tmax){
    	t_roll = TIME_PERIOD;
    } 
    // if e is under minimum, set time to 0
    else if(abs(e_roll) < Tmin){
    	t_roll = 0;
    }
    // if not, set to fraction of time period
    else {
    	t_roll = abs(e_roll)/Tmax*TIME_PERIOD;
    }
    if(abs(e_pitch) > Tmax){
    	t_pitch = TIME_PERIOD;
    } 
    else if(abs(e_pitch) < Tmin){
    	t_pitch = 0;
    }
    else {
    	t_pitch = abs(e_pitch)/Tmax*TIME_PERIOD;
    }
    if(abs(e_yaw) > Tmax){
    	t_yaw = TIME_PERIOD;
    } 
    else if(abs(e_yaw) < Tmin){
    	t_yaw = 0;
    }
    else {
    	t_yaw = abs(e_yaw)/Tmax*TIME_PERIOD;
    }
	fprintf(stdout, "t roll : %f\n\n", t_roll);
	fprintf(stdout, "T pitch : %f\n\n", t_pitch);
	fprintf(stdout, "T yaw : %f\n\n", t_yaw);
    
    return vector<double>{t_roll, t_pitch, t_yaw};
}

int
main(int argc, char **argv)
{
    int process_Rank, size_Of_Cluster, message_Item;
    MPI_Init(&argc, &argv);
    MPI_Comm_size(MPI_COMM_WORLD, &size_Of_Cluster);
    MPI_Comm_rank(MPI_COMM_WORLD, &process_Rank);
    
    if(process_Rank == 0)
    {
        vector<double> durations = GetDurations();
        int value = 5;
        //MPI_Send(&value, 1, MPI_INT, 1, 1, MPI_COMM_WORLD);
        MPI_Send(&durations[0], 1, MPI_INT, 1, 1, MPI_COMM_WORLD);
        value = 10;
        //MPI_Send(&value, 1, MPI_INT, 2, 1, MPI_COMM_WORLD);
        MPI_Send(&durations[1], 1, MPI_INT, 2, 1, MPI_COMM_WORLD);
        value = 3;
        //MPI_Send(&value, 1, MPI_INT, 3, 1, MPI_COMM_WORLD);
        MPI_Send(&durations[2], 1, MPI_INT, 3, 1, MPI_COMM_WORLD);
        //usleep(7 * 1000000.0);
        printf("Parent Done!");
        value = 0;
        for(int i=1;i<size_Of_Cluster;i++)
        {
            MPI_Send(&value, 1, MPI_INT, i, 1, MPI_COMM_WORLD);   
        }
        printf("Closing Others Request Done!");
    }
    else
    {
        FireThrusterThread(process_Rank);
    }
    MPI_Barrier(MPI_COMM_WORLD);
    MPI_Finalize();
    return 0;
}