"""
Plots raw acceleration data and then plot orientation data with kalman filter
"""
import csv
import matplotlib.pyplot as plt
import numpy as np

def read_file(filename):
    f = open(filename, 'r')
    records = csv.reader(f)
    datas = []
    for record in records:
        datas.append(record)
    datas.pop(0)
    datas.pop(len(datas)-1)
    f.close()
    return np.array(datas, dtype=np.float)

def model_cmp(file1, file2):
    data1 = read_file(file1)
    data2 = read_file(file2)
    
    # TODO: Accerelation Plot
    plt_list = []
    for i in range(3):
        p1, = plt.plot(data1[:,i])
        p2, = plt.plot(data2[:,i])
        plt_list.append(p1)
        plt_list.append(p2)

    plt.xlabel("Timestamp (ms)")
    plt.ylabel("Acceleration m/s^2")
    plt.legend(plt_list, ["Acc_X", "P_AccX", "Acc_Y", "P_AccY", "AccZ", "P_AccZ"])
    #plt.show()
    plt.savefig("accel_cmp.png")
    # TODO: Angle Plot
    plt_list = []
    for i in range(3,5):
        p1, = plt.plot(data1[:,i])
        p2, = plt.plot(abs(data2[:,i]))
        plt_list.append(p1)
        plt_list.append(p2)

    plt.xlabel("Timestamp (ms)")
    plt.ylabel("Degree")
    plt.legend(plt_list, ["Roll", "P_Roll", "Pitch", "P_Pitch"])
    #plt.legend(plt_list, ["Roll", "Pitch"])
    #plt.show()
    plt.savefig("angle_cmp.png")

if __name__ == "__main__":
    #model2("data2.dat")
    #plot_raw("data_raw_imu.dat")
    #plot_dmp("data_proc_imu.dat")
    model_cmp("data_raw_imu.dat", "data_proc_imu.dat")


