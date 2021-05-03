import sys
import os
import xlsxwriter

datatypes = {
    "JCP300-Engine-Telem":"Turbine RPM, EGT V, Pump V, Turbine State, Throttle, Engine A",
    "JCP300-Fuel-Telem": "Actual Fuel Flow, Rest Fuel, RPM, Battery V, Last Run, Fuel Actual Run",
    "JCP300-System-Status": "Off Condition, Flight Speed",
    "Sensors-IMU": "raw_linear_acc_x, raw_linear_acc_y, raw_linear_acc_z, raw_angular_acc_x, raw_angular_acc_y, raw_angular_acc_z, roll, pitch, temp, linear_acc_x, linear_acc_y, linear_acc_z, angular_acc_x, angular_acc_y, angular_acc_z",
    "Sensors-Laser": "Distance, Strength, CheckSum"
}

if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise ValueError("Please pass log filename and output filename as input.")
    
    log_filename = sys.argv[1]
    output_filename = sys.argv[2]
    workbook = xlsxwriter.Workbook(output_filename)
    worksheets = dict([(k,workbook.add_worksheet(k)) for k in datatypes])
    # Add headers to sheets
    for k in worksheets:
        worksheets[k].write_row(0,0,datatypes[k].split(", "))
    row_counter = dict([(k,1) for k in datatypes])
    f = open(log_filename,'r')
    lines = f.readlines()
    f.close()
    for line in lines:
        split_1 = line.split("[rclcpp]: ")
        if len(split_1) == 2:
            split_2 = split_1[1].split(":")
            if len(split_2) == 2:
                split_3 = split_2[1].replace('\n','').split(",")
                if row_counter.get(split_2[0]) != None:
                    worksheets[split_2[0]].write_row(row_counter[split_2[0]],0, split_3)
                    row_counter[split_2[0]] += 1
    workbook.close()