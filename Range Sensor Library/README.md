# (UNTESTED) Range Sensor Librar for the SEN0259 Sensor
This is a driver for the SEN0259 sensor used to measure range (distance). 
The `src` folder contains 1 file: `SEN0259.cpp`
The `include` folder contains 2 folders: `SEN0259` and `serialib`

## Compilation and Running
You can run the included `compile.sh` file like this:
`./compile.sh` 
(make sure to give this file execution permission by running `./chmod +x compile.sh`)

If you do not want to run the bash file, you can run the following command:

`g++ -o MainProgram src/main.cpp include/serialib/serialib.cpp include/SEN0259/SEN0259.cpp`

The output file will be `MainProgram` which you can run using the command `./MainProgram`

## SEN0259.cpp
This contains the methods to use the sensor. You must run `begin(port)` in order to start using the sensor. This tries to establish a connection with the sensor. After that in your function you can call `measure()` and `getDistance()` and/or `getStrength()`. 
You must call `measure()` before calling `getDistance()` or `getStrength()`.
Once you are done using the device, you can call `close()` in order to close the device and call the destructor.

## SEN0259.h
This is the header file for the `SEN0259.cpp` class. Outlines the class, functions, and member variables of the class.

## Serialib Library
The built-in serial library in C++ is a bit complex to use, so this external library from [here](https://github.com/imabot2/serialib "here"), which requires no external dependencies, is convenient to use since it is similar to the built-in Arduino serial library.

## Wiring (UNTESTED)
Connect the SEN0259 Sensor to a [USB to TTL Serial Cable](https://www.adafruit.com/product/954 "USB to TTL Serial Cable"), then connect the USB end to any System**, find the usb port address on your system, then run the program.

## **Notes
This library is designed for use on Linux, but might be usable on Windows since the `serialib` library is designed for use on both Linux and Windows.
This library is untested, since the sensor was not available for testing at the time of development of this library. Small tweaks may be needed.