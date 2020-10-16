# ArtemisLeapFrog
Repository for project LEAPFROG at Space Engineering Research Center (SERC), USC as part of NASA's project Artemis

## Software Sources
1. [Draw.io](https://github.com/jgraph/drawio-desktop/releases/download/v13.2.4/draw.io-amd64-13.2.4.deb) \
    Software to draw flowcharts and diagrams

## Tutorial Sources
1. [Gazebo](http://gazebosim.org/tutorials)
2. [ROS2](https://index.ros.org/doc/ros2/Tutorials/)

## Reference
### Sites
1. [URDF](http://gazebosim.org/tutorials/?tut=ros_urdf)
### Papers
### Books

## Architectures
### System Flow Diagram
![](Diagrams/SystemArchitecture-System%20Logic.png)


## Notes from merges

### Branch: Laser-Range-Sensor

#### Serialib

Serialib is a simple C++ library for serial communication. The library has been designed to work under Linux and Windows.

More details on [Lulu's blog](https://lucidar.me/en/serialib/cross-plateform-rs232-serial-library/)

#### Compile

```g++ -o MainProgram src/main.cpp include/serialib/serialib.cpp include/SEN0259/SEN0259.cpp```

### Branch: Pseudo-GPS

Pseudo GPS is a class written to calculate the relative position of the vehicle using just the accelerometer on an IMU.
The `src` folder contains 3 files: `PseudoGPS.cpp`,`PseudoGPS.h`, and `IMU.h`. 

#### PseudoGPS.cpp

This has all the implementation of the algorithm to double integrate the acceleration values coming from the IMU to position. The algorithm is based on this [paper](https://www.nxp.com/docs/en/application-note/AN3397.pdf). This class was designed around that algorithm and the code presented in the paper.

#### PseudoGPS.h

This is the header file for the PseudoGPS.cpp class. Outlines the functions and member variables of the class.

#### IMU.h

Because the IMU libraries were not written at the time of the publishing of this code, a generic IMU header file was created so implementation of each function can be decided in the future, while still being able to write the PseudoGPS class with the intended functionality.