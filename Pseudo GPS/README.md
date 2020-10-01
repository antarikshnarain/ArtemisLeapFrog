 # Pseudo GPS
Pseudo GPS is a class written to calculate the relative position of the vehicle using just the accelerometer on an IMU.
The `src` folder contains 3 files: `PseudoGPS.cpp`,`PseudoGPS.h`, and `IMU.h`. 

## PseudoGPS.cpp
This has all the implementation of the algorithm to double integrate the acceleration values coming from the IMU to position. The algorithm is based on this [paper](https://www.nxp.com/docs/en/application-note/AN3397.pdf). This class was designed around that algorithm and the code presented in the paper.

## PseudoGPS.h
This is the header file for the PseudoGPS.cpp class. Outlines the functions and member variables of the class.

## IMU.h
Because the IMU libraries were not written at the time of the publishing of this code, a generic IMU header file was created so implementation of each function can be decided in the future, while still being able to write the PseudoGPS class with the intended functionality.