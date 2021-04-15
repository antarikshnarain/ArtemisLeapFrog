test-imu:
	g++ src/MPU6050.cpp src/Kalman.cpp -Iinclude/ -DTEST -lwiringPi -lpthread -lm -lcrypt -lrt -o imu_test

test-laser:
	g++ src/SEN0259.cpp src/Serial.cpp -Iinclude/ -DTEST -lpthread
