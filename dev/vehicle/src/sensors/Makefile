test-imu:
	g++ src/MPU6050.cpp -Iinclude/ -DTEST -lwiringPi -lpthread -lm -lcrypt -lrt -o imu_test

test-laser:
	g++ src/SEN0259.cpp -Iinclude/ -DTEST -lpthread

test-ads:
	g++ src/ADS1015.cpp -Iinclude/ -DTEST_ADS1015 -lI2C -lpthread -o ads_test