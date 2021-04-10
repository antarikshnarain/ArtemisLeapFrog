install-wiringpi:
	sudo apt-get install wiringpi

build-mpu6050:
	g++ MPU6050/src/MPU6050.cpp -IMPU6050/include/ -DTEST_MPU6050 -o mpu6050 -lpthread -lwiringPi -lrt -lm -lcrypt

build-serial:
	g++ Serial/src/Serial.cpp -ISerial/include/ -DTEST_SERIAL -o serial -lpthread

build-serial2:
	g++ Serial/src/Serial.cpp -ISerial/include/ -DTEST_SERIAL2 -o serial -lpthread

open-serial:
	socat -d -d pty,raw,echo=0 pty,raw,echo=0

clean:
	rm serial mpu6050