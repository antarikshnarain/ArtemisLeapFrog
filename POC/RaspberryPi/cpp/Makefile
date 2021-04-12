install-wiringpi:
	sudo apt-get install wiringpi

build-mpu6050:
	g++ MPU6050/src/MPU6050.cpp I2C/src/I2C.cpp -IMPU6050/include/ -II2C/include/ -DTEST_MPU6050 -lpthread -li2c -o mpu6050

build-serial:
	g++ Serial/src/Serial.cpp -ISerial/include/ -DTEST_SERIAL -o serial -lpthread

build-serial2:
	g++ Serial/src/Serial.cpp -ISerial/include/ -DTEST_SERIAL2 -o serial -lpthread

open-serial:
	socat -d -d pty,raw,echo=0 pty,raw,echo=0

clean:
	rm -f serial mpu6050