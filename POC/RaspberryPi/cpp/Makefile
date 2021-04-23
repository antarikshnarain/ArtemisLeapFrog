objname := serial.o
libname := libSerial.so

build-lib-serial:
	g++ -c -Wall -Werror -fpic -Iinclude/ src/*.cpp -o $(objname)
	g++ -shared -o $(libname) $(objname)
	rm $(objname)

install-wiringpi:
	sudo apt-get install wiringpi

build-mpu6050:
	g++ MPU6050/src/MPU6050.cpp I2C/src/I2C.cpp -IMPU6050/include/ -II2C/include/ -DTEST_MPU6050 -lpthread -li2c -o mpu6050

build-serial:
	g++ Serial/src/Serial.cpp -ISerial/include/ -DTEST_SERIAL -DDEBUG_SERIAL -o serial -lpthread

build-serial2:
	g++ Serial/src/Serial.cpp -ISerial/include/ -DTEST_SERIAL2 -o serial -lpthread

open-serial:
	socat -d -d pty,raw,echo=0 pty,raw,echo=0

build-sen0259:
	g++ SEN0259/SEN0259.cpp Serial/src/Serial.cpp -ISerial/include/ -o sen0259 -lpthread

clean:
	rm -f serial mpu6050 a.out sen0259
	rm -f *.so *.o
