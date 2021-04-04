objname := serial.o
libname := libSerial.so

build-gs:
	g++ src/Serial.cpp src/GroundStation.cpp -Iinclude/ -o GroundStation -lpthread
build-lib:
	g++ -c -Wall -Werror -fpic -Iinclude/ src/*.cpp -o $(objname)
	g++ -shared -o $(libname) $(objname)
	rm $(objname)
clean:
	rm *.so *.o GroundStation