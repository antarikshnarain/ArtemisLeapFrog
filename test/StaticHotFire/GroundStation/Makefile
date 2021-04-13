objname := serial.o
libname := libSerial.so

build-echo:
	#g++ src/Serial.cpp src/Echo.cpp -Iinclude/ -o Echo -lpthread
	g++ src/Echo.cpp -o Echo -lpthread -lSerial

build-echo2:
	#g++ src/Serial.cpp src/Echo2.cpp -Iinclude/ -o Echo -lpthread
	g++ src/Echo2.cpp -o Echo2 -lpthread -lSerial

build-gs:
	g++ src/Serial.cpp src/GroundStation.cpp -Iinclude/ -o GroundStation -lpthread

build-lib:
	g++ -c -Wall -Werror -fpic -Iinclude/ src/*.cpp -o $(objname)
	g++ -shared -o $(libname) $(objname)
	rm $(objname)

open-serial:
	socat -d -d pty,raw,echo=0 pty,raw,echo=0

clean:
	rm -f *.so *.o GroundStation Echo Echo2