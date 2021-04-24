objname := serial.o
libname := libSerial.so

build-echo:
	#g++ src/Serial.cpp src/Echo.cpp -Iinclude/ -o Echo -lpthread
	g++ src/Echo.cpp -o Echo -lpthread -lSerial

build-echo2:
	#g++ src/Serial.cpp src/Echo2.cpp -Iinclude/ -o Echo2 -lpthread
	g++ src/Echo2.cpp -o Echo2 -lpthread -lSerial

build-gs:
	#g++ src/Serial.cpp src/GroundStation.cpp -Iinclude/ -o GroundStation -lpthread
	g++ src/GroundStation.cpp -o GroundStation -lpthread -lSerial

build-gst:
	#g++ src/GroundStationTerminal.cpp -o GST -lpthread -lSerial
	g++ src/GroundStationTerminal.cpp -o GST -lpthread -lSerial -lboost_system

build-lib:
	g++ -c -Wall -Werror -fpic -Iinclude/ src/*.cpp -o $(objname)
	g++ -shared -o $(libname) $(objname)
	rm $(objname)

open-serial:
	socat -d -d pty,raw,echo=0 pty,raw,echo=0

clean:
	rm -f *.so *.o GroundStation Echo Echo2 GST

run-gst: build-gst
	python3 flush_serial.py
	./GST /dev/ttyACM0 57600
