objname := serial.o
libname := libSerial.so

flush-serial:
	python3 flush_serial.py

build-echo:
	g++ src/Echo.cpp -o Echo -lpthread -lSerial -std=c++17

build-echo2:
	g++ src/Echo2.cpp -o Echo2 -lpthread -lSerial -std=c++17

build-gs:
	#g++ src/Serial.cpp src/GroundStation.cpp -Iinclude/ -o GroundStation -lpthread
	g++ src/GroundStation.cpp -o GroundStation -lpthread -lSerial -std=c++17

build-gst: flush-serial
	g++ src/GroundStationTerminal.cpp -o GST -lpthread -lSerial -lboost_system -std=c++17

build-lib:
	g++ -c -Wall -Werror -fpic -Iinclude/ src/*.cpp -o $(objname) -std=c++17
	g++ -shared -o $(libname) $(objname) -std=c++17
	rm $(objname)

open-serial:
	socat -d -d pty,raw,echo=0 pty,raw,echo=0

clean:
	rm -f *.so *.o GroundStation Echo Echo2 GST

run-gst: build-gst
	python3 flush_serial.py
	./GST /dev/ttyACM0 57600
