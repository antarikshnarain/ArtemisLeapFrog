objname := serial.o
libname := libSerial.so

build-exec:
	g++ src/Serial.cpp -Iinclude/ -o serial -lpthread -DTEST_SERIAL
build-lib:
	g++ -c -Wall -Werror -fpic -Iinclude/ src/*.cpp -o $(objname)
	g++ -shared -o $(libname) $(objname)
	rm $(objname)
clean:
	rm *.so *.o serial