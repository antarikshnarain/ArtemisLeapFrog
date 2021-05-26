OBJ1=I2C.o
OBJ2=Serial.o
OLIB1=libI2C.so
OLIB2=libSerial.so

objname1 := serial.o
libname1 := libSerial.so
deps1 := include/Serial.hpp
objname2 := i2c.o
libname2 := libI2C.so
deps2 := include/I2C.hpp

build-lib-serial:
<<<<<<< HEAD
	g++ -c -Wall -fpic -Iinclude/ src/Serial.cpp -o $(objname1) -std=c++17
	g++ -shared -o $(libname1) $(objname1) -std=c++17
	rm $(objname1)

build-lib-i2c:
	g++ -c -Wall -Werror -fpic -Iinclude/ src/I2C.cpp -o $(objname2) -li2c -std=c++17
	g++ -shared -o $(libname2) $(objname2) -li2c -std=c++17
	rm $(objname2)

install: build-lib-serial
	install -m 755 -p $(libname1) /usr/lib/
	install -m 644 -p $(deps1) /usr/include/
	#install -m 755 -p $(libname2) /usr/lib/
	#install -m 644 -p $(deps2) /usr/include/
=======
	g++ -c -Wall -Werror -fpic -Iinclude/ src/Serial.cpp -o $(objname1) -lpthread
	g++ -shared -o $(libname1) $(objname1) -lpthread
	rm $(objname1)

build-lib-i2c:
	g++ -c -Wall -Werror -fpic -Iinclude/ src/I2C.cpp -o $(objname2) -li2c
	g++ -shared -o $(libname2) $(objname2) -li2c
	rm $(objname2)

install: build-lib-serial build-lib-i2c
	install -m 755 -p $(libname1) /usr/lib/
	install -m 644 -p $(deps1) /usr/include/
	install -m 755 -p $(libname2) /usr/lib/
	install -m 644 -p $(deps2) /usr/include/
>>>>>>> Dev-Hardware

uninstall:
	rm -f /usr/include/I2C.hpp /usr/include/Serial.hpp
	rm -f /usr/lib/libI2C.so /usr/lib/libSerial.so

clean:
	rm -f libSerial.so libI2C.so
