objname1 := pid.o
libname1 := libPID.so
deps1 := include/PID.hpp
objname2 := kf.o
libname2 := libKF.so
deps2 := include/KalmanFilter.hpp

build-lib-pid:
	g++ -c -Wall -Werror -fpic -Iinclude/ src/PID.cpp -o $(objname1) 
	g++ -shared -o $(libname1) $(objname1)
	rm $(objname1)

build-lib-kf:
	g++ -c -Wall -Werror -fpic -Iinclude/ src/KalmanFilter.cpp -o $(objname2)
	g++ -shared -o $(libname2) $(objname2)
	rm $(objname2)

install: build-lib-pid build-lib-kf
	install -m 755 -p $(libname1) /usr/lib/
	install -m 644 -p $(deps1) /usr/include/
	install -m 755 -p $(libname2) /usr/lib/
	install -m 644 -p $(deps2) /usr/include/

uninstall:
	rm -f /usr/include/KalmanFilter.hpp /usr/include/PID.hpp
	rm -f /usr/lib/libKF.so /usr/lib/libPID.so

clean:
	rm -f libPID.so libKF.so
