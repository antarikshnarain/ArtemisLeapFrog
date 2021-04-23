teamid := MyGPS1

teamid := lib$(teamid).so

build:
	g++ -c -Wall -Werror -fpic -I../include/ -Iinclude/ *.cpp 
	g++ -shared -o $(teamid) *.o
	cp $(teamid) ..

clean:
	rm *.o *.so