current_dir = $(shell pwd)

build:
	mkdir -p build

clean:
	rm -r *.so

test-competition:
	g++ -L../competition/ -I../competition/include/ -Wall -o TC src/TestCompetition.cpp -lMyGPS
	#g++ -L../competition/ -I../competition/include/ -Wl,-rpath=$(pwd)/../competition/ -Wall -o TC src/TestCompetition.cpp -lMyGPS

ACS
	mpic++ src/ColdGasThrusters.cpp -o ACSSystem
run-ACS: ACSSystem
	mpirun -n 7 --oversubscribe ./ACSSystem