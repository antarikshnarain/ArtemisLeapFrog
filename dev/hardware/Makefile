current_dir = $(shell pwd)
num_procs = 4


build:
	mpiCC -o FlightSoftware src/*.cpp ../common/src/Communication.cpp ../common/src/Serial.cpp \
	 -Isrc/include/ -I../common/src/include/ -lboost_system -lboost_serialization

run: FlightSoftware
	mpirun -n $(num_procs) ./FlightSoftware

clean:
	rm -r *.so

test-competition:
	g++ -L../competition/ -I../competition/include/ -Wall -o TC src/TestCompetition.cpp -lMyGPS
	#g++ -L../competition/ -I../competition/include/ -Wl,-rpath=$(pwd)/../competition/ -Wall -o TC src/TestCompetition.cpp -lMyGPS

ACS
	mpic++ src/ColdGasThrusters.cpp -o ACSSystem

run-ACS: ACSSystem
	mpirun -n 7 --oversubscribe ./ACSSystem