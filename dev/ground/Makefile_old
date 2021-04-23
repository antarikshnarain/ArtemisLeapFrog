build:
	mpiCC -o GroundStation src/*.cpp ../common/src/Communication.cpp ../common/src/Serial.cpp \
	 -Isrc/include/ -I../common/src/include/ -lboost_system -lboost_serialization

run: GroundStation
	mpirun -n 2 ./GroundStation
all:
	g++ -o build/GroundStation src/*.cpp -Isrc/include/
clean:
	rm -r build/*

build:
	mpiCC src/*.cpp ../common/src/*.cpp -I../common/src/include -lboost_system -lboost_serialization -std=c++17 -o GroundStation