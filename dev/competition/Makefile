#export LD_LIBRARY_PATH=$(pwd)/team:$LD_LIBRARY_PATH
current_dir = $(shell pwd)
SUBDIRS := $(wildcard team*/.)

build: main.cpp
	g++ -L$(current_dir)/team -Wall -o main main.cpp -lfoo
	#g++ -L$(pwd)/team -Wall -Wl,-rpath=$(pwd)/team -o main main.cpp -lfoo

# make source team=team_name
source: $(team)
	cp $(team)/*.so .
	mv *.so libMyGPS.so
	cp libMyGPS.so ../hardware/

clean:
	rm *.so
