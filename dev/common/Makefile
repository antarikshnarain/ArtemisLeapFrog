build-test:
	g++ src/*.cpp -DTEST -Isrc/include/ -o Communication

run: ./Communication $(port)
	./Communication $(port)

pseudo-port:
	socat -d -d pty,raw,echo=0 pty,raw,echo=0

clean-properties:
	rm libProperties.so Properties.o

build:
	@make properties utilities

properties:
	@tput setaf 2
	@echo "Building dynamic library for Properties namespace"
	@g++ -c -fpic src/*Properties.cpp -Isrc/include/ -o Properties.o
	@g++ -shared -o libProperties.so Properties.o
	@tput setaf 0

utilities:
	@tput setaf 2
	@echo "Building dynamic library for Utilities namespace"
	@tput setaf 0
