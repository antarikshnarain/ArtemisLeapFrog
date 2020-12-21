build:
	mkdir -p build
all:
	g++ -o build/GroundStation src/*.cpp -Isrc/include/
clean:
	rm -r build/*

pseudo-port:
	socat -d -d pty,raw,echo=0 pty,raw,echo=0
