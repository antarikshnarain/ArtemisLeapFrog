build-test:
	g++ src/*.cpp -DTEST -Isrc/include/ -o Communication

run: ./Communication $(port)
	./Communication $(port)

pseudo-port:
	socat -d -d pty,raw,echo=0 pty,raw,echo=0

clean:
	rm Communication