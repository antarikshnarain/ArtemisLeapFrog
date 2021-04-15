build-sensors:
	colcon build --packages-select sensors

build-communication:
	colcon build --packages-select communication

source:
	. install/local_setup.bash

run-sensors:
	ros2 lauch sensors sensors.launch.py

sensors:
	make build-sensors
	make source
	make run-sensors