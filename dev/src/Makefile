
build-sensors:
	colcon build --packages-select sensors

source:
	. install/local_setup.bash

run-sensors:
	ros2 lauch sensors sensors.launch.py

sensors:
	make build-sensors
	make source
	make run-sensors