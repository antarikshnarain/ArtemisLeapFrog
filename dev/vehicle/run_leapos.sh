#!/bin/bash
# give exec permission -> sudo chmod 744 run_leapos.sh
set -e
# Source ROS2
echo "Sourcing ROS2"
source /opt/ros/foxy/setup.bash
# Source Install path for packages
echo "Loading package path"
. ~/vehicle/install/local_setup.bash
# Run packages
echo "Running Sensor Package"
ros2 launch sensors sensors.launch.py
