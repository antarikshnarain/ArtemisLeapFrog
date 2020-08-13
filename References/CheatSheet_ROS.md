# ROS2 Cheat Sheet

## Installation step
*ROS Distro: ROS2-Foxy*
### Add repository
```bash
sudo sh -c 'echo "deb [arch=amd64] http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
```
### ROS Eloquent
```python
sudo apt install ros-foxy-desktop
sudo apt install ros-foxy-gazebo-ros-pkgs
sudo apt install python3-colcon-common-extensions
```
**Source the setup file**
```bash
source /opt/ros/foxy/setup.bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
printenv | grep -i ROS
```

## Command Architecture
* ros2
    * run
        * <package_name> <exectable_name>
        * rqt_console rqt_console
    * node
        * list
        * info <node_name>
    * topic
        * list -t
        * echo <topic_name>
        * info <topic_name>
        * pub <topic_name> <msg_type> '<args>'
        * hz <topic_name>
    * interface
        * show <type>.msg
    * service
        * list
        * type <service_name>
        * find <type_name>
        * call <service_name> <service_type> '<args>'
    * param - values set at the time of launch
        * list
        * get <node_name> <parameter_name>
        * set <node_name> <parameter_name> <value>
        * dump <node_name>
    * action
        * list
        * info <action_name>
        * send_goal <action_name> <action_type> <values>
    * launch
        * *Directory Structure*
            1. launch / <file_name>_launch.py
        * <package_name> <launch_file_name>
    * bag
        * record <topic_name>
        * info <bag_file_name>
        * play
    * doctor --report

* Check dependencies: 
```bash
rosdep install -i --from-path src --rosdistro <distro> -y
```
* Build workspace with colcon
    * creates the following directories *build* *install* *log*
    * 'install' is the new name for 'devel'. ROS 2 & 1 difference.

## Package
**Package can be created using CMake (C++) or Python**
### CMake
1. **package.xml** file containing meta information about the package
2. **CMakeLists.txt** file that describes how to build the code within the package
### Python
1. **package.xml** file containing meta information about the package
2. **setup.py** containing instructions for how to install the package
3. **setup.cfg** is required when a package has executables, so *ros2 run* can find them
4. **/<package_name>** - a directory with the same name as your package, used by ROS 2 tools to find your package, contains *__init__.py*
### Tasks
1. CMake
    1. Create package 
    ```bash
    ros2 pkg create --build-type ament_cmake <package_name>
    ```
    2. Create package with node 
    ```bash
    ros2 pkg create --build-type ament_cmake --node-name my_node <package_name>
    ```
2. Python
    1. Create package 
    ```bash
    ros2 pkg create --build-type ament_python <package_name>
    ```
    2. Create package with node 
    ```bash
    ros2 pkg create --build-type ament_python --node-name my_node <package_name>
    ```
### Custom Msg and Srv
Refer [link](https://index.ros.org/doc/ros2/Tutorials/Custom-ROS2-Interfaces/#custominterfaces)

## Steps to build
### Build changes
1. Build Packages
    1. All 
    ```bash 
    colcon build
    ```
    2. Specific 
    ```bash
    colcon build --packages-select my_package
    ```
2. Source the setup file 
```bash
. install/local_setup.bash
```

## Tips
### RQT
1. Use rqt service to pass values using GUI to the ROS services.
### Structure
1. Directory structure : {project_name}/dev_ws/src/
### Design
1. The ROS msgs are used by publisher and subscribers to continuously monitor data
2. The ROS srv are used to send requested data by client node