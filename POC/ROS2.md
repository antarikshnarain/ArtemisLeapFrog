# ROS2-Foxy | Ubuntu-Focal

**System Requirements**
- Linux Distro: Ubuntu 20.04 (Focal)
- Python3.6+
- Git ```sudo apt install git-all```

**General**
- There are 5 primary modules in the ROS2 framework.
    - Nodes: independent program running a particular module.
    - Topics: topics available in the node service.
    - Services: Client-Server system to request/call and perform action.
    - Parameters: Setting changable properties of the node.
    - Actions: Run on client-server system to reach a goal, a result and provide feedback. 

## Installation

1. Set Locale
    ```Bash
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    locale  # verify settings
    ```
2. Setup Sources
    ```bash
    sudo apt update && sudo apt install curl gnupg2 lsb-release
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list'    
    ```
3. Install ROS2 packages
    ```bash
    sudo apt update
    ## Basic Installation
    #sudo apt install ros-foxy-ros-bash
    ## Full Installation
    sudo apt install ros-foxy-desktop
    source /opt/ros/foxy/setup.bash
    ```
4. Install ArgComplete
    ```bash
    sudo apt install -y python3-pip
    pip3 install -U argcomplete
    ```
5. Install Colcon
    ```bash
    sudo apt install python3-colcon-common-extensions
    ```
6. Install RosDep
    ```bash
    sudo apt-get install python3-rosdep
    sudo rosdep init
    rosdep update
    ```

## Environment Setup
```bash
source /opt/ros/foxy/setup.bash
```
- Workspace design
```
workspace_folder/
    src/
      package_1/
          CMakeLists.txt
          package.xml

      package_2/
          setup.py
          package.xml
          resource/package_2
      ...
      package_n/
          CMakeLists.txt
          package.xml
```

1. Create Package
    ```bash
    ros2 pkg create --build-type ament_cmake <package-name>
    ## Package with node name
    ros2 pkg create --build-type ament_cmake <package-name> --node-name <node-name>
    ```
2. Build and Source Package
    ```bash
    colcon build --packages-select <package-name>
    . install/setup.bash
    ```
3. Using the package
    ```bash
    ros2 run <package-name> <node-name>
    ```

### Node Setup
1. Dependencies corresponding to nodes are added to 
    - *package.xml*
    ```xml
    <depend>package-name</depend>
    ```
    - *CMakeLists.txt*
    ```CMake
    find_package(package-name REQUIRED)
    ...
    add_executable(node-name src/filename.cpp)
    ament_target_dependencies(node-name package-name[s])
    ...
    install(TARGETS
        node-name
        DESTINATION lib/${PROJECT_NAME})
    ```

## Publisher Subscriber


## Service Client

## Uninstall ROS2

## Tips


## References

1. [ROS-Interfaces](https://docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html)
2. [ROS-Parameters](https://docs.ros.org/en/foxy/Tutorials/Using-Parameters-In-A-Class-CPP.html#cppparamnode)