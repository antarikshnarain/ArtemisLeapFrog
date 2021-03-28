# Setup Instructions

## Setting up System

**System Requirements**
1. Ubuntu 20.04 (Focal) / Mint 20.1 (Ulyssa) 

```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo apt-get install synaptics python3-lark
```

### Install Gazebo 11
```bash
# Setup APT
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable focal main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
# Install Gazebo
sudo apt-get update
sudo apt-get install gazebo11
# For developers that work on top of Gazebo, one extra package
sudo apt-get install libgazebo11-dev
```

### Install ROS | Noetic
```bash
# Setup APT
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# Install ROS Noetic
sudo apt update
sudo apt install ros-noetic-desktop

sudo apt install ros-noetic-gazebo-ros-pkgs
# Mavros package to communicate with 
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
```


### Install ROS2
```bash
# Setup APT
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list'
# Install ROS2 Foxy
sudo apt install ros-foxy-ros-base
## Desktop version
# sudo apt install ros-foxy-desktop

sudo apt install python3-argcomplete
sudo apt install ros-foxy-gazebo-ros-pkgs
sudo apt install python3-colcon-common-extensions
```

## Setting up Raspberry Pi

### Installation media

1. Install Raspberry Imager.
2. Download Navio Image.
3. Flash Raspbian OS Lite.
<!-- 4. Flash Navio Image. -->
4. Flash Ubuntu Server 20.04.2
5. Mount SD card on your desktop and create an empty file named ssh in boot partition.

```bash
touch ssh
```

### Configure Pi

1. Search Pi on network

    ```bash
    nmap -sn <base-ip-address>/24
    ## OR
    ssh pi@raspberrypi.local
    ## OR
    ssh pi@navio.local
    ```

2. Using *raspi-config* rename hostname and connect to wifi
    - Hostnames:
        1. leapfrog-root : master controller
        2. navio : pixhawk mimic controller
    - Wifi:
        - SSID: SERC Leapfrog
        - Pswd: *Refer the box* -> Hint: project hashname and timeline

3. Setup auto-login: On your local and Pi's

    ```bash
    ssh-keygen
    ssh-copy-id <username>@<hostip>
    ```

4. Install ROS on Raspberry Pi [link](https://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi)

## Setting up coding environment

### Pseudo Environment

1. To mimic serial port
    - Package

        ```bash
        sudo apt install socat
        socat -d -d pty,raw,echo=0 pty,raw,echo=0
        ```

    - Sample test

        ```bash
        # Terminal 1
        socat -d -d pty,raw,echo=0 pty,raw,echo=0
        # Terminal 2
        cat < /dev/pts/2
        # Terminal 3
        echo "Test2" > /dev/pts/2
        ```

## Setup Ground Station

1. Install QGroundControl Environment

    ```bash
    git clone https://github.com/mavlink/qgroundcontrol.git --recursive
    git submodule update
    ```

2. Install [Qt5](https://www.qt.io/download-open-source#section-2)

## Tips

1. Add SSH target to VSCode for better user experience.