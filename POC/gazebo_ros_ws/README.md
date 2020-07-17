# Gazebo ROS workspace

This workspace is designed to build solutions for Gazebo and ROS environment
combined using [colcon](https://colcon.readthedocs.io/en/released/)

## Directory structure

```
.
├── path_setup.sh
├── README.md
└── src
    ├── gazebo_dev
    ├── gazebo_msgs
    ├── gazebo_plugins
    ├── gazebo_ros
    ├── gazebo_ros_pkgs
    ├── model_lift
    └── models
```

## Usage

Build and compile all packages with: 

```sh
colcon build --symlink-install
```

### Notes

- `src/models` contains custom models that need to be loaded into a world.
- In order to load models and plugins correctly `GAZEBO_PLUGIN_PATH` and
  `GAZEBO_MODEL_PATH` need to be set correctly.

Run the following script to set path vars to the defaults needed for this
directory structure.

```sh
source ./path_setup.sh
```

## Credits
The base source code has been cloned from 
- [Main Repository OSRF](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2)
- [Forked Repository](https://github.com/antarikshnarain/gazebo_ros_pkgs/tree/ros2)
