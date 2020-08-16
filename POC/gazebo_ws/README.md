# Gazebo Workspace
Directory structure
* src
    * <project_name>
* build
    * <project_name>

## Build instruction

```bash
# cd build/<project_name>
cmake ../../src/<project_name>
make

# Add path to Gazebo Plugin
source /usr/share/gazebo/setup.sh
GAZEBO_PLUGIN_PATH=$(pwd):$GAZEBO_PLUGIN_PATH

# Run Gazebo Environment
gazebo -u src/<project_name>/worlds/<world_name>.world
```
## Project description
### Model Lift
Project focuses on lifting a block and hovering it using a PID Controller.

# Crater Landing

## Build instruction

```bash
# 1. Copy models into ~/.gazebo/models
cd crater_landing
cp -r models ~/.gazebo/models

# 2. Make build directory
cd crater_landing
mkdir build && cd build
cmake ../
make

# Should result in:
# [ 33%] Built target UpdateGreenZoneScore
# [ 66%] Built target UpdateBlueZoneScore
# [100%] Built target UpdateRedZoneScore

# Add path to Gazebo Plugin
source /usr/share/gazebo/setup.sh
GAZEBO_PLUGIN_PATH=$(pwd):$GAZEBO_PLUGIN_PATH

# Run Gazebo Environment
gazebo -u -- verbose crater_landing.world
```
## Project description
### Crater Landing
This is a proof of concept that we can use the Contain Plugin to print the corresponding score to their respective landing zone priority. 

Useful link:
[Contain Plugin Introduction](http://gazebosim.org/tutorials?tut=contain_plugin&cat=plugins)
