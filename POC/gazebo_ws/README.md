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