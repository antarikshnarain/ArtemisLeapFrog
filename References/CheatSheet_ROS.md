# ROS2 Cheat Sheet

## Installation step
### ROS Eloquent
```python
sudo apt install ros-eloquent-desktop
```
**Source the setup file**
```bash
source /opt/ros/eloquent/setup.bash
echo "source /opt/ros/eloquent/setup.bash" >> ~/.bashrc
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
    * param
        * list
        * get <node_name> <parameter_name>
        * set <node_name> <parameter_name> <value>
        * dump <node_name>
    * action
        * list
        * info <action_name>
        * send_goal <action_name> <action_type> <values>



## Tips
### RQT
1. Use rqt service to pass values using GUI to the ROS services.
