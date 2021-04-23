"""Launch the sensors executable in this package"""

from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='flightcontrol', 
            executable='flightmanager',
            namespace='flightcontrol',
            #output='screen',
            arguments=["/dev/ttyAMA4","57600"]
        ),
    ])