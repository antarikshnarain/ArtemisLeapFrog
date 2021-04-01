"""Launch the sensors executable in this package"""

from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='sensors', 
            executable='imu', 
            output='screen',
            arguments=["/dev/ttyAMA0"]
        ),
        launch_ros.actions.Node(
            package='sensors', 
            executable='laser', 
            output='screen',
            arguments=["/dev/ttyAMA1","115200"]    
        ),
    ])