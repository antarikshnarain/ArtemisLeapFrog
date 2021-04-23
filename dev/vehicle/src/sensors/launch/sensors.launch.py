"""Launch the sensors executable in this package"""

from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='sensors', 
            executable='imu', 
            namespace='sensors',
            #output='screen',
            arguments=["104"]
        ),
        launch_ros.actions.Node(
            package='sensors', 
            executable='laser', 
            namespace='sensors',
            #output='screen',
            arguments=["/dev/ttyAMA2","115200"]    
        ),
    ])