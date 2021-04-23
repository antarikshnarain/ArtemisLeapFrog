"""Launch the sensors executable in this package"""

from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='actuators', 
            executable='engine',
            namespace='actuators',
            #output='screen',
            arguments=["/dev/ttyAMA3", "9600"]
        ),
        launch_ros.actions.Node(
            package='actuators', 
            executable='acs',
            namespace='actuators',
            #output='screen',
            arguments=[]    
        ),
    ])