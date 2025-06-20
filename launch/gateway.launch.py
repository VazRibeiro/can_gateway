from launch import LaunchDescription
from launch_ros.actions import LifecycleNode


def generate_launch_description():
    return LaunchDescription([
        LifecycleNode(
            package='can_gateway',
            executable='can_gateway_node',
            name='can_gateway',
            namespace='',               
            output='screen',
            parameters=[{
                'interface': 'vcan0',   # change to 'can0' on the robot
                'rx_rate_hz': 1000
            }]
        )
    ])
