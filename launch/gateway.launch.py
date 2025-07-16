from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    can_gateway = LifecycleNode(
        namespace='',
        package='can_gateway',
        executable='can_gateway_node',
        name='can_gateway',
        output='screen',
        parameters=[{'interface': 'vcan0'}]
    )

    lifecycle_manager = Node(
        namespace='',
        package='can_gateway',
        executable='lifecycle_manager.py',
        name='lifecycle_manager_can',
        output='screen',
        parameters=[{'managed_nodes': ['can_gateway']}]
    )

    return LaunchDescription([can_gateway, lifecycle_manager])