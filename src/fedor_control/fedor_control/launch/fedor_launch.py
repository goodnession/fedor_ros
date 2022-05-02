from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fedor_control',
            namespace='fedorns',
            executable='fedor_connector'
        ),
        Node(
            package='fedor_control',
            namespace='fedorns',
            executable='fedor_controller'
        )
    ])
