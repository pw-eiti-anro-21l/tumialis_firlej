from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='zadanie1',
            prefix='gnome-terminal --',
            namespace='',
            executable='talker',
            name='velocityNode'
        )
    ])
