import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    poz1 = 0.0
    poz2 = 0.0
    poz3 = 0.0

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='zadanie2',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'poz1': poz1,
                'poz2': poz2,
                'poz3': poz3,
                'use_sim_time': use_sim_time,
            }],
            output='screen'),

    ])
