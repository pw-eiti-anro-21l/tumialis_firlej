import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz2_file_name = 'manipulator.rviz'
    rviz2 = os.path.join(get_package_share_directory('zadanie5'), rviz2_file_name)

    x = 1.0
    y = 0.0
    z = 0.5
    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='rviz2',
            executable='rviz2',
            name='manipulator_rviz2',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            arguments=['-d', rviz2],
        ),

        Node(
            package='zadanie5',
            executable='oint',
            name='oint',
            parameters=[{
                'x': x,
                'y': y,
                'z': z,
                'roll': roll,
                'pitch': pitch,
                'yaw': yaw,
                'use_sim_time': use_sim_time,
            }],
            output='screen'),

    ])
