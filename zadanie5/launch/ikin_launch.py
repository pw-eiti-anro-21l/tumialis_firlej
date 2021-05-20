import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    yaml_file_name = 'macierzDH.yaml'
    yaml = os.path.join(get_package_share_directory('zadanie5'), yaml_file_name)
    xacro_file_name = 'manipulator.urdf.xacro.xml'
    xacro = os.path.join(get_package_share_directory('zadanie5'), xacro_file_name)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='zadanie5',
            executable='ikin',
            name='ikin_node',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_file': yaml
            }],
            output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': Command(['xacro', ' ', xacro])
            }]),

    ])
