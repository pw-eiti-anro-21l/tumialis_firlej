import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz2_file_name = 'manipulator.rviz'
    rviz2 = os.path.join(get_package_share_directory('zadanie4'), rviz2_file_name)
    xacro_file_name = 'manipulator.urdf.xacro.xml'
    xacro = os.path.join(get_package_share_directory('zadanie4'), xacro_file_name)

    poz1 = 0.0
    poz2 = 0.0
    poz3 = 0.0

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': Command(['xacro', ' ', xacro])
            }]),

        Node(
            package='zadanie4',
            executable='jint',
            name='jint',
            parameters=[{
                'poz1': poz1,
                'poz2': poz2,
                'poz3': poz3,
                'use_sim_time': use_sim_time,
            }],
            output='screen'),

    ])
