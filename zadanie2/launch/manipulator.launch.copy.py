import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # urdf_file_name = 'manipulator.urdf.xml'
    urdf_file_name = 'manipulator.urdf.xacro.xml'
    # poz1 = 0.0
    # poz2 = 0.0
    # poz3 = 0.0
    # print("urdf_file_name : {}".format(urdf_file_name))
    urdf = os.path.join(
        get_package_share_directory('zadanie2'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_robot',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': Command(['xacro', ' ', urdf])
            }]),
            # arguments=[urdf]),
        # Node(
        #     package='zadanie2',
        #     executable='state_publisher',
        #     name='state_publisher',
        #     parameters=[{'poz1': poz1}, {'poz2': poz2}, {'poz3': poz3}],
        #     output='screen'),
    ])
