import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():

    sensors_launch_pkg = get_package_share_directory('sensors_launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    urdf_file_name = 'wagon.urdf'
    urdf = os.path.join(
        get_package_share_directory('wagon_description'),
        "urdf",
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()


    sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true')

    state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf])

    wheel_vel_translator_node = Node(
            package='wheel_vel_translator',
            executable='wheel_vel_translator',
            name='wheel_vel_translator',
            output='screen',
            parameters=[{}],
    )

    wheel_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensors_launch_pkg, 'launch', 'odom_ekf.launch.py')),
        launch_arguments={
        }.items(),
    )



    return LaunchDescription([
        sim_time_arg,
        state_publisher_node,
        wheel_vel_translator_node,
        wheel_odom_launch
        ])