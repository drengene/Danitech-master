import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
   


        print("Launching from file: " + __file__)

        mouse_teleop = get_package_share_directory('mouse_teleop')

        velocity_scale = LaunchConfiguration('scale', default= '1.0')

        frequency = LaunchConfiguration('frequency', default= '10.0')

        velocity_scale_arg = DeclareLaunchArgument(
                'scale',
                default_value='1.0',
                description='Set scaling of velocity commands')

        

        mouse_teleop = Node(
                package='mouse_teleop',
                executable='mouse_teleop',
                name='mouse_teleop',
                output='screen',
                parameters=[{'scale': velocity_scale, 'frequency': frequency}],
                remappings=[('/mouse_vel', '/cmd_vel')]
        )


        return LaunchDescription([
        velocity_scale_arg,
        frequency,
        mouse_teleop,

        ])