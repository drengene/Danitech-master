# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from tkinter.messagebox import NO

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    sensors_launch_pkg = get_package_share_directory('sensors_launch')
    robot_localization_dir = get_package_share_directory('sensors_launch')
    parameters_file_dir = os.path.join(robot_localization_dir, 'params', "ekf")
    parameters_file_path = os.path.join(parameters_file_dir, 'imu_twist.yaml')

    config_arg = DeclareLaunchArgument(name='config', default_value=parameters_file_path,
                                    description='Path to ekf config file')
    
    sim_arg = DeclareLaunchArgument(name='sim', default_value='false')


    wheel_odom_node = Node(
        package='wheel_odometry',
        executable='wheel_odom',
        name='wheel_odom_node',
        output='screen',
        parameters=[{'wheel_radius': 0.292,
                    'front_wheel_separation': 0.963,
                    'rear_wheel_separation': 0.963,
                    'front_base_radius': 0.6948+0.085,
                    'rear_base_radius': 0.8716-0.085,
                    'joint_state_topic': '/joint_states',
                    'wheel_odom_topic': '/wheel_odom',

                    }]
        )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensors_launch_pkg, 'launch', 'ekf', 'imu_twist.launch.py')),
        launch_arguments={
        }.items(),
        condition = UnlessCondition(LaunchConfiguration('sim'))
    )
    ekf_launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensors_launch_pkg, 'launch', 'ekf', 'dual_ekf_navsat_sim.launch.py')),
        launch_arguments={
        }.items(),
        condition = IfCondition(LaunchConfiguration('sim'))
    )

    return LaunchDescription([
        config_arg,
        sim_arg,
        wheel_odom_node,
        ekf_launch,
        ekf_launch_sim,

    ])