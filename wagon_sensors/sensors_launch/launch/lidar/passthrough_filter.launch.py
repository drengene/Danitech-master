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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():


    passthrough_filter_node = Node(
        package='nodelet',
        executable='nodelet',
        name='lidar_passthrough_filter_node',
        output='screen',
        arguments=['standalone pcl/PassThrough'],
        parameters=[{'filter_limit_min': 2,

                    }],
        remappings=[('input', '/wagon/base_scan/lidar_data')]
        )

    return LaunchDescription([
        passthrough_filter_node,

    ])