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

    bringup_launch_dir = get_package_share_directory('wagon_bringup')
    

    

    global_planner_node = Node(
        package='wagon_navigation',
        executable='global_planner',
        name='global_planner',
        output='screen',
        )


    controller_node = Node(
        package='wagon_navigation',
        executable='articulation_controller',
        name='articulation_controller',
        output='screen',
        )

    wagon_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch_dir, 'launch', 'wagon_bringup.launch.py')),
        launch_arguments={
            'use_ekf': 'false',
            'pub_world' : 'true',
        }.items(),
    )


    return LaunchDescription([
        wagon_bringup_launch,
        global_planner_node,
        controller_node,
    ])