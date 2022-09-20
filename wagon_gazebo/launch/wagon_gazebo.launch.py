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

    pkg_description = get_package_share_directory('wagon_description')
    pkg_bringup = get_package_share_directory('wagon_bringup')
    pkg_wagon_sim = get_package_share_directory('wagon_gazebo')
    model = "wagon_gazebo.sdf"

    pkg_ros_gz_sim = get_package_share_directory('ros_ign_gazebo')

    #model_path = os.path.join(pkg_description, 'urdf', model)
    model_path = os.path.join(pkg_wagon_sim, "models", model)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={'ign_args': 'empty.sdf'}.items(),

    )


    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')


    state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'wagon_state_publisher.launch.py')),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            # 'model' : '',
            # 'rvizconfig' : ''
            # 'use_sim_time' : ''
            # 'robot_description' : '' 
        }.items(),
    )

    print(model_path)
    spawn = Node(package='ros_ign_gazebo', executable='create',
                arguments=[
                    '-name', 'wagon',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '10.4',
                    '-file', model_path],
                output='screen')


    # RViz
    # rviz = Node(
    #    package='rviz2',
    #    executable='rviz2',
    #    arguments=['-d', os.path.join(pkg_description, 'rviz', 'wagon_model.rviz')],
    #    condition=IfCondition(LaunchConfiguration('rviz'))
    # )



    # Bridge
        # Gz - ROS Bridge
    bridge = Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            arguments=[
                #'/world/empty/model/wagon/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model',
                #'/model/wagon/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                #'/joint_states@sensors_msgs/msg/JointState@ignition.msgs.Model',
                #'/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V'
            ],
            #remappings=[
            #    ('/model/wagon/pose', '/tf'),
            #    ('/world/default/model/wagon/joint_state', '/joint_states')
            #]
        ),
    # bridge = Node(
    #     package='ros_ign_bridge',
    #     executable='parameter_bridge',
    #         arguments=[
                
    #             #'/world/empty/model/wagon/joint_state@'
    #             #'sensor_msgs/msg/JointState[ignition.msgs.Model',
    #             #'/model/wagon/pose@'
    #             #'tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
    #             # '/world/empty/model/wagon/joint_state@'
    #             # 'sensor_msgs/msg/JointState[ignition.msgs.Model',

    #         ],
    #         remappings=[
    #             ('/model/wagon/pose', '/tf'),
    #             ('/world/empty/model/wagon/joint_state', '/joint_states')
    #         ]
    # )

    return LaunchDescription([
        gz_sim,
        gui_arg,
        state_publisher,
        spawn,
        #bridge,
        #rviz
    ])