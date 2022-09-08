#!/bin/bash

cp wagon.xacro wagon_gazebo.xacro
sed -i 's+package://wagon_description/+$(find wagon_description)/+g' wagon_gazebo.xacro
xacro wagon_gazebo.xacro > wagon_gazebo.urdf
