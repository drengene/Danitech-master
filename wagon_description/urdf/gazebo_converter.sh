#!/bin/bash

file=$1
echo ${file}
filename=$(echo "${file}" | sed "s/.xacro//")
echo ${filename}
cp ${file} ${filename}_gazebo.xacro
sed -i 's+package://wagon_description/+$(find wagon_description)/+g' ${filename}_gazebo.xacro
xacro ${filename}_gazebo.xacro > ${filename}_gazebo.urdf
ign sdf -p ${filename}_gazebo.urdf > ../../wagon_gazebo/models/${filename}_gazebo.sdf
