#!/bin/bash

file=$1
#echo ${file}

Extension='.xacro'
if [[ "$file" == *"$Extension"* ]]; 
then


    filename=$(echo "${file}" | sed "s/.xacro//")
    #echo ${filename}
    cp ${file} ${filename}_gazebo.xacro
    sed -i 's+package://wagon_description/meshes+meshes+g' ${filename}_gazebo.xacro
    sed -i 's+$(find wagon_description)/meshes+meshes+g' ${filename}_gazebo.xacro
    xacro ${filename}_gazebo.xacro > ${filename}_gazebo.urdf
    rm ${filename}_gazebo.xacro
    ign sdf -p ${filename}_gazebo.urdf > ../../wagon_gazebo/models/${filename}/model.sdf

    cp -r ../meshes ../../wagon_gazebo/models/${filename}/

    sed -i '/model /r ../../wagon_gazebo/models/wagon/plugins.sdf' ../../wagon_gazebo/models/${filename}/model.sdf
else
    echo "Please give a .xacro file as input"

fi
