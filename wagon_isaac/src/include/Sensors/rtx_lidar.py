# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import carb
import sys
import shutil
import os

from omni.kit.viewport_legacy import get_viewport_interface
import omni.kit.viewport.utility
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import stage, extensions, nucleus
from omni.syntheticdata import sensors
from omni.kit.commands import execute
from omni.isaac.core.utils.render_product import create_hydra_texture
from omni.isaac.core.utils.nucleus import get_assets_root_path
import omni.replicator.core as rep

from pxr import Gf

# enable ROS2 bridge extension
extensions.enable_extension("omni.isaac.ros2_bridge")

class lidar_3d():
    def __init__(self, lidar_path="/sensor", lidar_parent=None, config_file="Example_Rotary", copy_config=False):

        # Create a new viewport for the RTX sensor and acquire the viewport window
        # self.vpi = get_viewport_interface()
        # self.instance = self.vpi.create_instance()
        # self.viewport_handle = self.vpi.get_instance("Viewport 2")
        # self.viewport = self.vpi.get_viewport_window(self.viewport_handle)
        # in order for the sensor to generate data properly we let the viewport know that it should create a buffer for the associated render variable.
        # self.viewport.add_aov("RtxSensorCpu", False)

        # # Create the post process graph that publishes the render var
        # sensors.get_synthetic_data().activate_node_template(
        #     "RtxSensorCpu" + "ROS2PublishPointCloud", 0, [self.viewport.get_render_product_path()]
        # )

        if copy_config:
            dir_path = os.path.dirname(os.path.realpath(__file__))
            config_path = os.path.abspath(os.path.join(dir_path, "..", "..", "..", "config"))
            config_file_name = config_file + ".json"
            # print(dir_path)
            # print(config_path)
            ext_path = os.path.abspath(os.path.join(extensions.get_extension_path_from_name("omni.isaac.ros2_bridge"), ".."))
            ext_config_path = os.path.join(ext_path, "omni.drivesim.sensors.nv.lidar", "data")
            print("\nConfig file at path: ", os.path.join(config_path, config_file_name), "\nis being moved to: ", ext_config_path)
            shutil.copy2(os.path.join(config_path, config_file_name), ext_config_path)

        # Create the lidar sensor that generates data into "RtxSensorCpu"
        # Sensor needs to be rotated 90 degrees about X so that its Z up
        # Possible options are Example_Rotary and Example_Solid_State
        # Create the lidar sensor that generates data into "RtxSensorCpu"
        # Sensor needs to be rotated 90 degrees about X so that its Z up

        # Possible options are Example_Rotary and Example_Solid_State
        # drive sim applies 0.5,-0.5,-0.5,w(-0.5), we have to apply the reverse
        _, sensor = execute(
            "IsaacSensorCreateRtxLidar",
            path=lidar_path,
            parent=lidar_parent,
            config=config_file,
            translation=(0, 0, 1.0),
            orientation=Gf.Quatd(0.5, 0.5, -0.5, -0.5),  # Gf.Quatd is w,i,j,k
        )

        # RTX sensors are cameras and must be assigned to their own render product
        _, render_product_path = create_hydra_texture([1, 1], sensor.GetPath().pathString)

        # Create Point cloud publisher pipeline in the post process graph
        writer = rep.writers.get("RtxLidar" + "ROS2PublishPointCloud")
        writer.initialize(topicName="point_cloud", frameId="base_scan")
        writer.attach([render_product_path])

        # Create the debug draw pipeline in the post process graph
        writer = rep.writers.get("RtxLidar" + "DebugDrawPointCloud")
        writer.attach([render_product_path])


        # Create LaserScan publisher pipeline in the post process graph
        writer = rep.writers.get("RtxLidar" + "ROS2PublishLaserScan")
        writer.initialize(topicName="laser_scan", frameId="base_scan")
        writer.attach([render_product_path])


        # simulation_context = SimulationContext(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0, stage_units_in_meters=1.0)