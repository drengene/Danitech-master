# Copyright (c) 2020-2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import omni
import asyncio

from omni.isaac.core.utils import stage, extensions
from omni.isaac.sensor import _sensor
from omni.kit.commands import execute

from pxr import Gf

import rclpy
from rclpy.node import Node

from omni.isaac.ros2_bridge import _ros2_bridge

from sensor_msgs.msg import Imu
from numpy import array as nparray

# enable ROS2 bridge extension
extensions.enable_extension("omni.isaac.ros2_bridge")
extensions.enable_extension("omni.isaac.isaac_sensor")



class IMU(Node):
    def __init__(self, imu_prim_path="/wagon/base_scan", visualize=False, frame_id="base_scan"):
        super().__init__('imu_publisher',namespace=imu_prim_path)

        #imu_path = imu_parent + "/" + name

        self.imu_path = imu_prim_path
        self.frame_id = frame_id
        self.imu_sensor = _sensor.acquire_imu_sensor_interface()

        asyncio.ensure_future(self.create_scenario())

        #imu_ros_pub = _ros2_bridge.acquire_ros2_publisher_interface()



        self.imu_pub = self.create_publisher(Imu, "imu", 10)

        self.Covariance = nparray([0.01, 0, 0,
                                    0, 0.01, 0,
                                    0, 0, 0.01,])

        #self.timer = self.create_timer(0.05, self.imu_callback)





    def ros_pub(self):
        sensor_reading = self.imu_sensor.get_sensor_readings(self.imu_path + "/Imu_Sensor")

        
        for reading in sensor_reading:
            imu_msg = Imu()
            imu_msg.header.frame_id = self.frame_id
            imu_msg.header.stamp.sec = int(reading[0])
            imu_msg.header.stamp.nanosec = int((reading[0] - int(reading[0])) * 1e9)

            imu_msg.orientation.x = float(reading[7][0])
            imu_msg.orientation.y = float(reading[7][1])
            imu_msg.orientation.z = float(reading[7][2])
            imu_msg.orientation.w = float(reading[7][3])
            imu_msg.angular_velocity.x = float(reading[4])
            imu_msg.angular_velocity.y = float(reading[5])
            imu_msg.angular_velocity.z = float(reading[6])
            imu_msg.linear_acceleration.x = float(reading[1])
            imu_msg.linear_acceleration.y = float(reading[2])
            imu_msg.linear_acceleration.z = float(reading[3])
            imu_msg.orientation_covariance = self.Covariance
            imu_msg.angular_velocity_covariance = self.Covariance
            imu_msg.linear_acceleration_covariance = self.Covariance


            self.imu_pub.publish(imu_msg)

    
    def _on_stage_event(self, event):
        if event.type == int(omni.usd.StageEventType.CLOSED):
            self.on_closed()


    async def create_scenario(self):

        # Add IMU Sensor

        result, sensor = execute(
            "IsaacSensorCreateImuSensor",
            parent=self.imu_path,

            sensor_period=1 / 500.0,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
            visualize=True,
        )

        print("Starting imu sensors at: ", self.imu_path, result)
        self._events = omni.usd.get_context().get_stage_event_stream()
        self._stage_event_subscription = self._events.create_subscription_to_pop(
            self._on_stage_event, name="IMU Sensor Sample stage Watch"
        )

