from omni import usd
import omni.isaac.core as core
# from omni.isaac.core import SimulationContext, get_relative_transform

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import PoseStamped

class pose_pub(Node):
    def __init__(self, pose_prim_path="/wagon/base_link", frame_id="base_link"):
        super().__init__('isaac_gps_publisher')
        self.stage = usd.get_context().get_stage()      # Used to access Geometry
        # self.transform_utils = core.get_transform_utils()  # Used to access Geometry
        self.sim_context =  core.SimulationContext()        # Used to interact with simulation
        self.pose_prim = self.stage.GetPrimAtPath(pose_prim_path)
        self.world_prim = self.stage.GetPrimAtPath("/world")
        self.frame_id = frame_id
        self.gps_pub = self.create_publisher(PoseStamped, self.frame_id + "_pose", 10)

        self.pose_msg = PoseStamped()



    def ros_pub(self):
        sim_time = self.sim_context.current_time
        self.pose_msg.header.stamp.sec = int(sim_time)
        self.pose_msg.header.stamp.nanosec = int((sim_time - int(sim_time)) * 1e9)
        self.pose_msg.header.frame_id = self.frame_id
        pos = self.pose_prim.GetAttribute("xformOp:translate").Get()
        orr = self.pose_prim.GetAttribute("xformOp:orient").Get()
        orr_imag = orr.GetImaginary()

        self.pose_msg.pose.position.x = float(pos[0])
        self.pose_msg.pose.position.y = float(pos[1])
        self.pose_msg.pose.position.z = float(pos[2])
        self.pose_msg.pose.orientation.x = float(orr_imag[0])
        self.pose_msg.pose.orientation.y = float(orr_imag[1])
        self.pose_msg.pose.orientation.z = float(orr_imag[2])
        self.pose_msg.pose.orientation.w = orr.GetReal()

        self.gps_pub.publish(self.pose_msg)
