from omni import usd
import omni.isaac.core as core
# from omni.isaac.core import SimulationContext, get_relative_transform

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry

class pose_pub(Node):
    def __init__(self, pose_prim_path="/wagon/base_link", frame_id="base_link"):
        super().__init__('isaac_' + frame_id + '_publisher')
        self.stage = usd.get_context().get_stage()      # Used to access Geometry
        # self.transform_utils = core.get_transform_utils()  # Used to access Geometry
        self.sim_context =  core.SimulationContext()        # Used to interact with simulation
        self.pose_prim = self.stage.GetPrimAtPath(pose_prim_path)
        self.world_prim = self.stage.GetPrimAtPath("/world")
        self.frame_id = frame_id
        self.odom_pub = self.create_publisher(Odometry, pose_prim_path + "_pose_gt", 10)

        self.pose_msg = PoseWithCovariance()
        self.twist_msg = TwistWithCovariance()



    def ros_pub(self):
        sim_time = self.sim_context.current_time
        odom_msg = Odometry()

        odom_msg.header.stamp.sec = int(sim_time)
        odom_msg.header.stamp.nanosec = int((sim_time - int(sim_time)) * 1e9)
        odom_msg.header.frame_id = self.frame_id
        odom_msg.child_frame_id = "base_link"
        pos = self.pose_prim.GetAttribute("xformOp:translate").Get()
        orr = self.pose_prim.GetAttribute("xformOp:orient").Get()
        orr_imag = orr.GetImaginary()

        self.pose_msg.pose.position.x = float(pos[0])
        self.pose_msg.pose.position.y = float(pos[1])
        self.pose_msg.pose.position.z = float(pos[2]) + 0.20686
        self.pose_msg.pose.orientation.x = float(orr_imag[0])
        self.pose_msg.pose.orientation.y = float(orr_imag[1])
        self.pose_msg.pose.orientation.z = float(orr_imag[2])
        self.pose_msg.pose.orientation.w = orr.GetReal()

        self.pose_msg.covariance = np.zeros(36).tolist()
        self.pose_msg.covariance[0::7] = np.ones(6)*1e-3 # set the diagonal to 1e-3
        # set the diagonal to 1e-3

        # Get the twist 
        twist_linear = self.pose_prim.GetAttribute("physics:velocity").Get()
        twist_angular = self.pose_prim.GetAttribute("physics:angularVelocity").Get()
        self.twist_msg.twist.linear.x = float(twist_linear[0])
        self.twist_msg.twist.linear.y = float(twist_linear[1])
        self.twist_msg.twist.linear.z = float(twist_linear[2])
        self.twist_msg.twist.angular.x = np.deg2rad(float(twist_angular[0]))
        self.twist_msg.twist.angular.y = np.deg2rad(float(twist_angular[1]))
        self.twist_msg.twist.angular.z = np.deg2rad(float(twist_angular[2]))

        self.twist_msg.covariance = np.zeros(36).tolist()
        self.twist_msg.covariance[0::7] = np.ones(6)*1e-3 # set the diagonal to 1e-3

        odom_msg.pose = self.pose_msg
        odom_msg.twist = self.twist_msg
        
        self.odom_pub.publish(odom_msg)
        # Set the entire diagnoal in the covariance matrix to 1e-3

        
