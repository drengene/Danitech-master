from omni import usd, timeline
import asyncio

from omni.isaac.core import SimulationContext
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import JointState


class joint_state_pub(Node):
    def __init__(self, prim_parent="wagon"):
        super().__init__('isaac_joint_publisher')
        self.stage = usd.get_context().get_stage()                      # Used to access Geometry
        self.sim_context =  SimulationContext()        # Used to interact with simulation
        self.joints = []
        #asyncio.ensure_future(self.create_scenario())

        self.get_joints(self.stage.GetPrimAtPath("/wagon")) # Get all physical revolute joints from prim

        # convert joints to dict with pos and vel
        self.joint_vals = dict.fromkeys(self.joints, [0,0])
        self.joint_names = [joint.GetName() for joint in self.joints]

        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)

    def get_joints(self, prim):

        for child in prim.GetChildren():
            #print(child.GetTypeName())
            if child.GetTypeName() == "PhysicsRevoluteJoint":
                self.joints.append(child)
            elif child.GetTypeName() == "Xform":
                self.get_joints(child) # Recursively get all joints in the scene

        
    def joint_state_callback(self):
        self.get_joint_states()
        joint_msg = JointState()
        sim_time = self.sim_context.current_time
        joint_msg.header.stamp.sec = int(sim_time)
        joint_msg.header.stamp.nanosec = int((sim_time - int(sim_time)) * 1e9)
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.joint_names
        joint_msg.position = [self.joint_vals[joint][0] for joint in self.joints]
        joint_msg.velocity = [self.joint_vals[joint][1] for joint in self.joints]
        self.joint_pub.publish(joint_msg)


    def get_joint_states(self):
        for joint in self.joints:
            
            joint_pos = np.radians(joint.GetAttribute("state:angular:physics:position").Get())
            joint_val = np.radians(joint.GetAttribute("state:angular:physics:velocity").Get()) # A bit unsure if it is rad/s

            self.joint_vals[joint] = [joint_pos, joint_val]