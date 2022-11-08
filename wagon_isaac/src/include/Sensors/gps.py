from omni import usd
from omni.isaac.core import SimulationContext

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, NavSatStatus

class gps_pub(Node):
    def __init__(self, rtk_prim_path="/wagon/rtk_pole"):
        super().__init__('isaac_gps_publisher')
        self.stage = usd.get_context().get_stage()      # Used to access Geometry
        self.sim_context =  SimulationContext()        # Used to interact with simulation
        self.rtk_prim = self.stage.GetPrimAtPath(rtk_prim_path)

        self.gps_pub = self.create_publisher(NavSatFix, "gps", 10)



    def pub_gps_data(self):
        gps_msg = NavSatFix()
        sim_time = self.sim_context.current_time
        gps_msg.header.stamp.sec = int(sim_time)
        gps_msg.header.stamp.nanosec = int((sim_time - int(sim_time)) * 1e9)
        gps_msg.header.frame_id = "gps"
        gps_msg.status.status = NavSatStatus.STATUS_FIX
        gps_msg.status.service = NavSatStatus.SERVICE_GPS
        gps_msg.latitude = float(self.rtk_prim.GetAttribute("xformOp:translate").Get()[0])
        gps_msg.longitude = float(self.rtk_prim.GetAttribute("xformOp:translate").Get()[1])
        gps_msg.altitude = float(self.rtk_prim.GetAttribute("xformOp:translate").Get()[2])

        self.gps_pub.publish(gps_msg)