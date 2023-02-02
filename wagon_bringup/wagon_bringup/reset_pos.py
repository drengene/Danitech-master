## Calls localization service to reset position

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

reset_pos = PoseWithCovarianceStamped()
reset_pos.header.frame_id = 'base_link'
