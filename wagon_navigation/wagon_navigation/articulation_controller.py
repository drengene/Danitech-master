import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
# import tf
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rcl_interfaces.msg import ParameterDescriptor
from scipy.spatial.transform import Rotation as R

import numpy as np

class articulationController(Node):
    
    def __init__(self):
        super().__init__('articulation_controller')

        # Declare parameters related to topic names
        self.declare_parameter('cmd_vel_topic', '/cmd_vel', ParameterDescriptor(description="Command velocity topic name"))
        self.declare_parameter('base_link_frame', 'base_link', ParameterDescriptor(description="Base link frame name"))
        self.declare_parameter('odom_frame', 'odom', ParameterDescriptor(description="Odometry frame name"))
        self.declare_parameter('world_frame', 'world', ParameterDescriptor(description="World frame name"))

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        # self.joint_state_topic = self.get_parameter('joint_state_topic').value
        self.base_link_frame = self.get_parameter('base_link_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.world_frame = self.get_parameter('world_frame').value

        # declare parameters from the parameters above

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.setup_sub_pub()

        self.next_position = np.array([1,1,1])

        # Get the topic parameters

        # while self


    def control_vehicle(self):
        beta = self.angle_between_vectors(self.base_link_pose, self.next_position)
        while angle > 0.1:
            self.send_twist(0.2, angle)
            angle = self.angle_between_vectors(self.base_link_pose, self.next_position)
        self.send_twist(0.15,0)

    def send_twist(self, linear_vel, angular_vel):
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self.twist_publisher.publish(twist_msg)

    def get_pos_vector(self):
        r = R.from_quat(self.base_link_orientation)
        orientation_vector = r.apply([1,0,0]) 


    def angle_between_vectors(self, a, b):
        magnitude_a = np.linalg.norm(a)
        magnitude_b = np.linalg.norm(b)
        cos_theta = np.dot(a, b) / (magnitude_a * magnitude_b)
        theta = np.arccos(cos_theta)
        return theta
 
    def setup_sub_pub(self):
        # self.tf_topic = self.create_subscription(TFMessage, 'tf', self.tf_callback, 10)
        # self.tf_topic  # prevent unused variable warning
        self.base_link_pose_sub = self.create_subscription(PoseStamped, "base_link_pose", self.base_link_pose_callback, 10)
        self.twist_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)


    def base_link_pose_callback(self, msg):
        self.base_link_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.base_link_orientation = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.base_link_pose = np.array([self.base_link_position, self.base_link_orientation]).reshape(7,1)

    def tf_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.transforms[0].header.frame_id)
        self.base_link_pose = msg.transforms[0].transform.translation


    


def main():

    rclpy.init()
    controller = articulationController()
    # Create test values

    rclpy.spin(controller)
    # Destroy the node explicitly
    controller.destroy_node()
    rclpy.shutdown()