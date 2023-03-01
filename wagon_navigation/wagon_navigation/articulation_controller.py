import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
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


forwards = 1
backwards = -1

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
        
        self.waypoints = np.array([[8.20931,3.333,-0.206],[9.20931,2.333,-0.206],[10.20931,2.333,-0.206], [14.20931,6.333,-0.206], [16.20931,6.333,-0.206]])

        # self.next_position = np.array([8.20931,3.333,-0.206])
        self.waypoint_index = 0

        self.direction = forwards
        self.setup_sub_pub()
        # self.base_link_position = None

        # while not np.any(self.base_link_position):
        #     self.get_logger().info('Waiting for base_link pose')
        #     rclpy.spin_once(self, timeout_sec=1.0)


        # print(self.angle_between_base_point(self.base_link_position, self.base_link_orientation, self.next_position))
        # base_pos = np.array([0,0,0])
        # orr = np.array([0,0,0,1])
        # end = np.array([4,2,0.5 ])
        # test = self.angle_between_base_point(base_pos, orr, end)

        # print("Angle in deg: ", np.rad2deg(test))

        # Get the topic parameters

        # while self
        # self.control_vehicle()


    def control_vehicle(self):

        angle = self.angle_between_base_point(self.base_link_position, self.base_link_orientation, self.waypoints[self.waypoint_index])
        dist = np.linalg.norm(self.base_link_position - self.waypoints[self.waypoint_index])

        if np.abs(angle) > 0.1:
            self.send_twist(0.5*dist*self.direction, angle)
            angle = self.angle_between_base_point(self.base_link_position, self.base_link_orientation, self.waypoints[self.waypoint_index])
            # print("Angle in deg: ", np.rad2deg(angle))
            # rclpy.spin_once(self, timeout_sec=0.1)
            # return


        if dist > 0.5 and np.abs(angle) < 0.1:
            self.send_twist(0.5*dist*self.direction,0.0)
            # print("Distance: ", dist)
            dist = np.linalg.norm(self.base_link_position - self.waypoints[self.waypoint_index])
            # rclpy.spin_once(self, timeout_sec=0.1)
            # return
        if dist < 0.5:
            print("Reached Waypoint, Distance: ", dist)
            if self.waypoint_index >= len(self.waypoints):
                print("Reached Final Waypoint, quitting, Distance: ", dist)
                self.send_twist(0.0,0.0)
                self.destroy_node()
                return
            else:
                self.waypoint_index += 1



        # self.send_twist(0, 0)

    def send_twist(self, linear_vel, angular_vel):
        if linear_vel > 1.0:
            linear_vel = 1.0
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self.twist_publisher.publish(twist_msg)



    def angle_between_base_point(self, base_point, base_orr, goal_point):

        r = R.from_quat(base_orr)
        base_plane_norm = r.apply([0,1,0])
        base_dir_vec = r.apply([1,0,0])

        
        goal_vec = goal_point - base_point


        cos_theta = np.dot(base_plane_norm, goal_vec) / (np.linalg.norm(base_plane_norm) * np.linalg.norm(goal_vec))
        theta = np.arcsin(cos_theta)

        cos_theta_dir = np.dot(base_dir_vec, goal_vec) / (np.linalg.norm(base_dir_vec) * np.linalg.norm(goal_vec))

        # print(np.arccos(cos_theta_dir))
        if np.abs(np.arccos(cos_theta_dir)) > np.pi/2:
            print("point possibly behind")
            self.direction = backwards
            # return np.arcsin(cos_theta) + np.sign(np.arcsin(cos_theta)) * np.pi/2
            return np.pi * np.sign(theta) - theta
        
        self.direction = forwards

        return theta
 
    def setup_sub_pub(self):
        # self.tf_topic = self.create_subscription(TFMessage, 'tf', self.tf_callback, 10)
        # self.tf_topic  # prevent unused variable warning
        self.base_link_pose_sub = self.create_subscription(PoseStamped, "/base_link_pose", self.base_link_pose_callback, 10)
        self.base_link_pose_sub  # prevent unused variable warning
        self.twist_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)


    def base_link_pose_callback(self, msg):
        self.base_link_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.base_link_orientation = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        # print("pose updated", self.base_link_position)
        try: 
            self.control_vehicle()
        except KeyboardInterrupt as e:
            self.send_twist(0.0,0.0)
            print("Keyboard interrupt", e)
        # combine the two np arrays in a single flat array
        # self.base_link_pose = np.concatenate((self.base_link_position, self.base_link_orientation))


    def tester(self):
        self.get_logger().info('I heard: "%s"' % self.base_link_position)
        # self.base_link_pose = msg.transforms[0].transform.translation
    def tf_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.transforms[0].header.frame_id)
        self.base_link_pose = msg.transforms[0].transform.translation


    

def stop_on_shutdown():
    # rclpy.init()

    stopper = rclpy.create_node("stopper")
    twist_publisher = stopper.create_publisher(Twist, "/cmd_vel", 10)
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    twist_msg.angular.z = 0.0
    twist_publisher.publish(twist_msg)



def main():
    # Creater a try except that will destroy the node if an exception is raised

    rclpy.init()
    controller = articulationController()
    # Create test values

    try:

        rclpy.spin(controller)
        # Destroy the node explicitly

    except ExternalShutdownException as e:
        print(e)
    # finally:
    #     controller.send_twist(0.0, 0.0)
    #     rclpy.spin_once(controller, timeout_sec=0.5)
    #     controller.destroy_node()
    #     rclpy.shutdown()

