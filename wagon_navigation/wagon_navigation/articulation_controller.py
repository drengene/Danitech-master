import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist, PoseStamped, PoseArray, Pose
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
        
        self.secs = 0
        self.nanosecs = 0
        self.waypoints = np.array([[8.20931,3.333,-0.206],[9.20931,2.333,-0.206],[10.20931,2.333,-0.206], [14.20931,6.333,-0.206], [16.20931,6.333,-0.206]])
        # create a copy of waypoints with 4 additional dimensions
        self.wayposes = np.zeros((len(self.waypoints), 7))
        # copy the waypoints into the first 3 dimensions of wayposes
        self.wayposes[:,0:3] = self.waypoints

        # calculate the direction of travel for each waypoint
        for i in range(1, len(self.waypoints)):
            self.wayposes[i-1,3:7] = self.quat_between_points(self.waypoints[i-1], self.waypoints[i])

        #print(self.wayposes)
        # self.next_position = np.array([8.20931,3.333,-0.206])
        self.waypoint_index = 0

        self.direction = forwards
        self.setup_sub_pub()
        # self.base_link_position = None


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
                self.send_wayposes(self.waypoints[self.waypoint_index:])




        # self.send_twist(0, 0)

    def send_twist(self, linear_vel, angular_vel):
        if linear_vel > 1.0:
            linear_vel = 1.0
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self.twist_publisher.publish(twist_msg)


    def send_wayposes(self, wayposes):
        wayposes_msg = PoseArray()
        wayposes_msg.header.frame_id = self.world_frame
        wayposes_msg.header.stamp.sec = self.secs
        wayposes_msg.header.stamp.nanosec = self.nanosecs
        for pose in wayposes:
            # print(pose)
            waypose = Pose()
            waypose.position.x = pose[0]
            waypose.position.y = pose[1]
            waypose.position.z = pose[2]
            waypose.orientation.w = pose[3]
            waypose.orientation.x = pose[4]
            waypose.orientation.y = pose[5]
            waypose.orientation.z = pose[6]
            # waypose.orientation = self.base_link_pose.orientation
            wayposes_msg.poses.append(waypose)

        self.waypose_publisher.publish(wayposes_msg)

    def quat_between_points(self, curr_point, next_point):
        # normalize the vectors
        v1 = curr_point / np.linalg.norm(curr_point)
        v2 = next_point / np.linalg.norm(next_point)
        # calculate the cross product
        c = np.cross(v1, v2)
        
        # calculate the dot product
        d = np.dot(v1, v2)

            # calculate the quaternion components
        w = np.sqrt((np.linalg.norm(v1) ** 2) * (np.linalg.norm(v2) ** 2)) + d
        x = c[0]
        y = c[1]
        z = c[2]
        
           # normalize the quaternion
        q = np.array([w, x, y, z])
        # q = q / np.linalg.norm(q)
        print(q)

        return q


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
        self.clock_sub = self.create_subscription(Clock, "/clock", self.clock_callback, 10)
        self.waypose_publisher = self.create_publisher(PoseArray, "/wayposes", 10)
        self.send_wayposes(self.wayposes)



    def clock_callback(self, msg):
        self.secs = msg.clock.sec
        self.nanosecs = msg.clock.nanosec
        # print("Time: " + str(self.time))



    def base_link_pose_callback(self, msg):
        self.base_link_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.base_link_orientation = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        # print("pose updated", self.base_link_position)
        try: 
            #self.control_vehicle()
            self.send_wayposes(self.wayposes)
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

