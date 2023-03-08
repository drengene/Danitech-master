import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist, PoseStamped, PoseArray, Pose, PointStamped
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
# import tf
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rcl_interfaces.msg import ParameterDescriptor
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import CubicSpline, Rbf


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
        self.waypoints = np.array([[8.50931,3.333,-0.206],[10.20931,2.333,-0.206], [14.20931,6.333,-0.206], [16.20931,6.333,-0.206], [18.20931,6.533,-0.206], [20.20931,6.533,-0.206], [22.20931,6.533,-0.206]])

        self.spline = np.array([self.waypoints[0]])
        for i in range(0, len(self.waypoints)-2, 2):
            splinePoints = self.gen_spline(self.waypoints[i], self.waypoints[i+1], self.waypoints[i+2], 10)
            # print(splinePoints)
            self.spline = np.append(self.spline, splinePoints, axis=0)
 
        # print(self.spline)
        # create a copy of waypoints with 4 additional dimensions
        self.wayposes = np.zeros((len(self.spline), 7))
        # copy the waypoints into the first 3 dimensions of wayposes        print(q)

        self.wayposes[:,0:3] = self.spline

        # calculate the direction of travel for each waypoint
        for i in range(1, len(self.spline)):
            self.wayposes[i-1,3:7] = self.quaternion_from_two_vectors(self.spline[i-1], self.spline[i])

        #print(self.wayposes)   
        # self.next_position = np.array([8.20931,3.333,-0.206])
        self.waypoints = self.spline
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
            waypose.orientation.x = 0.0 #pose[3]
            waypose.orientation.y = 0.0 #pose[4]
            waypose.orientation.z = 0.0 #pose[5]
            waypose.orientation.w = 1.0 #pose[6]
            # waypose.orientation = self.base_link_pose.orientation
            wayposes_msg.poses.append(waypose)

            point = PointStamped()
            point.header.frame_id = self.world_frame
            point.header.stamp.sec = self.secs
            point.header.stamp.nanosec = self.nanosecs
            point.point.x = pose[0]
            point.point.y = pose[1]
            point.point.z = pose[2]
            self.point_publisher.publish(point)

        self.waypose_publisher.publish(wayposes_msg)
        # self.marker_publisher.publish(marker_msgs)

    def gen_spline(self, p1, p2, p3, num_points):
                
        # Concatenate the points to form a 3x3 array
        points = np.array([p1, p2, p3])

        # Calculate the distances between each pair of points
        distances = np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1))

        # Calculate the cumulative distance along the curve
        cumulative_distances = np.cumsum(distances)
        cumulative_distances = np.insert(cumulative_distances, 0, 0) # Add initial distance of 0

        # Create a cubic spline interpolation of the points
        interp = CubicSpline(cumulative_distances, points, bc_type='natural')

        # Generate 10 points along the curve
        s_vals = np.linspace(0, cumulative_distances[-1], num_points)
        interp_points = interp(s_vals)
        
        return interp_points
    

    def quaternion_from_two_vectors(self, point1, point2):

            # Normalize the vectors
            vec2 = point2 - point1

            vec1 = np.asarray([1,0,0])
            vec2 = np.asarray(vec2) / np.linalg.norm(vec2)

            # Find the angle between the vectors
            cos_theta = np.dot(vec1, vec2)
            axis = np.cross(vec1, vec2)

            # Compute the quaternion
            quat = np.zeros(4)
            quat[:3] = axis * np.sin(np.arccos(cos_theta) / 2)
            quat[3] = np.cos(np.arccos(cos_theta) / 2)

            print(quat)
            return quat


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

        self.marker_publisher = self.create_publisher(MarkerArray, "/markers", 10)
        self.point_publisher = self.create_publisher(PointStamped, "/points", 10)
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
            self.control_vehicle()
            #self.send_wayposes(self.wayposes)
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

