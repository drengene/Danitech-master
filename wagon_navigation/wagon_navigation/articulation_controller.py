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
from copy import deepcopy
from time import sleep
import pickle
import datetime

import numpy as np


FORWARDS = 1
BACKWARDS = -1
MAX_VEL = 2

class articulationController(Node):
    
    def __init__(self, waypoints=None):
        super().__init__('articulation_controller')

        # Declare parameters related to topic names
        self.declare_parameter('cmd_vel_topic', '/cmd_vel', ParameterDescriptor(description="Command velocity topic name"))
        self.declare_parameter('base_link_frame', 'base_link', ParameterDescriptor(description="Base link frame name"))
        self.declare_parameter('odom_frame', 'odom', ParameterDescriptor(description="Odometry frame name"))
        self.declare_parameter('world_frame', 'odom', ParameterDescriptor(description="World frame name"))
        self.declare_parameter('path_update_rate', 2, ParameterDescriptor(description="Update rate in Hz of the path generation"))
        self.declare_parameter('manual_waypoints', False, ParameterDescriptor(description="If true, waypoints are manually set by the user"))

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        # self.joint_state_topic = self.get_parameter('joint_state_topic').value
        self.base_link_frame = self.get_parameter('base_link_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        self.path_update_rate = self.get_parameter('path_update_rate').value
        manual = self.get_parameter('manual_waypoints').value

        # declare parameters from the parameters above
        self.base_poses = np.array([0, 0, 0])
        self.base_orrientations = np.array([0, 0, 0, 0])
        self.twist_msgs = np.array([0, 0])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.secs = 0
        self.nanosecs = 0
        self.old_dist = np.inf
        self.waypoints = None
        self.goal_position = None
        self.waypoint_index = 0
        self.base_link_position = None
        self.global_plan_points = None
        self.wayposes = None
        if waypoints is None:
            if manual:
                # self.waypoints = np.array([[8.50931,3.333,0.0],[10.20931,2.333,0.0], [14.20931,6.333,0.0], [20.20931,6.333,0.0], [26.20931,6.333,0.0], [28.20931,2.533,0.0], [30.20931,2.533,0.0], [32.20931,6.533,0.0]])
                self.waypoints = np.array([[6.314, 1.616, 0.0], [13.768, 1.616, 0.0], [19.326, -1.1029, 0.0], [20.02, -4.45, 0.284], [19.75, -11.98, 1.665], [19.285, -18.48, 2.84], [18.45, -23.215, 3.215], [11.90, -24.39, 3.215],[-3.45, -22.88, 3.215],[-3.25, -17.00, 2.59], [-3.93, -9.92, 1.28], [-3.85, -3.28, 0.07], [1.726, 0.75, 0.0], [5.34, 2.355, 0.0]])
                self.setup_sub_pub()
                self.gen_init_path(self.waypoints)
                exit()
            else:
                print("Waiting for goal position or path")
                self.setup_sub_pub()
                self.wait_for_goal()
        else:
            self.waypoints = waypoints
            
    def wait_for_goal(self):

        if np.any(self.goal_position):
            print("Goal position received: ", self.goal_position)
            self.gen_init_path(np.array([self.goal_position]))
        elif np.any(self.waypoints):
            self.gen_init_path(self.waypoints)
            print("Wayposes received: ")
            return
        sleep(0.5)
        return



    def gen_init_path(self, _waypoints):

        self.og_waypoints = deepcopy(_waypoints)


        while self.base_link_position is None:
            # Spin once
            rclpy.spin_once(self, timeout_sec=0.1)
        _waypoints = np.insert(_waypoints, 0, self.base_link_position, axis=0)


        theta, direction = self.angle_between_base_point(self.base_link_position, self.base_link_orientation, _waypoints[1], get_direction=True)
        if direction == BACKWARDS: # Could also be done by placing point behind robot for the spline, but still going to the original next point
            # create a new point 2 meters in front of base_link_position, in relation to the base_link_orientation
            _waypoints = np.insert(_waypoints, 1, [self.base_link_position[0] + 5 * np.cos(self.base_link_orientation[2]),
                                                    self.base_link_position[1] + 5 * np.sin(self.base_link_orientation[2]),
                                                      self.base_link_position[2]], axis=0)

        else:
            _waypoints = np.insert(_waypoints, 1, [self.base_link_position[0] + 2 * np.cos(self.base_link_orientation[2]),
                                                    self.base_link_position[1] + 2 * np.sin(self.base_link_orientation[2]),
                                                      self.base_link_position[2]], axis=0)

        self.spline = self.gen_spline(_waypoints, 2)

        # create a copy of waypoints with 4 additional dimensions
        self.wayposes = np.zeros((len(self.spline), 7))

        # copy the waypoints into the first 3 dimensions of wayposes        print(q)
        self.wayposes[:,0:3] = self.spline

        # calculate the direction of travel for each waypoint
        for i in range(1, len(self.spline)):
            self.wayposes[i-1,3:7] = self.quaternion_from_two_vectors(self.spline[i-1], self.spline[i])
        

        #print(self.wayposes)   
        # self.next_position = np.array([8.20931,3.333,0.0])
        self.send_wayposes(self.wayposes)

        self.waypoints = self.spline

        

        # self.direction = FORWARDS


    def setup_sub_pub(self):
        # self.tf_topic = self.create_subscription(TFMessage, 'tf', self.tf_callback, 10)
        # self.tf_topic  # prevent unused variable warning
        # self.base_link_pose_sub = self.create_subscription(Odometry, "/odometry/local", self.base_link_pose_callback, 10)
        self.goal_pose_sub = self.create_subscription(PoseStamped, "/goal_pose", self.goal_pose_callback, 10)
        self.twist_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.path_subscriber = self.create_subscription(PoseArray, "/path", self.path_callback, 10)
        self.clock_sub = self.create_subscription(Clock, "/clock", self.clock_callback, 10)
        self.global_plan_sub = self.create_subscription(PoseArray, "/global_plan", self.global_plan_callback, 10)
        self.update_timer = self.secs + self.nanosecs * 1e-9
        
        # spin for a bit to get the first message
        for i in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)


        self.base_link_pose_sub = self.create_subscription(Odometry, "/wagon/base_link_pose_gt", self.base_link_pose_callback, 10)
        #self.base_link_pose_sub = self.create_subscription(Odometry, "/odometry/local", self.base_link_pose_callback, 10)

        self.base_link_pose_sub  # prevent unused variable warning

        rclpy.spin_once(self, timeout_sec=0.1)
        
        
        self.waypose_publisher = self.create_publisher(PoseArray, "/wayposes", 10)

        self.marker_publisher = self.create_publisher(MarkerArray, "/markers", 10)
        self.point_publisher = self.create_publisher(PointStamped, "/points", 10)

        





    def global_plan_callback(self, msg):
        self.global_plan = msg
        print("Global plan received")
        self.waypoints = np.zeros((len(msg.poses), 3))

        for i, pose in enumerate(msg.poses):
            self.waypoints[i,:] = [pose.position.x, pose.position.y, pose.position.z]

        print(np.shape(self.waypoints))
        self.destroy_subscription(self.global_plan_sub)

    def clock_callback(self, msg):
        self.secs = msg.clock.sec
        self.nanosecs = msg.clock.nanosec
        self.time = self.secs + self.nanosecs * 1e-9
        # print("Time: " + str(self.time))


    def path_callback(self, msg):
        self.path = msg
        # print("Path received")

    def goal_pose_callback(self, msg):
        # self.goal_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.goal_position = np.array([msg.pose.position.x, msg.pose.position.y, self.base_link_position[2]])
        self.goal_orientation = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        print("goal pose updated", self.goal_position)


    def base_link_pose_callback(self, msg):
        self.base_link_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        self.base_link_orientation = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        # print("pose updated", self.base_link_position)
        self.base_poses = np.vstack([self.base_poses, self.base_link_position])
        self.base_orrientations = np.vstack([self.base_orrientations, self.base_link_orientation])
        #print(self.base_poses)

        try: 
            if np.any(self.waypoints) and self.wayposes is not None:
                end = self.control_vehicle2()
                elapsed_time = self.time - self.update_timer

                if end:
                    self.goal_position = None
                    self.waypoints = None
                    self.waypoint_index = 0
                    print("waiting for new goal")
                    
                # Run the gen_init_path functino at 2 hz
                # elif elapsed_time > self.path_update_rate:
                #     print("Updates the new path, from index: ", self.waypoint_index)
                #     self.gen_init_path(self.wayposes[self.waypoint_index:,0:3])
                #     self.waypoint_index = 0

                #     self.update_timer = self.time
            
            else:
                self.wait_for_goal()

            


        except KeyboardInterrupt as e:
            self.send_twist(0.0,0.0)
            print("Keyboard interrupt", e)


        # combine the two np arrays in a single flat array
        # self.base_link_pose = np.concatenate((self.base_link_position, self.base_link_orientation))


    def control_vehicle2(self):

        if np.linalg.norm(self.base_link_position - self.waypoints[-2]) < 0.5:
            print("Reached Final Waypoint, saving to data to file" )
            self.send_twist(0.0,0.0)


            with open("/home/danitech/master_ws/src/Danitech-master/wagon_navigation/wagon_navigation/pose_data/" + str(datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")) + "max_vel_" + str(MAX_VEL) + ".pkl", "wb") as f:
                pickle.dump({'base_pose' : self.base_poses, 'base_or' : self.base_orrientations, 'wayposes' : self.wayposes, 'twist_msgs' : self.twist_msgs, 'max_vel' : MAX_VEL, 'waypoints' : self.waypoints}, f)

            #np.save("/home/danitech/master_ws/src/Danitech-master/wagon_navigation/wagon_navigation/pose_data/" + str(datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")) + ".npy", [self.base_poses, self.base_orrientations, self.wayposes])


            return 1
        
        angle_diff, direction = self.angle_between_base_point(self.base_link_position, self.base_link_orientation, self.waypoints[self.waypoint_index], get_direction=True)
        angle_diff_next = self.angle_between_base_point(self.base_link_position, self.base_link_orientation, self.waypoints[self.waypoint_index+1])

        dist_pose_point = np.linalg.norm(self.base_link_position - self.waypoints[self.waypoint_index])
        dist_pointA_pointB = np.linalg.norm(self.waypoints[self.waypoint_index - 1] - self.waypoints[self.waypoint_index])

        curr_point_weight = dist_pose_point/dist_pointA_pointB

        # create a sigmoid function from dist pose point to dist pointA pointB with a gain of five
        point_weight = 1/(1 + np.exp(10*(-curr_point_weight+0.5)))

        next_point_weight = 1 - point_weight

        if direction == BACKWARDS:
            if len(self.waypoints) < self.waypoint_index + 2:
                lookahead_nums = len(self.waypoints) - self.waypoint_index
            else:
                lookahead_nums = 2
            waypoint_dists = np.linalg.norm(self.base_link_position - self.waypoints[self.waypoint_index:self.waypoint_index + lookahead_nums], axis=1)

            print("waypoint_dists: ", waypoint_dists)
            min_dist_index = np.argmin(waypoint_dists)
            print("min_dist_index: ", min_dist_index, " dist", waypoint_dists[min_dist_index])
            self.waypoint_index += min_dist_index +1
            self.old_dist = np.inf

        else:
            self.old_dist = dist_pose_point
        

        self.send_wayposes(self.wayposes[self.waypoint_index:])
        angle = angle_diff * point_weight + angle_diff_next * next_point_weight

        
        # print("Angle_pi: ", 1-(abs(angle/np.pi)))
        self.send_twist(MAX_VEL * (1-abs(angle/np.pi)), angle*1.5) # Add regulatation if diff is larger than max angle, e.g. lower speed


        return 0


    def send_twist(self, linear_vel, angular_vel):
        # if linear_vel > 1.0:
        #     linear_vel = 1.0
        self.twist_msgs = np.vstack([self.twist_msgs, [linear_vel, angular_vel]])
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self.twist_publisher.publish(twist_msg)

    def send_points(self, pose):
        point = PointStamped()
        point.header.frame_id = self.world_frame
        point.header.stamp.sec = self.secs
        point.header.stamp.nanosec = self.nanosecs
        point.point.x = pose[0]
        point.point.y = pose[1]
        point.point.z = pose[2]
        self.point_publisher.publish(point)


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
            waypose.orientation.x = pose[3]
            waypose.orientation.y = pose[4]
            waypose.orientation.z = pose[5]
            waypose.orientation.w = pose[6]
            # waypose.orientation = self.base_link_pose.orientation
            wayposes_msg.poses.append(waypose)



        self.waypose_publisher.publish(wayposes_msg)
        # self.marker_publisher.publish(marker_msgs)


    def gen_spline(self, points, resolution):
                
        # Concatenate the points to form a 3x3 array
        # points = np.array([p1, p2, p3])

        # Calculate the distances between each pair of points
        distances = np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1))

        # Calculate the cumulative distance along the curve
        cumulative_distances = np.cumsum(distances)
        cumulative_distances = np.insert(cumulative_distances, 0, 0) # Add initial distance of 0

        # Create a cubic spline interpolation of the points
        interp = CubicSpline(cumulative_distances, points, bc_type='not-a-knot')
 
        # Generate points along the curve at the specified resolution 
        # - there are issues if the supplied points are at a higher resolution than the desired output

        print(distances)
        print(np.min(distances))

        s_vals = np.array([])

        if resolution*0.9 <= np.min(distances):
            print("resolution smaller than min distance")
            for idx, dist in enumerate(cumulative_distances[:-1], ):
                num_points = int(np.ceil((cumulative_distances[idx + 1] - dist)/resolution))
                # print(num_points)
                s_val = np.linspace(dist, cumulative_distances[idx + 1], num_points)
                s_vals = np.append(s_vals, s_val[1:])
        else:
            print("resolution larger than min distance")
            intermediate_dist = 0.0
            for idx, dist in enumerate(distances[:-1], ):
                intermediate_dist += dist
                if intermediate_dist >= resolution:
                    s_vals = np.append(s_vals, cumulative_distances[idx])
                    intermediate_dist = 0.0



        # Generate 10 points along the curve
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

            # print(quat)
            return quat


    def angle_between_base_point(self, base_point, base_orr, goal_point, get_direction = False):

        r = R.from_quat(base_orr)
        base_plane_norm = r.apply([0,1,0])
        base_dir_vec = r.apply([1,0,0])

        
        goal_vec = goal_point - base_point


        cos_theta = np.dot(base_plane_norm, goal_vec) / (np.linalg.norm(base_plane_norm) * np.linalg.norm(goal_vec))
        theta = np.arcsin(cos_theta)

        cos_theta_dir = np.dot(base_dir_vec, goal_vec) / (np.linalg.norm(base_dir_vec) * np.linalg.norm(goal_vec))
        
        direction = FORWARDS
        # print(np.arccos(cos_theta_dir))
        if np.abs(np.arccos(cos_theta_dir)) > np.pi/2:
            print("point possibly behind")
            direction = BACKWARDS
            # return np.arcsin(cos_theta) + np.sign(np.arcsin(cos_theta)) * np.pi/2
            theta = np.pi * np.sign(theta) - theta
            
        
 

        if get_direction:
            return theta, direction
        
        return theta
 


    # Create function that saves an np array to a new file
    def save_np_array(self, array, filename):
        np.save(filename, array)


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

