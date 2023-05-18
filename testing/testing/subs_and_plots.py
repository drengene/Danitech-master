import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline, Rbf
import time



class MinimalSubscriber(Node):
    def __init__(self, file_path=None):
        super().__init__('minimal_subscriber')


        self.setup_subscribers()





    def setup_subscribers(self):
        self.odometry_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odometry_callback,
            10)
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.global_plan_subscriber = self.create_subscription(
            PoseArray,
            'global_plan',
            self.global_plan_callback,
            10)
        
        self.waypose_subscriber = self.create_subscription(
            PoseArray,
            'waypose',
            self.waypose_callback,
            10)
        
        self.joint_controller_subscriber = self.create_subscription(
            JointState,
            'joint_states_controller',
            self.joint_controller_callback,
            10)

    def odometry_callback(self, msg):
        self.odometry = msg
        self.odometry_received = True

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg
        self.cmd_vel_received = True

    def waypose_callback(self, msg):
        self.waypoints = msg
        self.waypoints_received = True

    def joint_controller_callback(self, msg):
        self.joint_controller = msg
        self.joint_controller_received = True

    def global_plan_callback(self, msg):
        self.plan = msg
        self.plan_received = True

    def get_waypoints(self):
        if self.waypoints_received:
            plan = np.array([[pose.position.x, pose.position.y, pose.position.z] for pose in self.waypoints.poses])

            return plan
        else:
            return None
       
    def get_plan(self):
        if self.plan_received:
            # convert the plan to a numpy array
            plan = np.array([[pose.position.x, pose.position.y, pose.position.z] for pose in self.plan.poses])

            return plan
        else:
            return None

def interpolate_points(points, num_points):
            
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
    s_vals = np.linspace(cumulative_distances[0], cumulative_distances[-1], num_points)

    # for idx, dist in enumerate(cumulative_distances[:-1], ):
    #     num_points = int(np.ceil((cumulative_distances[idx + 1] - dist)/resolution))
    #     # print(num_points)
    #     s_val = np.linspace(dist, cumulative_distances[idx + 1], num_points)
    #     s_vals = np.append(s_vals, s_val[1:])

    # Generate 10 points along the curve
    interp_points = interp(s_vals)

    return interp_points


def scatter_plot(base_pose, wayposes):


    # create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # plot the base poses as red circles
    ax.scatter(base_pose[:,0], base_pose[:,1], base_pose[:,2], c='r', marker='o')

    # plot the wayposes as numbered blue triangles
    ax.scatter(wayposes[:,0], wayposes[:,1], wayposes[:,2], c='b', marker='^')
    for i, (x, y, z) in enumerate(zip(base_pose[:,0], base_pose[:,1], base_pose[:,2])):
        if i % 100 == 0:
            ax.text(x, y, z, str(i), color="red", fontsize=12)

    # set the axis labels
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    # show the plot
    plt.show()


def poly_fit_plot(wayposes, fig):


    waypose_t = np.arange(wayposes.shape[0])  # simple assumption that data was sampled in regular steps
    way_x = wayposes[:,0]
    way_y = wayposes[:,1]
    way_z = wayposes[:,2]

    # fit a 4th order polynomial to the waypose data
    waypose_fitx = np.polyfit(waypose_t, way_x, 4)
    waypose_fity = np.polyfit(waypose_t, way_y, 4)
    waypose_fitz = np.polyfit(waypose_t, way_z, 4)

    ax = fig.add_subplot(111, projection='3d')
     # set the axis labels
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    # even axes
    ax.set_box_aspect((np.ptp(way_x), np.ptp(way_y), np.ptp(way_z)))  # aspect ratio is 1:1:1 in data space

    # add legend
    ax.legend(['wayposes'])
    # show the plot
    return fig, ax
    


def main(args=None):
    rclpy.init(args=args)
    data = MinimalSubscriber()

    while data.get_plan() is None:
        rclpy.spin_once(data)
        print("Waiting for plan")
        time.sleep(0.1)

    plan = data.get_plan()
    interpolate_points(plan, plan.shape[0] * 10)
    fig = plt.figure()
    fig, ax = poly_fit_plot(plan, fig)
    plt.show()
    # print(data.get_waypoints())