import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import lmao_lib.lidar as Lidar
import lmao_lib.util.pclmao as pclmao
from lmao_lib.mapping import get_normals

from scipy.spatial.transform import Rotation as R

import concurrent.futures

import threading

import time

# Ros
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

# Import odometry message
from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovariance

# Tf2
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener



# Define the class for the raycast localization node
class RaycastLocalization(Node):
	def __init__(self, map, max_range, resolution, max_iterations):
		super().__init__('raycast_localization')
		self.map = map
		self.max_range = max_range
		self.resolution = resolution
		self.max_iterations = max_iterations
		self.lock = threading.Lock()
		self.points = np.empty((0, 6)) # x, y, z, nx, ny, nz, n being the normal
		self.pos = np.array([0, 0, 0])
		self.orientation = np.array([0, 0, 0, 0])

		# Declare parameters
		from rcl_interfaces.msg import ParameterDescriptor
		self.declare_parameter('map_path', "map.ply", ParameterDescriptor(description="Path to the map file"))
		self.declare_parameter('lidar_topic', "/wagon/base_scan/lidar_data", ParameterDescriptor(description="Topic to subscribe to for lidar data"))
		self.declare_parameter('max_range', 50000, ParameterDescriptor(description="Maximum range of the lidar in mm"))
		self.declare_parameter("world_frame", "world", ParameterDescriptor(description="The world frame (origin of the map)"))
		self.declare_parameter("odom_topic", "/odom", ParameterDescriptor(description="Topic to publish odometry data to"))

		# Get parameters
		self.map_path = self.get_parameter("map_path").value
		self.lidar_topic = self.get_parameter("lidar_topic").value
		self.max_range = self.get_parameter("max_range").value
		self.world_frame = self.get_parameter("world_frame").value
		self.odom_topic = self.get_parameter("odom_topic").value


		sub_cb_group = MutuallyExclusiveCallbackGroup()
		timer_cb_group = MutuallyExclusiveCallbackGroup()

		# Create the subscriber
		self.create_subscription(PointCloud2, self.lidar_topic, self.lidar_callback, 10, callback_group=sub_cb_group)

		# Create timer to run function every 10s set a function to run every 10s
		timer_period = 10  # seconds
		self.timer = self.create_timer(timer_period, self.create_mesh, callback_group=timer_cb_group)

		# Print info
		self.get_logger().info("Subscribed to topic: {}".format(self.lidar_topic))


		# Create publisher
		self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
		self.get_logger().info("Publishing odometry data to topic: {}".format(self.odom_topic))
		self.rviz_pub = self.create_publisher(PointCloud2, "/rviz_pcl", 10)
		self.get_logger().info("Publishing point cloud data to topic: {}".format("/rviz_pcl"))

		# Create tf2 buffer and listener
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)

		self.last_call = time.time()


	def lidar_callback(self, msg):
		self.get_logger().info("Received lidar data")

		# Detect time between calls to see if the data is being published at the correct rate
		now = time.time()
		# Print if time between calls is greater than 1s
		if now - self.last_call > 1:
			self.get_logger().info("Time between calls: {}".format(now - self.last_call))
		self.last_call = now


		data = pclmao.extract_PointCloud2_data(msg)
		# x = data["x"], y = data["y"], z = data["z"]
		# Shape of x: (1024, 128, 1)
		xyz = np.dstack((data["x"], data["y"], data["z"]))
		# Shape of xyz: (1024, 128, 3)
		depth = data["range"]
		# Transform the data to the correct position
		try:
			# Get the most recent transform
			t = self.tf_buffer.lookup_transform(self.world_frame, msg.header.frame_id, rclpy.time.Time())
		except TransformException as e:
			self.get_logger().error("Transform error: {}, when transforming from {} to {}".format(e, msg.header.frame_id, self.world_frame))
			return
		
		# extract the rotation and translation components of the transform
		rotation = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
		translation = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
		
		# Construct the transformation matrix
		r = R.from_quat(rotation)
		T = np.eye(4)
		T[:3, :3] = r.as_matrix()
		T[:3, 3] = translation

		#Shape of xyz: (1024, 128, 3)
		# Transform the data
		xyz = np.matmul(xyz, T[:3, :3].T) + T[:3, 3]
		#Shape of xyz: (1024, 128, 3)

		# Get the normals
		normals = get_normals(xyz)
		
		# Shape of xyz: (131072, 3)
		# Flatten array to [-1, 3]
		xyz = xyz.reshape(-1, 3)
		depth = depth.reshape(-1)
		normals = normals.reshape(-1, 3)
		# Remove points that are too far away
		xyz = xyz[depth < self.max_range]
		normals = normals[depth < self.max_range]
		depth = depth[depth < self.max_range]

		# Add points to point
		with self.lock:
			self.points = np.concatenate((self.points, np.hstack((xyz, normals))), axis=0)

			# print size of points
			self.get_logger().info("Shape of points: {}".format(self.points.shape))

		xyz = xyz.astype(np.float32)


		# Create new point cloud object for visualization in rviz
		pc = pclmao.construct_pointcloud2_msg({"x": xyz[:, 0], "y": xyz[:, 1], "z": xyz[:, 2], "range": depth}, height=1, width=xyz.shape[0])
		pc.header.frame_id = self.world_frame
		pc.header.stamp = msg.header.stamp
		self.rviz_pub.publish(pc)
		self.get_logger().info("Published point cloud data to topic: {}".format("/rviz_pcl"))

	def create_mesh(self):
		with self.lock:
			points = self.points.copy()
		print("I'm sleeping for 2 seconds")
		time.sleep(2)
		print("I'm awake")
		# Create mesh from points
		pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points[:, :3]))
		pcd.normals = o3d.utility.Vector3dVector(points[:, 3:])
		self.get_logger().info("Creating mesh from points")
		self.mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=10, n_threads=1)
		self.get_logger().info("Created mesh from points")
		# Show mesh
		o3d.visualization.draw_geometries([self.mesh])


	def timer_callback(self):
		print("I'm sleeping for 2 seconds")
		time.sleep(2)
		print("I'm awake")
		# Submit the task to the executor
		# with self.lock:
		# 	points_copy = self.points.copy()
		# self.create_mesh(points_copy)

	
	def load_map(self, map_path):
		self.map = o3d.io.read_triangle_mesh(map_path)

	def save_map(self, map_path):
		o3d.io.write_triangle_mesh(map_path, self.map)


def main(args=None):
	rclpy.init(args=args)
	localizer = RaycastLocalization(map=None, max_range=10, resolution=0.1, max_iterations=10)
	
	executor = MultiThreadedExecutor()
	executor.add_node(localizer)
	executor.spin()

	print("What happened?")

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	localizer.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

