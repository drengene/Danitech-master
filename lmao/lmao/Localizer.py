import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import lmao_lib.lidar as Lidar
import lmao_lib.util.pclmao as pclmao
from lmao_lib.mapping import get_normals

from scipy.spatial.transform import Rotation as R


from multiprocessing import Process, Queue

import threading

import time

# Ros
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# Import odometry message
from nav_msgs.msg import Odometry

# Tf2
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

#Import cv2
import cv2

# Create window for cv2
cv2.namedWindow("Normals", cv2.WINDOW_NORMAL)


class Localizer(Node):
	def __init__(self):
		# Init node
		super().__init__('localizer')
		self.get_logger().info("Localizer node started")

		# Declare parameters
		from rcl_interfaces.msg import ParameterDescriptor
		self.declare_parameter('map_path', '/home/junge/Documents/mesh_map/map.ply', ParameterDescriptor(description="Path to the map file"))
		self.declare_parameter('lidar_topic', "/wagon/base_scan/lidar_data", ParameterDescriptor(description="Topic to subscribe to for lidar data"))
		self.declare_parameter('max_range', 50000, ParameterDescriptor(description="Maximum range of the lidar in mm"))
		self.declare_parameter('min_range', 2300, ParameterDescriptor(description="Minimum range of the lidar in mm"))
		self.declare_parameter("world_frame", "world", ParameterDescriptor(description="The world frame (origin of the map)"))
		self.declare_parameter("odom_topic", "odom", ParameterDescriptor(description="Topic to publish odometry data to"))

		# Get parameters
		self.map_path = self.get_parameter("map_path").value
		self.lidar_topic = self.get_parameter("lidar_topic").value
		self.max_range = self.get_parameter("max_range").value
		self.min_range = self.get_parameter("min_range").value
		self.world_frame = self.get_parameter("world_frame").value
		self.odom_topic = self.get_parameter("odom_topic").value

		# Load map
		self.map = o3d.io.read_point_cloud(self.map_path)

		# Create tf2 buffer and listener
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)

				# Create publisher
		self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
		self.get_logger().info("Publishing odometry data to topic: {}".format(self.odom_topic))

		# Create the subscriber
		self.create_subscription(PointCloud2, self.lidar_topic, self.lidar_callback, 10)
		self.get_logger().info("Subscribed to topic: {}".format(self.lidar_topic))



	def lidar_callback(self, msg):
		# Attempt to get transform at time of message, otherwise get most recent
		try:
			# Get the true transform at the time of the message
			t = self.tf_buffer.lookup_transform(self.odom_topic, msg.header.frame_id, msg.header.stamp)
		except TransformException as e:
			# self.get_logger().warn("Transform error: {}, when transforming from {} to {}\n Trying most recent".format(e, msg.header.frame_id, self.world_frame))
			t = None
			
		if t is None:
			try:
				# Get the most recent transform
				t = self.tf_buffer.lookup_transform(self.odom_topic, msg.header.frame_id, rclpy.time.Time())
			except TransformException as e:
				self.get_logger().error("Transform error: {}, when transforming from {} to {}".format(e, msg.header.frame_id, self.world_frame))
				return

		data = pclmao.extract_PointCloud2_data(msg)
		# x = data["x"], y = data["y"], z = data["z"]
		# Shape of x: (1024, 128, 1)
		xyz = np.dstack((data["x"], data["y"], data["z"]))
		# Shape of xyz: (1024, 128, 3)
		depth = data["range"]

		# Show data with matplotlib

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
		normals = get_normals(xyz).transpose(1, 0, 2)

		# Show normals as 2d image in opencv that updates as new data comes in
		# Shape of normals: (1024, 128, 3)
		# Convert to 2d image

		cv2.imshow("Normals", abs(normals))
		cv2.waitKey(1)
		print("Drawing normals")


				
		# Shape of xyz: (131072, 3)
		# Flatten array to [-1, 3]
		xyz = xyz.reshape(-1, 3)
		depth = depth.reshape(-1)
		normals = normals.reshape(-1, 3)

		# Show two 


		# Remove points that are too far away or are zero
		mask = np.logical_and(self.min_range < depth, depth < self.max_range)
		xyz = xyz[mask]
		normals = normals[mask]
		depth = depth[mask]


def main(args=None):
	rclpy.init(args=args)

	localizer = Localizer()

	rclpy.spin(localizer)

	localizer.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()