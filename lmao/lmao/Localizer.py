import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import lmao_lib.lidar as Lidar
import lmao_lib.world as World
import lmao_lib.util.pclmao as pclmao
from lmao_lib.mapping import get_normals

from scipy.spatial.transform import Rotation as R
import scipy.ndimage as ndimage


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
cv2.namedWindow("Virtual normals", cv2.WINDOW_NORMAL)
cv2.namedWindow("Probabilities", cv2.WINDOW_NORMAL)

class Localizer(Node):
	def __init__(self):
		# Init node
		super().__init__('localizer')
		self.get_logger().info("Localizer node started")

		# Declare parameters
		from rcl_interfaces.msg import ParameterDescriptor
		self.declare_parameter('map_path', '/home/junge/Documents/mesh_map/map.ply', ParameterDescriptor(description="Path to the map file"))
		self.declare_parameter('lidar_topic', "/wagon/base_scan/lidar_data", ParameterDescriptor(description="Topic to subscribe to for lidar data"))
		self.declare_parameter('max_range', 90000, ParameterDescriptor(description="Maximum range of the lidar in mm"))
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
		self.world = World.World(self.map_path)
		#points = self.world.get_random_points(100)
		self.particles = self.world.get_probable_random_points(1000)
		# Create random orientations for the particles with a uniform distribution for the yaw, but normal distribution for the pitch and roll
		yaw = np.random.uniform(0, 2*np.pi, self.particles.shape[0])
		pitch = np.random.normal(0, np.pi/20, self.particles.shape[0])
		roll = np.random.normal(0, np.pi/20, self.particles.shape[0])
		# Display roll pitch and yaw in 3 graphs
		plt.subplot(1, 3, 1)
		plt.hist(roll, bins=50)
		plt.title("Roll")
		plt.subplot(1, 3, 2)
		plt.hist(pitch, bins=50)
		plt.title("Pitch")
		plt.subplot(1, 3, 3)
		plt.hist(yaw, bins=50)
		plt.title("Yaw")
		plt.show()

		# Convert the particles to quaternions
		quats = R.from_euler("xyz", np.vstack((roll, pitch, yaw)).T).as_quat()
		self.rotations = R.from_quat(quats)

		# Convert quats to a direction vector, by rotating [1, 0, 0]
		dir_vecs = R.from_quat(quats).apply([1, 0, 0])

		# Create pcd from the particles
		pcd = o3d.geometry.PointCloud()
		pcd.points = o3d.utility.Vector3dVector(self.particles)
		pcd.normals = o3d.utility.Vector3dVector(dir_vecs)
		pcd.paint_uniform_color([0, 0, 1])
		o3d.visualization.draw_geometries([pcd, self.world.world], point_show_normal=True)
		
		# Add the quaternions to the particles
		self.particles = np.hstack((self.particles, quats))

		print("Particles shape: {}".format(self.particles.shape))

		self.get_logger().info("Loaded map from: {}".format(self.map_path))
		self.lidar = Lidar.Virtual_Lidar(offset=+3*np.pi/2)
		self.get_logger().info("Created lidar object")

		# Create tf2 buffer and listener
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)

				# Create publisher
		self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
		self.get_logger().info("Publishing odometry data to topic: {}".format(self.odom_topic))

		# Create the subscriber
		self.create_subscription(PointCloud2, self.lidar_topic, self.lidar_callback, 10)
		self.get_logger().info("Subscribed to topic: {}".format(self.lidar_topic))


	def get_lidar_rays(self, rays):
		new_rays = np.zeros((self.particles.shape[0], 6))
		#new_directions = self.rotations.apply(rays)
		# Rotate the ray directions to the particles
		new_rays[:, :3] = self.rotations.apply(rays)
		# Create a new origo for each ray, using the particles as origo
		new_origo = np.repeat(self.particles[:, :3], rays.shape[0], axis=0)
		# Add the new origo to the new directions
		new_rays[:, 3:] = new_origo
		new_rays[:, :3] += new_origo
		return new_rays



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

		# # extract the rotation and translation components of the transform
		rotation = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
		translation = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
		
		# # Construct the transformation matrix
		# r = R.from_quat(rotation)
		# T = np.eye(4)
		# T[:3, :3] = r.as_matrix()
		# T[:3, 3] = translation

		# #Shape of xyz: (1024, 128, 3)
		# # Transform the data
		# xyz = np.matmul(xyz, T[:3, :3].T) + T[:3, 3]


		#Shape of xyz: (1024, 128, 3)
		xyz = np.rot90(xyz, 1, (1, 0))
		depth = np.rot90(depth, 1, (1, 0))

		# Get the normals
		normals = get_normals(xyz)

		# Calculate gradient of depth
		depth_gradient = np.gradient(depth, axis=(0, 1))
		depth_gradient = np.abs(depth_gradient[0]) + np.abs(depth_gradient[1])

		# Find all places where depth outside min and max range
		rows, cols, _ = np.where((depth < self.min_range) | (depth > self.max_range))

		normals[rows, cols, :] = 0
		depth[rows, cols] = 0
		#normals = np.rot90(normals, 1, (1, 0))

		# Get the indices of the points with depth != 0, meaning they are within the min and max range
		nonzero_indices = np.nonzero(depth[:,:,0])

		# Select at random 100 points with where depth != 0
		# Using depth values as probabilities
		nonzero_values = depth[nonzero_indices]
		inverse_values = 1.0 / nonzero_values
		probabilities = (inverse_values / np.sum(inverse_values)).ravel()

		# Show probabilities as 2d image in opencv that updates as new data comes in
		# We will have to map the probabilities to a 2d image
		probabilities2 = np.zeros((128, 1024))
		probabilities2[nonzero_indices[0], nonzero_indices[1]] = probabilities
		# Scale the probabilities to 0-1
		dist_probabilities = (probabilities2 / np.max(probabilities2))

		# cv2.imshow("Probabilities", dist_probabilities)
		# Cap gradient between -1 and 1
		
		# Invert depth gradient, such that lower values are more likely to be selected
		depth_gradient = 1.0 / depth_gradient

		# Set nan to 0
		depth_gradient[np.isinf(depth_gradient)] = 0

		depth_gradient = np.clip(depth_gradient, 0.0, 1)

		# Apply gaussian filter to depth gradient
		depth_gradient = ndimage.gaussian_filter(depth_gradient, sigma=1.0)
		
		#plt.imshow(depth_gradient)
		#plt.show()


		#Scale depth gradient to 0-1
		depth_gradient = (depth_gradient / np.max(depth_gradient))
		cv2.imshow("Probabilities", depth_gradient)



		random_nonzero_indices = np.random.choice(len(nonzero_indices[0]), size=100, replace=False, p=probabilities)
		selected_indices = (nonzero_indices[0][random_nonzero_indices], nonzero_indices[1][random_nonzero_indices])
		
		normals[selected_indices[0], selected_indices[1], : ] = 1

		# Show normals as 2d image in opencv that updates as new data comes in
		# Shape of normals: (1024, 128, 3)
		# Convert to 2d image
		cv2.imshow("Normals", abs(normals))

		# Create similar image from virtual world and lidar
		rays = self.lidar.rotate_rays(rotation)
		rays += [translation[0], translation[1], translation[2], 0, 0, 0]

		raycast = self.world.cast_rays(rays)

		vdepth = raycast['t_hit'].numpy()
		vnormals = raycast['primitive_normals'].numpy()

		vnormals[rows, cols, :] *= 0.2

		# Show normals as 2d image in opencv that updates as new data comes in
		cv2.imshow("Virtual normals", abs(vnormals))
		cv2.waitKey(1)

				
		# Shape of xyz: (131072, 3)
		# Flatten array to [-1, 3]
		xyz = xyz.reshape(-1, 3)
		depth = depth.reshape(-1)
		normals = normals.reshape(-1, 3)


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