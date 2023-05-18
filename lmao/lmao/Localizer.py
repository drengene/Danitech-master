import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import lmao_lib.lidar as Lidar
import lmao_lib.world as World
import lmao_lib.util.pclmao as pclmao
from lmao_lib.mapping import get_normals

import os

from scipy.spatial.transform import Rotation as R
import scipy.ndimage as ndimage
from sklearn.metrics.pairwise import paired_cosine_distances


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
from geometry_msgs.msg import TransformStamped, PoseStamped, Transform
from builtin_interfaces.msg import Time

# Tf2
import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster

#Import cv2
import cv2

SAVEFIGS = False
VIRTUAL_PLAYBACK = False

# Create window for cv2
cv2.namedWindow("Normals", cv2.WINDOW_NORMAL)
cv2.namedWindow("Virtual normals", cv2.WINDOW_NORMAL)
cv2.namedWindow("Probabilities", cv2.WINDOW_NORMAL)
cv2.namedWindow("Lidar", cv2.WINDOW_NORMAL)
cv2.namedWindow("Dummy Lidar", cv2.WINDOW_NORMAL)


class Localizer(Node):
	def __init__(self):
		np.seterr(divide='ignore')
		np.seterr(invalid='ignore')
		# Init node
		super().__init__('localizer')
		self.get_logger().info("Localizer node started")

		# Declare parameters
		from rcl_interfaces.msg import ParameterDescriptor
		self.declare_parameter('map_path', '/home/junge/Documents/mesh_map/easter_island_boy.ply', ParameterDescriptor(description="Path to the map file"))
		#self.declare_parameter('map_path', "/home/danitech/Documents/maps/easter_island_boy.ply", ParameterDescriptor(description="Path to the map file"))
		self.declare_parameter('lidar_topic', "/wagon/base_scan/lidar_data", ParameterDescriptor(description="Topic to subscribe to for lidar data"))
		self.declare_parameter('max_range', 90000, ParameterDescriptor(description="Maximum range of the lidar in mm"))
		self.declare_parameter('min_range', 2300, ParameterDescriptor(description="Minimum range of the lidar in mm"))
		self.declare_parameter("world_frame", "world", ParameterDescriptor(description="The world frame (origin of the map)"))
		self.declare_parameter("odom_topic", "/wagon/base_link_odom_gt", ParameterDescriptor(description="Topic where odometry data is published"))

		# Get parameters
		self.map_path = self.get_parameter("map_path").value
		self.lidar_topic = self.get_parameter("lidar_topic").value
		self.max_range = self.get_parameter("max_range").value
		self.min_range = self.get_parameter("min_range").value
		self.world_frame = self.get_parameter("world_frame").value
		self.odom_topic = self.get_parameter("odom_topic").value

		# Create tf2 buffer and listener
		self.tf_buffer = Buffer()
		self.tf_listener = TransformListener(self.tf_buffer, self)
		self.tf_broadcaster = TransformBroadcaster(self)
		
		# Create publisher
		self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
		self.get_logger().info("Publishing odometry data to topic: {}".format(self.odom_topic))

		# Create the subscriber
		self.create_subscription(PointCloud2, self.lidar_topic, self.lidar_callback, 10)
		self.get_logger().info("Subscribed to topic: {}".format(self.lidar_topic))
		
		# Load map
		self.world = World.World(self.map_path)
		self.get_logger().info("Loaded map from: {}".format(self.map_path))
		#points = self.world.get_random_points(100)
		
		# Create lidar object
		self.lidar = Lidar.Virtual_Lidar(offset=3*(np.pi/2))
		self.dummy_lidar = Lidar.Virtual_Lidar(offset=3*(np.pi/2))
		self.get_logger().info("Created lidar object")

		# Create particles
		self.init_particles(1000, visualize=False)

		self.viz = o3d.visualization.Visualizer()
		self.viz.create_window()
		self.viz.add_geometry(self.world.world)
		#self.viz.update_geometry()
		#self.viz.poll_events()
		#self.viz.update_renderer()

		self.particle_pcd = o3d.geometry.PointCloud()
		self.average_pcd = o3d.geometry.PointCloud()
		# self.mesh_average_arrow = o3d.geometry.TriangleMesh()
		# self.mesh_average_arrow = self.mesh_average_arrow.create_arrow(1.0, 1.5, 5.0, 4.0, 20, 4, 1)


		self.particle_pcd.points = o3d.utility.Vector3dVector(self.particles[:, :3])
		dir_vecs = self.rotations.apply(np.array([[1, 0, 0]]))
		self.particle_pcd.normals = o3d.utility.Vector3dVector(dir_vecs)
		self.viz.add_geometry(self.particle_pcd)
		self.viz.add_geometry(self.average_pcd)
		self.render_option = self.viz.get_render_option()
		self.render_option.point_show_normal = True

		# Create timer to update the visualization every 0.1 seconds
		self.viz_timer = self.create_timer(0.1, self.viz_loop)

		# Subscribe to the odometry topic
		self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
		self.get_logger().info("Subscribed to topic: {}".format(self.odom_topic))
		self.stamp = None

		self.particle_learning_rate = 0.05
		self.particle_full_resample_rate = 0.0
		self.particles_to_keep = 0.2

		self.clock = None

		self.counter = 0

	def viz_loop(self):
		self.particle_pcd.points = o3d.utility.Vector3dVector(self.particles[:, :3])
		dir_vecs = self.rotations.apply(np.array([[1, 0, 0]]))
		self.particle_pcd.normals = o3d.utility.Vector3dVector(dir_vecs)
		colors = np.zeros((self.particles.shape[0], 3))
		colors[:, 1] = self.probabilities / np.max(self.probabilities)
		colors[:, 0] = 1 - (self.probabilities / np.max(self.probabilities))
		self.particle_pcd.colors = o3d.utility.Vector3dVector(colors)

		if self.clock is None:
			return

		avg_pos, avg_rot = self.get_centroid()

		# Get transform from base_scan to odom. This is the transform from the lidar to the odom zero
		try:
			t = self.tf_buffer.lookup_transform("base_scan", "odom", self.clock)
		except TransformException as e:
			self.get_logger().error("Transform error: {}, when transforming from {} to {}".format(e, "base_scan", "odom"))
			return
		
		BS2O = np.eye(4)
		BS2O[:3, :3] = R.from_quat([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]).as_matrix()
		BS2O[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]

		# Create transformation matrix of avg_pos and avg_rot
		M2BS = np.eye(4)
		M2BS[:3, :3] = avg_rot.as_matrix()
		M2BS[:3, 3] = avg_pos
		

		# Calculate the combined transform from map to odom
		M2O = np.matmul(BS2O, M2BS)

		# Create TransformStamped of W2O
		M2O_msg = TransformStamped()
		M2O_msg.header.stamp = self.clock
		M2O_msg.header.frame_id = "map"
		M2O_msg.child_frame_id = "odom"
		M2O_msg.transform.translation.x = M2O[0, 3]
		M2O_msg.transform.translation.y = M2O[1, 3]
		M2O_msg.transform.translation.z = M2O[2, 3]
		quat = R.from_matrix(M2O[:3, :3]).as_quat()
		M2O_msg.transform.rotation.x = quat[0]
		M2O_msg.transform.rotation.y = quat[1]
		M2O_msg.transform.rotation.z = quat[2]
		M2O_msg.transform.rotation.w = quat[3]
		
		# Publish the transform
		self.tf_broadcaster.sendTransform(M2O_msg)
		self.get_logger().info("Published transform from map to odom")

		#self.send_transform(avg_pos, avg_rot)
		self.average_pcd.points = o3d.utility.Vector3dVector(np.array([avg_pos]))
		self.average_pcd.normals = o3d.utility.Vector3dVector(avg_rot.apply(np.array([[1, 0, 0]])))

		# self.viz.update_geometry(self.particle_pcd)
		self.viz.update_geometry(self.average_pcd)
		self.viz.poll_events()
		self.viz.update_renderer()

		# self.mesh_average_arrow.rotate(avg_rot.inv().as_matrix, center=avg_pos)


	def send_transform(self, translation, rotation):
		t = TransformStamped()
		t.header.stamp = self.clock
		t.header.frame_id = "base_link"
		t.child_frame_id = "map"

		# Create transformation matrix
		transform = np.eye(4)
		transform[:3, :3] = rotation.as_matrix()
		transform[:3, 3] = translation
		# Compute inverse transformation
		transform = np.linalg.inv(transform)
		# Extract inverse translation and rotation
		t.transform.translation.x = transform[0, 3]
		t.transform.translation.y = transform[1, 3]
		t.transform.translation.z = transform[2, 3]
		quat = R.from_matrix(transform[:3, :3]).as_quat()
		t.transform.rotation.x = quat[0]
		t.transform.rotation.y = quat[1]
		t.transform.rotation.z = quat[2]
		t.transform.rotation.w = quat[3]

		# Publish transform
		self.tf_broadcaster.sendTransform(t)
		

	def odom_callback(self, msg):
		if self.stamp is None:
			self.stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
			return
		#self.clock = msg.header.stamp
		new_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
		dt = new_stamp - self.stamp
		self.stamp = new_stamp
		# print("dt: {}".format(dt))
		linear = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]) * dt
		angular = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]) * dt
		# Apply twist to particles
		for i, r in enumerate(self.rotations):
			if i == 0:
				continue
			self.particles[i, :3] += r.apply(linear)
			ang = r.apply(angular)
			# Concatenate rotation and ang
			rot = R.from_euler('xyz', ang, degrees=False)
			self.rotations[i] = self.rotations[i] * rot
		return
	
	def gt_callback(self, msg):
		if self.stamp is None:
			self.stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
			return
		new_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
		dt = new_stamp - self.stamp
		self.stamp = new_stamp
		# print("dt: {}".format(dt))
		linear = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]) * dt
		angular = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]) * dt
		# Apply twist to particles
		for i, r in enumerate(self.rotations):
			if i == 0:
				continue
			self.particles[i, :3] += linear
			# Angular is the twist in the body frame.
			# We need to rotate it to the world frame
			# before applying it to the particles
			ang = R.from_euler("xyz", angular)
			self.rotations[i] = self.rotations[i] * ang
		return



	def resample_particles(self):
		# Normalize probabilities
		# self.probabilities = self.probabilities / np.sum(self.probabilities)
		# Keep self.particles_to_keep of the particles
		sorted_index = np.argsort(self.probabilities)
		self.n_particles_to_keep = int(self.particles_to_keep * self.n_particles)
		
		# Create new particles
		new_particles = np.zeros_like(self.particles)
		new_particles[:self.n_particles_to_keep] = self.particles[sorted_index[-self.n_particles_to_keep:]]
		# Create new probabilities
		new_probabilities = np.zeros_like(self.probabilities)
		new_probabilities[:self.n_particles_to_keep] = self.probabilities[sorted_index[-self.n_particles_to_keep:]]

		new_rotations = self.rotations
		new_rotations[:self.n_particles_to_keep] = self.rotations[sorted_index[-self.n_particles_to_keep:]]

		yaw = np.random.normal(0, np.pi/40, self.n_particles - self.n_particles_to_keep)
		pitch = np.random.normal(0, np.pi/60, self.n_particles - self.n_particles_to_keep)
		roll = np.random.normal(0, np.pi/60, self.n_particles - self.n_particles_to_keep)

		xyz = np.random.normal(0, .2, (self.n_particles - self.n_particles_to_keep, 3))
		# Convert the particles to quaternions
		perturbance = R.from_euler("xyz", np.vstack((roll, pitch, yaw)).T)

		# Create a random index array
		try:
			indices = np.random.choice(self.particles.shape[0], self.particles.shape[0] - self.n_particles_to_keep, p=self.probabilities)
		except:
			print(self.probabilities)
		# Create a new particle array
		new_particles[self.n_particles_to_keep:] = self.particles[indices]
		# Perturb the particles
		new_particles[self.n_particles_to_keep:, :3] = new_particles[self.n_particles_to_keep:, :3] + xyz
		# Create a new probability array
		new_probabilities[self.n_particles_to_keep:] = self.probabilities[indices] * 0.9 # Discount the probability
		new_probabilities = new_probabilities / np.sum(new_probabilities)

		# New rotations
		new_rotations[self.n_particles_to_keep:] = self.rotations[indices]
		
		# Perturb the rotations
		#new_rotations = new_rotations * perturbance
		new_rotations[self.n_particles_to_keep:] = new_rotations[self.n_particles_to_keep:] * perturbance
		# Set the new particles, probabilities and rotations
		self.particles = new_particles
		self.probabilities = new_probabilities
		self.rotations = new_rotations

		if self.particle_full_resample_rate > 0:
			# Replace self.particle_full_resample_rate % of the particles with new ones
			resample_n = int(self.particle_full_resample_rate * self.particles.shape[0])
			# Replace the lowest resample_n probable particles with new ones
			sorted_index = np.argsort(self.probabilities)
			# print("Particle probabilities: {}".format(self.probabilities))
			# print("Particles being replaced: {}".format(sorted_index[:resample_n]))
			self.particles[sorted_index[:resample_n], :3] = self.world.get_probable_random_points(resample_n, poisson=True)
			# Set the probabilities of the new particles to 1/n
			self.probabilities[sorted_index[:resample_n]] = 1.0 / self.particles.shape[0]

			# Create random rotations for the new particles
			yaw = np.random.uniform(0, 2*np.pi, resample_n)
			pitch = np.random.normal(0, np.pi/20, resample_n)
			roll = np.random.normal(0, np.pi/20, resample_n)
			# Convert the particles to quaternions
			self.rotations[sorted_index[:resample_n]] = R.from_euler("xyz", np.vstack((roll, pitch, yaw)).T)

		# Normalize probabilities
		self.probabilities = self.probabilities / np.sum(self.probabilities)


	def get_particle_lidar_rays(self, rays):
		assert rays.shape[1] == 6, "Rays must be shape [n, 6]"

		new_rays = np.zeros((self.particles.shape[0]*rays.shape[0], 6))
		#new_directions = self.rotations.apply(rays)
		# Rotate the ray directions to the particles
		for i, r in enumerate(self.rotations):
			new_rays[i*rays.shape[0]:(i+1)*rays.shape[0], 3:] = r.apply(rays[:, 3:])
		#new_rays[:, :3] = self.rotations.apply(rays)

		# Create a new origo for each ray, using the particles as origo
		new_origo = np.repeat(self.particles[:, :3], rays.shape[0], axis=0)
		# Add the new origo to the new directions
		new_rays[:, :3] = new_origo
		#new_rays[:, :3] += new_origo
		# print("New rays shape: {}".format(new_rays.shape))
		return new_rays.astype(np.float32)
	

	def raycast_particles(self, target_rays): # Target rays are the rays in the lidar frame. Should be shape [n, 6]
		assert target_rays.shape[1] == 6, "Target rays must be shape [n, 6]"
		# Transform the rays for each particle
		rays = self.get_particle_lidar_rays(target_rays)

		# print("Casting {} rays".format(rays.shape[0]))
		t0 = time.time()
		raycast = self.world.scene.cast_rays(rays)
		raycast_depth =raycast["t_hit"].numpy()
		raycast_normals = raycast['primitive_normals'].numpy()
		# print("Time to cast rays: {}".format(time.time()-t0))

		# Reshape the raycast
		raycast_normals = raycast_normals.reshape(self.n_particles, -1, 3) # Was (n_particles, 128, 1024, 3)
		raycast_depth = raycast_depth.reshape(self.n_particles, -1, 1) # Was (n_particles, 128, 1024, 1)
		return raycast_depth*1000, raycast_normals


	def get_centroid(self):
		# Get the centroid of the particles
		pos_avg = np.average(self.particles[:,:3], axis=0, weights=self.probabilities)
		# Get the average rotation
		rot_avg = self.rotations.mean(weights=self.probabilities)

		return pos_avg, rot_avg


	def init_particles(self, n, visualize=False):
		# Create particles
		self.n_particles = n
		self.particles = self.world.get_probable_random_points(self.n_particles)
		# Create random orientations for the particles with a uniform distribution for the yaw, but normal distribution for the pitch and roll
		yaw = np.random.uniform(0, 2*np.pi, self.n_particles)
		pitch = np.random.normal(0, np.pi/20, self.n_particles)
		roll = np.random.normal(0, np.pi/20, self.n_particles)

		# Convert the particles to quaternions
		quats = R.from_euler("xyz", np.vstack((roll, pitch, yaw)).T).as_quat()

		# Inject actual robot pose for testing
		quats[0] = np.array([0.0010384084288430288, 0.00030120795595691265, -0.00011077939289218057, 0.9999994093546397])
		self.particles[0] = np.array([0.00046312785132803284, -0.002913896481242659, 1.2932854060325036])

		self.rotations = R.from_quat(quats)

		self.probabilities = np.ones(self.n_particles)/self.n_particles

		
		if visualize:
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

		# print("Particles shape: {}".format(self.particles.shape))


	def lidar_callback(self, msg):
		self.clock = msg.header.stamp
		data = pclmao.extract_PointCloud2_data(msg)

		# x = data["x"], y = data["y"], z = data["z"]
		# Shape of x: (1024, 128, 1)
		xyz = np.dstack((data["x"], data["y"], data["z"]))
		# Shape of xyz: (1024, 128, 3)
		depth = data["range"]

		#Shape of xyz: (1024, 128, 3)
		xyz = np.rot90(xyz, 1, (1, 0))
		depth = np.rot90(depth, 1, (1, 0))

		# Get the normals
		normals = get_normals(xyz)


		if VIRTUAL_PLAYBACK == True:
			# Attempt to get transform at time of message, otherwise get most recent
			try:
				# Get the true transform at the time of the message
				t = self.tf_buffer.lookup_transform(self.world_frame, msg.header.frame_id, msg.header.stamp)
			except TransformException as e:
				# self.get_logger().warn("Transform error: {}, when transforming from {} to {}\n Trying most recent".format(e, msg.header.frame_id, self.world_frame))
				t = None
				
			if t is None:
				try:
					# Get the most recent transform
					t = self.tf_buffer.lookup_transform(self.world_frame, msg.header.frame_id, rclpy.time.Time())
				except TransformException as e:
					self.get_logger().error("Transform error: {}, when transforming from {} to {}".format(e, msg.header.frame_id, self.world_frame))
					return
				
			# # extract the rotation and translation components of the transform
			rotation = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
			translation = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
			
			# print("Actual translation: {}".format(translation))
			# print("Actual rotation: {}".format(rotation))

			# Create similar image from virtual world and lidar
			rays = self.lidar.rotate_rays(rotation)
			rays += [translation[0], translation[1], translation[2], 0, 0, 0]

			raycast = self.world.cast_rays(rays)

			vdepth = raycast['t_hit'].numpy()
			vnormals = raycast['primitive_normals'].numpy()

			# Show normals as 2d image in opencv that updates as new data comes in
			vnormals[depth_mask[:, :, 0]] = 0
			cv2.imshow("Virtual normals", abs(vnormals))
			cv2.waitKey(1)

		# # Construct the transformation matrix
		# r = R.from_quat(rotation)
		# T = np.eye(4)
		# T[:3, :3] = r.as_matrix()
		# T[:3, 3] = translation

		# #Shape of xyz: (1024, 128, 3)
		# # Transform the data
		# xyz = np.matmul(xyz, T[:3, :3].T) + T[:3, 3]

		# Show normals as 2d image in opencv that updates as new data comes in
		# Shape of normals: (1024, 128, 3)
		# Convert to 2d image
		cv2.imshow("Normals", abs(normals))
		cv2.waitKey(1)
		# Get indices of depth < self.max_range
		depth_mask = depth > self.max_range
		normals[depth_mask[:, :, 0]] = 0
		# print("Depth mash shape: {}".format(depth_mask.shape))
		# print("Normals shape: {}".format(normals.shape))



		#plt.imshow(normals/2 + 0.5)
		#plt.show()
		#plt.imsave("normals.png", (normals/2 + 0.5)[:, :511, :])

		# Set self.lidar.rays as rays from the lidar
		# Assert that all rays are not nan or inf
		assert np.all(np.isfinite(xyz)), "xyz contains non-finite values"

		self.lidar.set_rays(np.divide(xyz, np.linalg.norm(xyz, axis=2, keepdims=True)))

		#print("First lidar ray: {}".format(self.lidar.rays[0, 0, :]))
		#print("First dummy lidar ray: {}".format(self.dummy_lidar.rays[0, 0, :]))

		# Use matplotlib to show normals
		#plt.imshow(vnormals/2 + 0.5)
		#plt.show()
		#plt.imsave("virtual_normals.png", (vnormals/2 + 0.5)[:, :511, :])

		# Select rays for localization
		ray_indices = self.select_rays(depth, xyz, 100, visualize=True)
		
		# 
		rays = self.lidar.rays[ray_indices] # Assumes same lidar model. More correct solution is to take xyz from the incoming image and normalize it
		#rays = xyz[ray_indices]
		#rays = rays.reshape(-1, 3)
		# Normalize for length 1
		#rays = rays / np.linalg.norm(rays, axis=1)[:, None]

		# Needs origo = [0,0,0] prepended to each ray
		#rays = np.hstack((np.zeros((rays.shape[0], 3)), rays))


		# FOR TESTING::::: ------------------------------------------------------
		# rays = self.lidar.rays
		# rays = rays.reshape(-1, 6)

		# print("Shape of rays: {}".format(rays.shape))

		# Inject actual robot position into particle [0]
		#self.particles[0] = np.array([translation[0], translation[1], translation[2], rotation[0], rotation[1], rotation[2], rotation[3]])
		#self.rotations[0] = R.from_quat(rotation)

		# Get the virtual rays
		actual_depth = depth[ray_indices]
		actual_normals = normals[ray_indices]
		raycast_depth, raycast_normals = self.raycast_particles(rays)

		# # FOR TESTING::::: ------------------------------------------------------
		# actual_depth = depth.reshape(-1)
		# actual_normals = normals.reshape(-1, 3)


		# Rotate normals, such that they are in the reference frame of the robot
		# Shape of raycast_normals: (n, rays, 3)
		for i, r in enumerate(self.rotations):
			raycast_normals[i] = r.apply(raycast_normals[i])


		# hard_depth = np.linalg.norm(raycast_depth[0] - actual_depth)
		# print("Hard depth: {}".format(hard_depth))

		# Should be shape n
		error_depth = np.linalg.norm(raycast_depth - actual_depth, axis=1).ravel()

		# print("Shape of error_depth: {}".format(error_depth.shape))

		# Calculate mean and variance of error
		# mean_depth = np.mean(error_depth[1])
		# var_depth = np.std(error_depth[1])

		# # Calculate the error for all. raycast is [n, rays, 3], actual is [rays, 3]
		# error_normal = np.linalg.norm(raycast_normals[:, :, 2][:, np.newaxis, :] - actual_normals[np.newaxis, :, 2], axis=2).ravel()

		# Calculate MSE for depth
		# mse = np.zeros(raycast_depth.shape[0])
		# for i in range(raycast_depth.shape[0]):
		# 	mse[i] = np.mean(np.square(raycast_depth[i] - actual_depth))

		# cosine_dist = np.zeros(self.particles.shape[0])
		# for i in range(self.particles.shape[0]):
		# 	#cosine_dist[i] = np.mean(paired_cosine_distances(actual_normals, raycast_normals[i]))
		# 	cosine_dist[i] = np.mean( np.square(paired_cosine_distances(actual_normals, raycast_normals[i])) )
		# Or perhaps the 35 times faster version i just cooked up:
		cosine_dist = np.mean(
			np.square(
				0.5 * np.square(
					np.linalg.norm(
						raycast_normals - actual_normals.reshape(1, actual_normals.shape[0], actual_normals.shape[1]), axis=2
						)
					)
				), axis=1
			)
		# Print what order of indexes with the lowest error, ie [100, 69, 52] would mean that the 100th particle has the lowest error, 69th the second lowest and so on
		# best_indicies_depth = np.argsort(error_depth)
		# best_indicies_normal = np.argsort(error_normal)
		# best_indicies_mse = np.argsort(mse)
		# best_indicies_cosine = np.argsort(cosine_dist)

		# print("Best indicies d: {}".format(best_indicies_depth))
		# print("Values d: {}".format(error_depth[best_indicies_depth]))
		# print("Best indicies n: {}".format(best_indicies_normal))
		# print("Values n: {}".format(error_normal[best_indicies_normal]))
		# print("Best indicies mse: {}".format(best_indicies_mse))
		# print("Values mse: {}".format(mse[best_indicies_mse]))
		# print("Best indicies cosine: {}".format(best_indicies_cosine))
		# print("Values cosine: {}".format(cosine_dist[best_indicies_cosine]))


		d_processed = np.power(error_depth, -1)
		d_processed = d_processed / np.sum(d_processed)

		# mse_processed = np.power(mse, -1)
		# mse_processed = mse_processed / np.sum(mse_processed)

		#n_processed = np.power(error_normal, 1)
		# n_processed = np.power(error_normal, -1)
		# n_processed = n_processed / np.sum(n_processed)

		cos_processed = np.power(cosine_dist, -1)
		cos_processed = cos_processed / np.sum(cos_processed)


		# print("d_processed: {}".format(d_processed[best_indicies_depth]))
		# print("mse_processed: {}".format(mse_processed[best_indicies_mse]))
		# print("n_processed: {}".format(n_processed[best_indicies_normal]))
		# print("cos_processed: {}".format(cos_processed[best_indicies_cosine]))

		#self.probabilities = self.probabilities + ((d_processed + cos_processed) / 2 )
		# If probability is nan dont
		if not(np.isnan(d_processed).any() or np.isnan(cos_processed).any()):
			self.probabilities = self.probabilities * (1-self.particle_learning_rate) + ((d_processed*3 + cos_processed)/4) * self.particle_learning_rate

		self.best_index = np.argmax(self.probabilities)

		self.best_indices = np.argsort(self.probabilities)

		# print("Highest 5 probabilities: {}".format(np.flip( self.probabilities[self.best_indices[-5:]])))
		# print("Indices of highest 5 probabilities: {}".format(np.flip(self.best_indices[-5:])))
		# print("Best index: {}".format(self.best_index))

		self.counter += 1
		if self.counter % 5 == 0:
			self.resample_particles()


	def plot_prob(self, p, name, n_selections = None, rows = None, cols = None, viz = False):
		if SAVEFIGS == False:
			return
		# Get number of dimensions of numpy array p
		dim = len(p.shape)
		if dim == 3:
			p_local = p[:, :, 0]
		else:
			p_local = p


		# Create image of probabilities
		#prob_image = np.zeros((p_local.shape[0], p_local.shape[1], 3))
		# Set the probabilities as the all (so it goes from black to white)

		cmap = plt.get_cmap('viridis')
		# Normalize the probabilities to 0-1
		prob_array = (p_local-np.min(p_local)) / (np.max(p_local)-np.min(p_local))
		prob_image = cmap(prob_array)
		# Remove alpha channel
		prob_image = prob_image[:, :, :3]

		#prob_image[:, :, 0] = (p_local-np.min(p_local)) / (np.max(p_local)-np.min(p_local))
		#prob_image[:, :, 1] = (p_local-np.min(p_local)) / (np.max(p_local)-np.min(p_local))
		#prob_image[:, :, 2] = (p_local-np.min(p_local)) / (np.max(p_local)-np.min(p_local))

		if rows is not None and cols is not None:
			prob_image[rows, cols, :] = [0.5, 0.5, 0.1]

		if n_selections is not None:
			random_indices = np.random.choice(p_local.shape[0] * p_local.shape[1], size=n_selections, replace=False, p=(p_local / np.sum(p_local)).ravel())
			selected_indices = np.unravel_index(random_indices, (128, 1024))
			prob_image[selected_indices[0], selected_indices[1], :] = [1, 0, 0]

		# Show the image
		if viz:
			plt.imshow(prob_image)
			plt.title(name)
			plt.show()
		plt.imsave(__file__[:-3] + "_" + name + ".png", prob_image)

			


	def select_rays(self, depth, xyz, amount, visualize=False):
		# Find all places where depth outside min and max range
		rows, cols, _ = np.where((depth < self.min_range) | (depth > self.max_range))

		# Get the normals
		normals = get_normals(xyz)

		# Calculate gradient of depth
		depth_gradient = np.gradient(depth, axis=(0, 1))
		depth_gradient = np.abs(depth_gradient[0]) + np.abs(depth_gradient[1])
		max_gradient = np.max(depth_gradient)
		depth_gradient[np.isinf(depth_gradient)] = max_gradient
		depth_gradient[np.isnan(depth_gradient)] = max_gradient

		depth_gradient = 1/(10+depth_gradient)
		
		depth_gradient[rows, cols] = 0

		depth_gradient = depth_gradient / np.sum(depth_gradient)


		

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
		
		#dist_probabilities = (probabilities2 / np.max(probabilities2))
		depth[np.isinf(depth)] = self.max_range
		depth[np.isnan(depth)] = self.max_range
		dist_probabilities = 1 / (1 + depth)
		dist_probabilities[rows, cols] = 0
		dist_probabilities = dist_probabilities / np.sum(dist_probabilities)

		# print("Max dept_gradient: {}".format(np.max(depth_gradient)))
		# print("Min dept_gradient: {}".format(np.min(depth_gradient)))
		# print("Size of depth_gradient: {}".format(depth_gradient.shape))
		# print("Max dist_probabilities: {}".format(np.max(dist_probabilities)))
		# print("Min dist_probabilities: {}".format(np.min(dist_probabilities)))
		# print("Size of dist_probabilities: {}".format(dist_probabilities.shape))
		self.plot_prob(depth_gradient, "depth_gradient", n_selections=1000, rows=rows, cols=cols)
		self.plot_prob(dist_probabilities, "dist_probabilities", n_selections=1000, rows=rows, cols=cols)

		# # Invert depth gradient, such that lower values are more likely to be selected
		# depth_gradient = 1.0 / depth_gradient

		# # Set nan to 0
		# depth_gradient[np.isinf(depth_gradient)] = 0

		# depth_gradient = np.clip(depth_gradient, 0.0, 1)

		# # Apply gaussian filter to depth gradient
		# depth_gradient = ndimage.gaussian_filter(depth_gradient, sigma=1.0)

		# #Scale depth gradient to 0-1
		# depth_gradient = (depth_gradient / np.max(depth_gradient))[:,:,0] # Otherwise is [m,n,1]

		# Set nan to max gradient



		# print("Depth gradient shape: {}".format(depth_gradient.shape))
		# print("Dist probabilities shape: {}".format(dist_probabilities.shape))

		# Average the two probabilities
		probabilities = depth_gradient*0.3 + dist_probabilities*0.7
		probabilities_multiply = depth_gradient * dist_probabilities
		probabilities_multiply = probabilities_multiply / np.sum(probabilities_multiply)

		self.plot_prob(probabilities, "probabilities", n_selections=1000, rows=rows, cols=cols)
		self.plot_prob(probabilities_multiply, "probabilities_multiply", n_selections=1000, rows=rows, cols=cols)
		probabilities[rows, cols] = 0
		probabilities = probabilities[:,:,0]

		random_indices = np.random.choice(probabilities.shape[0] * probabilities.shape[1], size=amount, replace=False, p=(probabilities / np.sum(probabilities)).ravel())
		
		selected_indices = np.unravel_index(random_indices, (128, 1024))

		

		if visualize:
			probabilities_image = np.zeros((128, 1024, 3))
			probabilities_image[:,:,0] = probabilities
			probabilities_image[selected_indices[0], selected_indices[1]] = [0, 1, 0]
			cv2.imshow("Probabilities", probabilities_image)
		return selected_indices


def main(args=None):
	rclpy.init(args=args)

	localizer = Localizer()

	rclpy.spin(localizer)

	localizer.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()