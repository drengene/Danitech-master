import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import lmao_lib.lidar as Lidar
import lmao_lib.world as World
import lmao_lib.util.pclmao as pclmao
from lmao_lib.mapping import get_normals

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
		self.declare_parameter('map_path', '/home/junge/Documents/mesh_map/map.ply', ParameterDescriptor(description="Path to the map file"))
		#self.declare_parameter('map_path', '/home/danitech/master_ws/src/Danitech-master/wagon_navigation/wagon_navigation/pose_data/isaac_map.ply', ParameterDescriptor(description="Path to the map file"))
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
		self.particle_pcd.points = o3d.utility.Vector3dVector(self.particles[:, :3])
		dir_vecs = self.rotations.apply(np.array([[1, 0, 0]]))
		self.particle_pcd.normals = o3d.utility.Vector3dVector(dir_vecs)
		self.viz.add_geometry(self.particle_pcd)
		self.render_option = self.viz.get_render_option()
		self.render_option.point_show_normal = True

		# Create timer to update the visualization every 0.1 seconds
		self.viz_timer = self.create_timer(0.1, self.viz_loop)

		# Subscribe to the odometry topic
		self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
		self.get_logger().info("Subscribed to topic: {}".format(self.odom_topic))
		self.stamp = None

		self.particle_learning_rate = 0.1
		self.particle_full_resample_rate = 0.3

		self.counter = 0

	def viz_loop(self):
		self.particle_pcd.points = o3d.utility.Vector3dVector(self.particles[:, :3])
		dir_vecs = self.rotations.apply(np.array([[1, 0, 0]]))
		self.particle_pcd.normals = o3d.utility.Vector3dVector(dir_vecs)
		colors = np.zeros((self.particles.shape[0], 3))
		colors[:, 1] = self.probabilities / np.max(self.probabilities)
		colors[:, 0] = 1 - (self.probabilities / np.max(self.probabilities))
		self.particle_pcd.colors = o3d.utility.Vector3dVector(colors)

		self.viz.update_geometry(self.particle_pcd)
		self.viz.poll_events()
		self.viz.update_renderer()


	def odom_callback(self, msg):
		if self.stamp is None:
			self.stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
			return
		new_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
		dt = new_stamp - self.stamp
		self.stamp = new_stamp
		print("dt: {}".format(dt))
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
		print("dt: {}".format(dt))
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
		# Create new particles
		new_particles = np.zeros_like(self.particles)
		# Create new probabilities
		new_probabilities = np.zeros_like(self.probabilities)

		yaw = np.random.normal(0, np.pi/30, self.n_particles)
		pitch = np.random.normal(0, np.pi/60, self.n_particles)
		roll = np.random.normal(0, np.pi/60, self.n_particles)

		xyz = np.random.normal(0, 0.5, (self.n_particles, 3))
		# Convert the particles to quaternions
		perturbance = R.from_euler("xyz", np.vstack((roll, pitch, yaw)).T)

		# Create a random index array
		indices = np.random.choice(self.particles.shape[0], self.particles.shape[0], p=self.probabilities)
		# Create a new particle array
		new_particles = self.particles[indices]
		# Perturb the particles
		new_particles[:, :3] = new_particles[:, :3] + xyz
		# Create a new probability array
		new_probabilities = self.probabilities[indices]
		new_probabilities = new_probabilities / np.sum(new_probabilities)
		# New rotations
		new_rotations = self.rotations[indices]
		# Perturb the rotations
		new_rotations = new_rotations * perturbance
		# Set the new particles, probabilities and rotations
		self.particles = new_particles
		self.probabilities = new_probabilities
		self.rotations = new_rotations


		# Replace self.particle_full_resample_rate % of the particles with new ones
		resample_n = int(self.particle_full_resample_rate * self.particles.shape[0])
		self.world.get_probable_random_points(resample_n, poisson=False)


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
		print("New rays shape: {}".format(new_rays.shape))
		return new_rays.astype(np.float32)
	

	def raycast_particles(self, target_rays): # Target rays are the rays in the lidar frame. Should be shape [n, 6]
		assert target_rays.shape[1] == 6, "Target rays must be shape [n, 6]"
		# Transform the rays for each particle
		rays = self.get_particle_lidar_rays(target_rays)

		print("Casting {} rays".format(rays.shape[0]))
		t0 = time.time()
		raycast = self.world.scene.cast_rays(rays)
		raycast_depth =raycast["t_hit"].numpy()
		raycast_normals = raycast['primitive_normals'].numpy()
		print("Time to cast rays: {}".format(time.time()-t0))

		# Reshape the raycast
		raycast_normals = raycast_normals.reshape(self.n_particles, -1, 3) # Was (n_particles, 128, 1024, 3)
		raycast_depth = raycast_depth.reshape(self.n_particles, -1, 1) # Was (n_particles, 128, 1024, 1)
		return raycast_depth*1000, raycast_normals


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
		quats[0] = np.array([-9.001217674607042e-05, 9.343880844289536e-05, 0.10426205165325972, 0.9945498518184245])
		self.particles[0] = np.array([7.739036989559301, 1.8213204770359357, 0.9957110470344852])

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

		print("Particles shape: {}".format(self.particles.shape))



	def lidar_callback(self, msg):
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

		# Show normals as 2d image in opencv that updates as new data comes in
		# Shape of normals: (1024, 128, 3)
		# Convert to 2d image
		cv2.imshow("Normals", abs(normals))
		cv2.waitKey(1)

		# Set self.lidar.rays as rays from the lidar
		# Assert that all rays are not nan or inf
		assert np.all(np.isfinite(xyz)), "xyz contains non-finite values"

		self.lidar.set_rays(np.divide(xyz, np.linalg.norm(xyz, axis=2, keepdims=True)))

		#print("First lidar ray: {}".format(self.lidar.rays[0, 0, :]))
		#print("First dummy lidar ray: {}".format(self.dummy_lidar.rays[0, 0, :]))
		


		# Create similar image from virtual world and lidar
		rays = self.lidar.rotate_rays(rotation)
		rays += [translation[0], translation[1], translation[2], 0, 0, 0]

		raycast = self.world.cast_rays(rays)

		vdepth = raycast['t_hit'].numpy()
		vnormals = raycast['primitive_normals'].numpy()

		# Show normals as 2d image in opencv that updates as new data comes in
		cv2.imshow("Virtual normals", abs(vnormals))
		cv2.waitKey(1)

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

		print("Shape of rays: {}".format(rays.shape))

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

		# Show normals as 2d image in opencv that updates as new data comes in
		#raycast_image = np.zeros((self.particles.shape[0], 128, 1024, 3))
		#for i in range(self.particles.shape[0]):
		#	raycast_image[i] = abs(raycast_normals[i].reshape(128, 1024, 3))

		#cv2.imshow("Virtual normals", abs(raycast_image[0]))
		#cv2.waitKey(1)


		# print("Shape of raycast_normals: {}".format(raycast_normals.shape))
		# print("Shape of actual_normals: {}".format(actual_normals.shape))

		# print("Mean of raycast_depth: {}".format(np.mean(raycast_depth[0])))
		# print("Mean of actual_depth: {}".format(np.mean(actual_depth)))

		# print("Shape of raycast_depth: {}".format(raycast_depth.shape))
		# print("Shape of actual_depth: {}".format(actual_depth.shape))

		hard_depth = np.linalg.norm(raycast_depth[0] - actual_depth)
		# print("Hard depth: {}".format(hard_depth))


		# Calculate the error for all. raycast is [n, rays], actual is [rays]
		#error_depth = raycast_depth - actual_depth
		# Should be shape n
		error_depth = np.linalg.norm(raycast_depth - actual_depth, axis=1).ravel()

		# print("Shape of error_depth: {}".format(error_depth.shape))

		# Calculate mean and variance of error
		mean_depth = np.mean(error_depth[1])
		var_depth = np.std(error_depth[1])
		# print("Mean depth error: {}".format(mean_depth))
		# print("StdDev depth error: {}".format(var_depth))
		# print("Min depth error_ {}, max depth error: {}".format(np.min(error_depth[1]), np.max(error_depth[1])))

		# Calculate the error for all. raycast is [n, rays, 3], actual is [rays, 3]
		error_normal = np.linalg.norm(raycast_normals[:, :, 2][:, np.newaxis, :] - actual_normals[np.newaxis, :, 2], axis=2).ravel()

		# Calculate MSE for depth
		mse = np.zeros(raycast_depth.shape[0])
		for i in range(raycast_depth.shape[0]):
			mse[i] = np.mean(np.square(raycast_depth[i] - actual_depth))

		cosine_dist = np.zeros(self.particles.shape[0])
		for i in range(self.particles.shape[0]):
			#cosine_dist[i] = np.mean(paired_cosine_distances(actual_normals, raycast_normals[i]))
			cosine_dist[i] = np.mean( np.square(paired_cosine_distances(actual_normals, raycast_normals[i])) )


		# print("Shape of normal error: {}".format(error_normal.shape))
		# print("Shape of depth error: {}".format(error_depth.shape))
		# print("Shape of cosine_dist_normal: {}".format(cosine_dist_normal.shape))


		# Print what order of indexes with the lowest error, ie [100, 69, 52] would mean that the 100th particle has the lowest error, 69th the second lowest and so on
		best_indicies_depth = np.argsort(error_depth)
		best_indicies_normal = np.argsort(error_normal)
		best_indicies_mse = np.argsort(mse)
		best_indicies_cosine = np.argsort(cosine_dist)

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

		mse_processed = np.power(mse, -1)
		mse_processed = mse_processed / np.sum(mse_processed)

		#n_processed = np.power(error_normal, 1)
		n_processed = np.power(error_normal, -1)
		n_processed = n_processed / np.sum(n_processed)

		cos_processed = np.power(cosine_dist, -1)
		cos_processed = cos_processed / np.sum(cos_processed)


		# print("d_processed: {}".format(d_processed[best_indicies_depth]))
		# print("mse_processed: {}".format(mse_processed[best_indicies_mse]))
		# print("n_processed: {}".format(n_processed[best_indicies_normal]))
		# print("cos_processed: {}".format(cos_processed[best_indicies_cosine]))

		#self.probabilities = self.probabilities + ((d_processed + cos_processed) / 2 )
		self.probabilities = self.probabilities * (1-self.particle_learning_rate) + ((d_processed + cos_processed) / 2 ) * self.particle_learning_rate

		self.best_index = np.argmax(self.probabilities)

		self.best_indices = np.argsort(self.probabilities)

		print("Highest 5 probabilities: {}".format(np.flip( self.probabilities[self.best_indices[-5:]])))
		print("Indices of highest 5 probabilities: {}".format(np.flip(self.best_indices[-5:])))
		print("Best index: {}".format(self.best_index))

		self.counter += 1
		if self.counter % 5 == 0:
			self.resample_particles()
			


	def select_rays(self, depth, xyz, amount, visualize=False):
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
		depth_gradient = (depth_gradient / np.max(depth_gradient))[:,:,0] # Otherwise is [m,n,1]

		# print("Depth gradient shape: {}".format(depth_gradient.shape))
		# print("Dist probabilities shape: {}".format(dist_probabilities.shape))

		# Average the two probabilities
		probabilities = depth_gradient*0.8 + dist_probabilities*0.2
		#probabilities = depth_gradient * dist_probabilities

		probabilities[rows, cols] = 0

		probabilities_image = np.zeros((128, 1024, 3))
		probabilities_image[:,:,0] = probabilities

		random_indices = np.random.choice(probabilities.shape[0] * probabilities.shape[1], size=amount, replace=False, p=(probabilities / np.sum(probabilities)).ravel())
		
		selected_indices = np.unravel_index(random_indices, (128, 1024))

		probabilities_image[selected_indices[0], selected_indices[1]] = [0, 1, 0]

		if visualize:
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