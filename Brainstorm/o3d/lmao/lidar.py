# -------------------
#
# Welcome to LMAO - Localization Mapping and Optimization (or Odometry)
# This is a library that will include the classes and functions needed to perform localization, mapping and optimization in a 3D environment.
# The library is built on top of Open3D, and uses raycasting in a triangle mesh.
# It is made by Rasmus Peter Junge, and Daniel Gahner Holm for our Master's Thesis at SDU.
#
# -------------------

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import quaternion
import cv2
from scipy.spatial.transform import Rotation as R

import json


class Virtual_Lidar:
	def __init__(self, horizontal_lines=64, rays_per_line=1024, vertical_fov = np.pi/2):
		self.horizontal_lines = horizontal_lines
		self.rays_per_line = rays_per_line
		self.vertical_fov = vertical_fov
		self.rays = self.generate_rays(offset=0)
		self.current_rotation = [0, 0, 0, 1]
		self.current_position = np.array([0, 0, 0])


	def generate_rays(self, offset = 0):
		rays = np.zeros((self.horizontal_lines,self.rays_per_line, 6), dtype=np.float32)
		rays_optimized = np.zeros((self.horizontal_lines,self.rays_per_line, 6), dtype=np.float32)

		rot_x = np.sin(np.linspace(0 + offset, 2*np.pi, self.rays_per_line + offset, endpoint=False))
		rot_y = np.cos(np.linspace(0 + offset, 2*np.pi, self.rays_per_line + offset, endpoint=False))

		# dz = np.cos(np.linspace((np.pi)/4, 3*np.pi/4, horizontal_lines))
		dz = np.cos(np.linspace((np.pi)/2 - self.vertical_fov/2, (np.pi)/2 + self.vertical_fov/2, self.horizontal_lines, endpoint=True))
		dz_scale = np.sin(np.linspace((np.pi)/2 - self.vertical_fov/2, (np.pi)/2 + self.vertical_fov/2, self.horizontal_lines, endpoint=True))
		rays[:, :, 3] = rot_x * dz_scale[:, None]
		rays[:, :, 4] = rot_y * dz_scale[:, None]
		rays[:, :, 5] = dz[:, None]

		return rays

	def rotate_rays(self, xyzw, change_original=False):
		# Uses Quaternion library
		# Library uses (w, x, y, z), but the function takes (x, y, z, w)
		# 
		# This one is twice as fast on large arrays compared to the scipy variant
		# At least on my machine
		#
		#
		# Rotate the lidar rays into the global coordinate frame
		q = quaternion.as_quat_array([xyzw[3], xyzw[0], xyzw[1], xyzw[2]]) # Change the order of the quaternion
		rays_local_flat = self.rays[:,:,3:].reshape(-1, 3) # Flatten the final 3 values of the array
		rays_global_flat = quaternion.rotate_vectors(q, rays_local_flat).astype(np.float32) # Change the data type to float32
		# Reshape and prepend the original position to the array
		rays_global = np.concatenate((self.rays[:, :, :3], rays_global_flat.reshape(self.rays.shape[:2] + (3,))), axis=2)
		if change_original:
			self.rays = rays_global
			self.current_rotation = q * self.current_rotation
		return rays_global

	
	def rotate_rays_scipy(self, q, change_original=False):
		# With scipy, the quaternion is in the form (x, y, z, w)
		rays_local_flat = self.rays[:,:,3:].reshape(-1, 3) # Flatten the final 3 values of the array
		r = R.from_quat(q)
		rays_global_flat = r.apply(rays_local_flat)
		# Reshape and prepend the original position to the array
		rays_global = np.concatenate((self.rays[:, :, :3], rays_global_flat.reshape(self.rays.shape[:2] + (3,))), axis=2)
		if change_original:
			self.rays = rays_global
			self.current_rotation = q * self.current_rotation
		return rays_global


	def translate_rays(self, pos, change_original=False):
		# Translate the lidar rays into the global coordinate frame
		if change_original:
			self.rays[:, :, :3] += pos
			self.current_position += pos
			return self.rays
		else:
			return self.rays + [pos[0], pos[1], pos[2], 0, 0, 0]

	def check_unity(self, rays = None):
		if rays is None:
			rays = self.rays
		directions = rays[:, :, 3:]
		return np.allclose(np.linalg.norm(directions, axis=2), 1)

	def plot_rays(self, rays=None,fig=None, ax=None, visualize=True, plot_unit_sphere=True):
		if fig is None:
			fig = plt.figure() 
		if ax is None:
			ax = fig.add_subplot(111, projection='3d')
		if rays is None:
			rays = self.rays
		# Extract x, y, and z coordinates from the data
		x = rays[:, :, 3].flatten()
		y = rays[:, :, 4].flatten()
		z = rays[:, :, 5].flatten()

		x0 = rays[:, :, 0].flatten()
		y0 = rays[:, :, 1].flatten()
		z0 = rays[:, :, 2].flatten()

		# Create a 3D plot. Colors should change gradually as index increases. The very first point should be red
		ax.scatter(x[1:]+x0[1:], y[1:]+y0[1:], z[1:]+z0[1:], c=np.arange(len(x[1:])), cmap='viridis', alpha=0.5)
		ax.scatter(x[0]+x0[0], y[0]+y0[0], z[0]+z0[0], c='r', alpha=1)
		if plot_unit_sphere:
			# Plot a sphere for the origin
			u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
			x = np.cos(u)*np.sin(v)
			y = np.sin(u)*np.sin(v)
			z = np.cos(v)
			ax.plot_surface(x, y, z, color="r", alpha=0.5)


		# Add labels and title
		ax.set_xlabel('X')
		ax.set_ylabel('Y')
		ax.set_zlabel('Z')
		plt.title('3D Scatter Plot')

		ax.set_box_aspect(aspect=(1, 1, 1)) 

		# Show the plot
		if visualize:
			plt.show()


	def noise(self, noise_std=0.01, change_original=False):
		# Add noise to the lidar rays
		if change_original:
			self.rays[:, :, 3:] += np.random.normal(0, noise_std, self.rays[:, :, 3:].shape)
			return self.rays
		else:
			return self.rays + np.random.normal(0, noise_std, self.rays.shape)
		

class Lidar:
	def __init__(self, config_file):
		# Load config json file from path 
		with open(config_file) as f:
			self.config = json.load(f)


	def unstagger(self, data):
		# Check if data is a dictionary
		if isinstance(data, dict):
			for key, value in data.items():
				data[key] = self.unstagger(value)
			return data
		# Check if data is a list
		elif isinstance(data, list):
			for i, value in enumerate(data):
				data[i] = self.unstagger(value)
			return data
		# Check if data is a numpy array
		elif isinstance(data, np.ndarray):
			rolled = np.empty_like(data)
			# Assert that data.shape[0] is similar to config "pixel_shift_by_row"
			assert data.shape[0] == len(self.config["data_format"]["pixel_shift_by_row"])
			for i in range(data.shape[0]):
				rolled[i] = np.roll(data[i], self.config["data_format"]["pixel_shift_by_row"][i])
			return rolled

	
