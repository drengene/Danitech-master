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


class Lidar:
	def __init__(self, horizontal_lines=64, rays_per_line=1024, vertical_fov = np.pi/2):
		self.horizontal_lines = horizontal_lines
		self.rays_per_line = rays_per_line
		self.vertical_fov = vertical_fov
		self.rays = self.generate_rays(self, offset = 0)


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

	def rotated_rays(self, rays, q):
		# Rotate the lidar rays into the global coordinate frame
		rays_local_flat = rays.reshape(-1, 3)
		rays_global_flat = quaternion.rotate_vectors(q, rays_local_flat)
		rays_global = rays_global_flat.reshape(rays.shape)
		return rays_global

	def check_unity(self, rays = None):
		if rays is None:
			rays = self.rays
		directions = rays[:, :, 3:]
		return np.allclose(np.linalg.norm(directions, axis=2), 1)

	def plot_rays(self, rays=None, fig=None, visualize=True, plot_unit_sphere=True):
		if fig is None:
			fig = plt.figure()
		if rays is None:
			rays = self.rays
		# Extract x, y, and z coordinates from the data
		x = rays=None[:, :, 3].flatten()
		y = rays=None[:, :, 4].flatten()
		z = rays=None[:, :, 5].flatten()

		# Create a 3D plot
		ax = fig.add_subplot(111, projection='3d')
		ax.scatter(x, y, z, c='b', marker='+')

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