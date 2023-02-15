# Welcome to LMAO - Localization Mapping and Optimization (or Odometry)
# This is a library that will include the classes and functions needed to perform localization, mapping and optimization in a 3D environment.
# The library is built on top of Open3D, and uses raycasting in a triangle mesh.
# It is made by Rasmus Peter Junge, and Daniel Gahner Holm for our Master's Thesis at SDU.
#
# The library is divided into the following modules:


import open3d as o3d
import numpy as np
import os

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class World:
	def __init__(self, world_file = None):
		# Initialize the world.
		# The world is represented as a Triangle Mesh.
		# The world is loaded from a file, if no file is given, an empty world is created.
		if world_file:
			# Test if world file exists.
			if not os.path.isfile(world_file):
				raise FileNotFoundError("World file not found.")
			self.world = o3d.io.read_triangle_mesh(world_file)
		else:
			self.world = o3d.geometry.TriangleMesh()

		self.scene = o3d.t.geometry.RaycastingScene()

	def add_mesh(self, mesh, boolean_operation = "union", tolerance = 1e-6):
		# Add a mesh to the world.
		# The boolean_operation argument specifies the type of merge.
		# The default is a union merge.
		# The boolean_operation argument can be one of the following:
		# "union" - The mesh is added to the world.
		# "intersection" - The mesh is added to the world, but only where it intersects with the world.
		# "difference" - The mesh is removed from the world.

		if boolean_operation == "union":
			self.world = self.world.boolean_union(mesh, tolerance)
		elif boolean_operation == "intersection":
			self.world = self.world.boolean_intersection(mesh, tolerance)
		elif boolean_operation == "difference":
			self.world = self.world.boolean_difference(mesh, tolerance)


	def generate_lidar_rays(horizontal_lines=64, rays_per_line=1024, vertical_fov = np.pi/2, offset = 0):
		rays = np.zeros((horizontal_lines,rays_per_line, 6), dtype=np.float32)
		rays_optimized = np.zeros((horizontal_lines,rays_per_line, 6), dtype=np.float32)

		rot_x = np.sin(np.linspace(0 + offset, 2*np.pi, rays_per_line + offset, endpoint=False))
		rot_y = np.cos(np.linspace(0 + offset, 2*np.pi, rays_per_line + offset, endpoint=False))

		# dz = np.cos(np.linspace((np.pi)/4, 3*np.pi/4, horizontal_lines))
		dz = np.cos(np.linspace((np.pi)/2 - vertical_fov/2, (np.pi)/2 + vertical_fov/2, horizontal_lines, endpoint=True))
		dz_scale = np.sin(np.linspace((np.pi)/2 - vertical_fov/2, (np.pi)/2 + vertical_fov/2, horizontal_lines, endpoint=True))

		# Instead of for loops, directly populate the rays array.
		# rays[:, :, 0] = 0
		# rays[:, :, 1] = 3
		# rays[:, :, 2] = 1
		rays[:, :, 3] = rot_x * dz_scale[:, None]
		rays[:, :, 4] = rot_y * dz_scale[:, None]
		rays[:, :, 5] = dz[:, None]

		return rays

	def check_unity(rays):
		directions = rays[:, :, 3:]
		return np.allclose(np.linalg.norm(directions, axis=2), 1)

	def plot_rays(data, fig=None, visualize=True, plot_unit_sphere=True):
		if fig is None:
			fig = plt.figure()
		# Extract x, y, and z coordinates from the data
		x = data[:, :, 3].flatten()
		y = data[:, :, 4].flatten()
		z = data[:, :, 5].flatten()

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






