# Welcome to LMAO - Localization Mapping and Optimization (or Odometry)
# This is a library that will include the classes and functions needed to perform localization, mapping and optimization in a 3D environment.
# The library is built on top of Open3D, and uses raycasting in a triangle mesh.
# It is made by Rasmus Junge, and Daniel Gahner Holm for our Master's Thesis at SDU.
#
# The library is divided into the following modules:


import open3d as o3d
import numpy as np
import os

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


	def generate_lidar_rays(horizontal_lines=64, rays_per_line=1024, vertical_fov = np.pi/2):
		rays = np.zeros((horizontal_lines,rays_per_line, 6), dtype=np.float32)
		rays_optimized = np.zeros((horizontal_lines,rays_per_line, 6), dtype=np.float32)

		rot_x = np.sin(np.linspace(0, 2*np.pi, rays_per_line))
		rot_y = np.cos(np.linspace(0, 2*np.pi, rays_per_line))

		# dz = np.cos(np.linspace((np.pi)/4, 3*np.pi/4, horizontal_lines))
		dz = np.cos(np.linspace((np.pi)/2 - vertical_fov/2, (np.pi)/2 + vertical_fov/2, horizontal_lines))
		dz_scale = np.sin(np.linspace((np.pi)/2 - vertical_fov/2, (np.pi)/2 + vertical_fov/2, horizontal_lines))

		# Instead of for loops, directly populate the rays array.
		# rays[:, :, 0] = 0
		# rays[:, :, 1] = 3
		# rays[:, :, 2] = 1
		rays[:, :, 3] = rot_x * dz_scale[:, None]
		rays[:, :, 4] = rot_y * dz_scale[:, None]
		rays[:, :, 5] = dz[:, None]

		return rays


		






