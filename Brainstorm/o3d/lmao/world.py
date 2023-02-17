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







