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
from lmao_lib.lidar import Lidar


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
			print("World loaded from file: ", world_file)
		else:
			self.world = o3d.geometry.TriangleMesh()
			print("Empty world created.")

		self.scene = o3d.t.geometry.RaycastingScene()
		self.scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(self.world))

	def boolean_mesh(self, mesh, boolean_operation = "union", tolerance = 1e-6):
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

	def add_mesh_to_scene(self, meshes):
		# Add a meshes to world
		for mesh in meshes:
			if not isinstance(mesh, o3d.cuda.pybind.t.geometry.TriangleMesh):
				raise TypeError("Mesh must be of type TriangleMesh. Mesh is of type: ", type(mesh))
			else:
				self.scene.add_triangles(mesh)

	def cast_rays(self, lidar):
		# Print type of lidar
		return self.scene.cast_rays(lidar)
		#return self.scene.cast_rays(lidar.rays if isinstance(lidar, Lidar) else lidar)


	def get_random_points(self, n):
		# Get the bounding box of the world
		bbox = self.world.get_axis_aligned_bounding_box()
		# Get min x y and z
		min_x = bbox.get_min_bound()[0]
		min_y = bbox.get_min_bound()[1]
		min_z = bbox.get_min_bound()[2]
		# Get max x y and z
		max_x = bbox.get_max_bound()[0]
		max_y = bbox.get_max_bound()[1]
		max_z = bbox.get_max_bound()[2]
		# Create a list of random points
		points = np.random.uniform([min_x, min_y, min_z], [max_x, max_y, max_z], size=(n, 3))
		# Create a point cloud from the points
		pcd = o3d.geometry.PointCloud()
		pcd.points = o3d.utility.Vector3dVector(points)
		# Color points red
		pcd.paint_uniform_color([1, 0, 0])
		# Show world and points
		o3d.visualization.draw_geometries([self.world, pcd])


	def get_probable_random_points(self, n, lidar_height=1):
		# Create copy of world
		world = self.world
		print("World has normals: ", world.has_triangle_normals(), "\nWorld has vertex normals: ", world.has_vertex_normals())
		# Check if world triangles has normals
		if not world.has_triangle_normals():
			print("Computing normals")
			# Compute normals
			world.compute_triangle_normals()
		else:
			print("World has normals")
		
		# Remove all triangles with normal z < 0.8
		print("Removing triangles with normal z < 0.8")
		world.remove_triangles_by_mask(np.asarray(world.triangle_normals)[:, 2] < 0.75)
		world.remove_unreferenced_vertices()
		world = world.remove_degenerate_triangles()

		# Find all non manifold edges and show them in red
		non_manifold_edges_indicis = world.get_non_manifold_edges(allow_boundary_edges = False)
		non_manifold_vertices_indicis = world.get_non_manifold_vertices()
		#print("Non manifold edges indicis: ", non_manifold_edges_indicis)
		#print("Non manifold vertices indicis: ", non_manifold_vertices_indicis)
		
		# Show non manifold vertices
		non_manifold_vertices = o3d.geometry.PointCloud()
		non_manifold_vertices.points = o3d.utility.Vector3dVector(np.asarray(world.vertices)[non_manifold_vertices_indicis])
		non_manifold_vertices.paint_uniform_color([1, 0, 0])

		# Show non manifold edges
		non_manifold_edges = o3d.geometry.LineSet()
		non_manifold_edges.points = o3d.utility.Vector3dVector(np.asarray(world.vertices))
		non_manifold_edges.lines = non_manifold_edges_indicis
		non_manifold_edges.paint_uniform_color([0, 1, 0])

		# Sample points with poisson disk sampling
		print("Sampling {} points with poisson disk sampling".format(n))
		pcd = world.sample_points_poisson_disk(n)
		# Color points red
		pcd.paint_uniform_color([1, 0, 0])
		pcd.normalize_normals()
		# Move all points by the normal assigned to them
		pcd.points = o3d.utility.Vector3dVector(np.asarray(pcd.points) + np.asarray(pcd.normals) * lidar_height)

		# Show world and points
		#o3d.visualization.draw_geometries([world, pcd])

	
		# Show world
		# o3d.visualization.draw_geometries([world, non_manifold_vertices, non_manifold_edges])

		return np.asarray(pcd.points)

			






