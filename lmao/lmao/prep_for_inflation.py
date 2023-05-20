import numpy as np
import open3d as o3d



def prep_world_for_random(world):
		print("World has normals: ", world.has_triangle_normals(), "\nWorld has vertex normals: ", world.has_vertex_normals())
		# Check if world triangles has normals
		if not world.has_triangle_normals():
			print("Computing normals")
			# Compute normals
			world.compute_triangle_normals()
		else:
			print("World has normals")

		# Construct K-d tree for nearest neighbor search
		kdtree = o3d.geometry.KDTreeFlann(world)
		
		# Remove all triangles with normal z < 0.8
		print("Removing triangles with normal z < 0.8")
		world.remove_triangles_by_mask(np.asarray(world.triangle_normals)[:, 2] < 0.75)
		world.remove_unreferenced_vertices()
		world = world.remove_degenerate_triangles()

		# Find all non manifold edges and show them in red
		non_manifold_edges_indicis = world.get_non_manifold_edges(allow_boundary_edges = False)
		
		# non_manifold_edges_indicies is open3d.cuda.pybind.utility.Vector2iVector
		non_manifold_indices = np.array(non_manifold_edges_indicis).ravel()
		
		print("Shape of non manifold indices: ", non_manifold_indices.shape)
		non_manifold_indices = np.unique(non_manifold_indices)
		print("Shape of non manifold indices after unique: ", non_manifold_indices.shape)

		# Create pcd of non manifold indices
		non_manifold_indices_pcd = o3d.geometry.PointCloud()
		non_manifold_indices_pcd.points = o3d.utility.Vector3dVector(np.asarray(world.vertices)[non_manifold_indices])
		non_manifold_indices_pcd.paint_uniform_color([0, 1, 0])

		# Show world and non manifold edges and vertices
		o3d.visualization.draw_geometries([world, non_manifold_indices_pcd])