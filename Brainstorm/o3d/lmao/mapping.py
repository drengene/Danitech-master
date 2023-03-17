# -------------------
#
# Welcome to LMAO - Localization Mapping and Optimization (or Odometry)
# This is a library that will include the classes and functions needed to perform localization, mapping and optimization in a 3D environment.
# The library is built on top of Open3D, and uses raycasting in a triangle mesh.
# It is made by Rasmus Peter Junge, and Daniel Gahner Holm for our Master's Thesis at SDU.
#
# -------------------

# Path: lmao/mapping.py

import cv2
import numpy as np
from scipy.spatial import Delaunay
import open3d as o3d
from scipy import ndimage

class Map:
	def __innit__(self):
		self.mesh = None

	def create_mesh(self, xyz, depth):
		# Get the normals of the point cloud
		normal_image = get_normals(xyz)
		# Extend normal image to include border points
		normal_image = np.concatenate((normal_image[0:1,:,:], normal_image, normal_image[-1:,:,:]), axis=0)
		# Remove salt and pepper noise from estimated normals # Experimental
		result = np.zeros_like(normal_image)
		for channel in range(3):
			result[:, :, channel] = ndimage.median_filter(normal_image[:, :, channel], size=3)

		# Remove points that are not significantly different from their neighbors
		clean = remove_significant_differences(result, 0.2)

		# Get a binary mask of the points that are not the null vector in the "clean" image
		mask = np.logical_and(np.logical_and(clean[:, :, 0] != 0, clean[:, :, 1] != 0), clean[:, :, 2] != 0)

		# Get binary mask of points that are not too close to the camera
		mask2 = depth > 500

		# Combine the two masks
		mask = np.logical_and(mask, np.concatenate((mask2[0:1,:], mask2, mask2[-1:,:]), axis=0))

		# Set the top and bottom row in mask to 1
		mask[0, :] = 1
		mask[-1, :] = 1

		# Find the triangulation of the points in the mask
		tri = Delaunay(np.array(np.nonzero(mask)).T)
		points = tri.points
		simplices = tri.simplices

		# Remove all tri.points in the top and bottom row
		points = points[points[:, 0] != 0]
		points = points[points[:, 0] != mask.shape[0]-1]

		# Replace all indexes in tri.simplices to the top row with 0
		simplices[simplices[:, 0] < mask.shape[0]-1] = 0
		# Replace all indexes in tri.simplices to the bottom row with 1
		tri.simplices[tri.simplices[:, 0] == mask.shape[0]-1] = 1

		# Remove all tri.simplices that contain more than one 0 or 1

		# Remove reference to added border
		points[:, 0] = points[:, 0] - 1

		# Count how many points have y = 0 or y = - 1
		count = np.count_nonzero(points[:, 0] == 0)
		count2 = np.count_nonzero(points[:, 0] == -1)

		print("Number of points with y = 0: ", count)
		print("Number of points with y = -1: ", count2)
		


		vertices = np.array(xyz[points[:, 0].astype(int), points[:, 1].astype(int), :3] )
		# Create a triangle mesh from the triangulation
		mesh = o3d.geometry.TriangleMesh()
		# Create vertices using the points from the triangulation as indeces to the "points" matrix
		mesh.vertices = o3d.utility.Vector3dVector(vertices)
		# Create triangles using the simplices from the triangulation
		mesh.triangles = o3d.utility.Vector3iVector(simplices)
		# Add edges
		mesh.vertex_colors = o3d.utility.Vector3dVector(abs(normal_image[points[:, 0].astype(int), points[:, 1].astype(int)]))
		
		return mesh

		





def get_normals(image):
	# Takes as input a [m,n,3] of points in 3d space.
	# Compute the gradients in the x and y directions
	dx, dy, _ = np.gradient(image)

	# Compute the cross product of the gradients to get the normal vector
	normal = np.cross(dx, dy)

	# Normalize the normal vector to have unit length
	normal_norm = np.linalg.norm(normal, axis=2)
	normal = np.divide(normal, np.stack([normal_norm, normal_norm, normal_norm], axis=2))

	# Create a new image to store the normal vectors
	normal_image = np.zeros_like(image)

	# Set the normal vectors in the new image
	normal_image[:, :, 0] = normal[:, :, 0]
	normal_image[:, :, 1] = normal[:, :, 1]
	normal_image[:, :, 2] = normal[:, :, 2]

	return normal_image


def remove_significant_differences(original_array, threshold):
	# Sets as the null vector, all the points in the array that are within a 
	#  threshold distance from their neighbors.

	# Time for remove_significant_differences_noangle:  0.9915423154830932
	null_vector = np.zeros((3,))
	result_array = np.copy(original_array)
	# iterate over each point in the array
	for i in range(1, original_array.shape[0]-1):
		for j in range(1, original_array.shape[1]-1):
			# calculate the distance between the normal vector of the current point and its neighbors
			distance = np.linalg.norm(original_array[i, j] - original_array[i-1:i+2, j-1:j+2], axis=1)
			
			# check if all the distances are below the threshold
			if np.all(distance < threshold):
				result_array[i, j] = null_vector
	return result_array


def construct_mesh(mask, xyz, normal_image=None):
	tri = Delaunay(np.array(np.nonzero(mask)).T)
	vertices = np.array(xyz[tri.points[:, 0].astype(int), tri.points[:, 1].astype(int), :3] )
		# Create a triangle mesh from the triangulation
	mesh = o3d.geometry.TriangleMesh()
	# Create vertices using the points from the triangulation as indeces to the "points" matrix
	mesh.vertices = o3d.utility.Vector3dVector(vertices)
	# Create triangles using the simplices from the triangulation
	mesh.triangles = o3d.utility.Vector3iVector(tri.simplices)
	# Add edges
	if normal_image is not None:
		# Paint the mesh in a color defined by the normals
		mesh.vertex_colors = o3d.utility.Vector3dVector(abs(normal_image[tri.points[:, 0].astype(int), tri.points[:, 1].astype(int)]))
	return mesh

