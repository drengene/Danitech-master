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


def remove_significant_differences_noangle(original_array, threshold):
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


def get_normals(image):
	# Takes as input a [m,n,3] of points in 3d space.
	# Compute the gradients in the x and y directions
	# dx = np.gradient(image[:, :, 0]) # Returns a
	# dy = np.gradient(image[:, :, 1])
	dx, dy, _ = np.gradient(image)

	print("dx: ", dx.shape)
	print("dy: ", dy.shape)

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