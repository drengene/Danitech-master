import open3d as o3d

import numpy as np
import matplotlib.pyplot as plt
from time import time, sleep
import cv2
from lmao.world import World
from lmao.lidar import Lidar
import quaternion
from scipy.spatial.transform import Rotation as R
from lmao.mapping import Map, get_normals
from scipy import ndimage
from scipy.spatial import Delaunay
from lmao.util.bag_loader import BagLoader


def convert_to_cv2_image(depth, funnycolor=False, normalize=False):
	depth[depth == np.inf] = 0
	depth = depth / np.max(depth)
	depth = depth * 255
	depth = depth.astype(np.uint8)
	if funnycolor:
		depth = cv2.applyColorMap(depth, cv2.COLORMAP_JET)
	return depth

def show_depth(depth):
	depth = convert_to_cv2_image(depth)
	cv2.imshow("Depth", depth)
	cv2.waitKey(1)


def remove_significant_differences_angle(input_array, threshold_degrees):
	# Time for remove_significant_differences_angle:  6.224948596954346
	threshold_cos = np.cos(np.deg2rad(threshold_degrees))
	output_array = np.zeros_like(input_array)
	for y in range(1, input_array.shape[0]-1):
		for x in range(1, input_array.shape[1]-1):
			neighbors = input_array[y-1:y+2, x-1:x+2]
			neighbor_norms = np.linalg.norm(neighbors, axis=2)
			input_norm = np.linalg.norm(input_array[y, x])
			cos_angles = np.sum(neighbors * input_array[y, x], axis=2) / (input_norm * neighbor_norms)
			sin_angles = np.linalg.norm(np.cross(neighbors, input_array[y, x]), axis=2) / (input_norm * neighbor_norms)
			angles = np.rad2deg(np.arctan2(sin_angles, cos_angles))
			max_angle = np.max(angles)
			if max_angle > threshold_degrees:
				output_array[y, x] = input_array[y, x]
			else:
				output_array[y, x] = 0
	return output_array

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


if __name__ == "__main__":
	# Create meshes and convert to open3d.t.geometry.TriangleMesh .
	cube = o3d.geometry.TriangleMesh.create_box().translate([0, 0, 0])
	cube.scale(2, center=cube.get_center())
	cube = o3d.t.geometry.TriangleMesh.from_legacy(cube)
	torus = o3d.geometry.TriangleMesh.create_torus().translate([0, 2, 2])
	torus = o3d.t.geometry.TriangleMesh.from_legacy(torus)
	sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.5).translate([0, 0, 2])
	sphere2 = o3d.geometry.TriangleMesh.create_sphere(radius=4).translate([2, 3, 1])

	bunny = o3d.data.BunnyMesh()
	mesh = o3d.io.read_triangle_mesh(bunny.path).translate([0, 0, 2])
	mesh.compute_vertex_normals()
	mesh.scale(20, center=mesh.get_center())
	bunny = o3d.t.geometry.TriangleMesh.from_legacy(mesh)

	sphere = o3d.t.geometry.TriangleMesh.from_legacy(sphere)
	sphere2 = o3d.t.geometry.TriangleMesh.from_legacy(sphere2)
	
	# Create a world.
	world = World()
	# Add meshes to world.
	world.add_mesh_to_scene([cube, torus, sphere, sphere2])
	digital_twin = World()
	digital_twin.add_mesh_to_scene([cube, torus, bunny, sphere2])

	# Create a lidar.
	lidar = Lidar()
	noisy_lidar = Lidar()

	# Translate rays.
	lidar.translate_rays([2, 3, 0], change_original=True)
	noisy_lidar.translate_rays([2, 3, 0], change_original=True)

	# Add noise to the lidar.
	noisy_lidar.noise(change_original=True)

	#lidar.plot_rays(lidar.rays)
	#sleep(1)
	#noisy_lidar.plot_rays(noisy_lidar.rays)


	t0 = time()
	# Rotate rays.
	q = np.random.rand(4)
	q /= np.linalg.norm(q)
	#q = [0,0,0,1]
	rotated = lidar.rotate_rays(q)
	rotated_noisy = noisy_lidar.rotate_rays(q)

	#print("Size of rotated: ", rotated.shape)
	#print("Size of rotated_noisy: ", rotated_noisy.shape)

	# Cast rays.
	
	


	# Cast in the digital twin.
	tens = o3d.core.Tensor(rotated)
	ans = digital_twin.cast_rays(tens)
	depth = ans['t_hit'].numpy()
	

	# Cast in simulated real world
	tens_noisy = o3d.core.Tensor(rotated_noisy)
	ans = world.cast_rays(tens_noisy)


	print("Keys in ans: ", ans.keys())
	depth_noisy = ans['t_hit'].numpy()
	hit = ans['t_hit'].isfinite()
	points = tens_noisy[hit][:,:3] + tens_noisy[hit][:,3:]*ans['t_hit'][hit].reshape((-1,1))
	pcd = o3d.t.geometry.PointCloud(points)
	# Convert to open3d.geometry.PointCloud
	pcd = pcd.to_legacy()
	# pcd.estimate_normals(
	# 	search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30))
	# print(type(pcd))
	origin = o3d.geometry.PointCloud()
	origin.points = o3d.utility.Vector3dVector([[2, 3, 0]])
	origin.colors = o3d.utility.Vector3dVector([[1, 0, 0]])
	#o3d.visualization.draw_geometries([pcd, origin], point_show_normal=True)

	# Apply numpy median filter to depth image
	depth_noisy = ndimage.median_filter(depth_noisy, size=5)
	# Apple gaussian filter to depth image
	depth_noisy = ndimage.gaussian_filter(depth_noisy, sigma=1)
	depth_noisy = ndimage.median_filter(depth_noisy, size=3)

	# Transform depth points to world coordinates.
	image = rotated[:,:,:3] + depth_noisy[:,:,np.newaxis]*rotated[:,:,3:]
	
	bagloader = BagLoader(bagpath = '/home/junge/Documents/rosbag2_2023_03_09-15_53_39',topics = ['/points'])
	image, depth = bagloader.get_pointcloud('/points')
	min_depth = np.min(depth)
	max_depth = np.max(depth)
	print("Min depth: ", min_depth, "Max depth: ", max_depth)
	# plt.imshow(abs(image))
	# plt.show()
	# plt.imshow(depth)
	# plt.show()

	newmap = Map()
	mesh = newmap.create_mesh(image, depth)

	o3d.visualization.draw_geometries([mesh])


	exit()




	normal_image = get_normals(image)
	# Extend image by one pixel up and down
	normal_image = np.concatenate((normal_image[0:1,:,:], normal_image, normal_image[-1:,:,:]), axis=0)
	print("Shape of normal_image after extension: ", normal_image.shape)

	# # Create matplotlib figure with 2 subplots
	# fig, axs = plt.subplots(2, 1)

	# plt.imshow(normal_image)
	# axs[0].imshow(abs(normal_image))
	# axs[0].set_title("Normals from gradient")
	# plt.show()

	# Compare with the normals from the original raycast
	actual_normals = ans['primitive_normals'].numpy()

	# Show actual normals with matplotlib
	# axs[1].imshow(abs(actual_normals))
	# axs[1].set_title("Actual normals from raycast")
	plt.show()

	# Remove salt and pepper noise from estimated normals
	result = np.zeros_like(normal_image)
	for channel in range(3):
		result[:, :, channel] = ndimage.median_filter(normal_image[:, :, channel], size=3)
	
	# Show the result
	# fig, axs = plt.subplots(2, 1)
	# axs[0].imshow(abs(result))
	# axs[0].set_title("Result after median filter")
	# axs[1].imshow(abs(actual_normals))
	# axs[1].set_title("Actual normals from raycast")
	# plt.show()

	# Wrapping image for edges.
	#result = np.concatenate((result[:, int(result.shape[1]/2):] ,result, result[:, :int(result.shape[1]/2)]), axis=1)
	
	#clean = remove_significant_differences_angle_optim(result, 5)
	#clean = remove_significant_differences_noangle(result, 0.1)
	clean = remove_significant_differences_noangle(result, 0.2)

	# Time "remove_significant_differences_angle" and "remove_significant_differences_noangle"
	# t0 = time()
	# for i in range(10):
	# 	clean = remove_significant_differences_angle(result, 5)
	# t1 = time()
	# print("Time for remove_significant_differences_angle: ", (t1-t0)/10)

	# t0 = time()
	# for i in range(10):
	# 	clean = remove_significant_differences_noangle(result, 0.1)
	# t1 = time()
	# print("Time for remove_significant_differences_noangle: ", (t1-t0)/10)

	fig, axs = plt.subplots(3, 1)
	axs[0].imshow(abs(clean))
	axs[0].set_title("Result after removing significant differences")
	axs[1].imshow(abs(result))
	axs[1].set_title("Estimated normals from lidar")
	axs[2].imshow(abs(actual_normals))
	axs[2].set_title("Actual surface normals from raycast")
	plt.show()

	# Get a binary mask of the points that are not the null vector in the "clean" image
	mask = np.logical_and(np.logical_and(clean[:, :, 0] != 0, clean[:, :, 1] != 0), clean[:, :, 2] != 0)
	
	# Get binary mask of points that are 0 in image[:, :, 8]
	mask2 = depth > 500
	hist = np.histogram(depth, bins=100)
	# Show histogram of depth values
	plt.plot(hist[1][:-1], hist[0])
	plt.show()


	# Show the mask
	# plt.imshow(mask2)
	# give a title
	# plt.title("Mask of points that are greater than 500mm in distance")
	# plt.show()

	# Show the mask
	# plt.imshow(mask)
	# give a title
	# plt.title("Mask of points that are not the null vector in the estimated normals")
	# plt.show()

	# Combine the two masks
	mask = np.logical_and(mask, np.concatenate((mask2[0:1,:], mask2, mask2[-1:,:]), axis=0))

	# Show the wrapped mask
	# plt.imshow(mask)
	# plt.show()

	# Set the top and bottom row in mask to 1
	mask[0, :] = 1
	mask[-1, :] = 1

	# Show the wrapped mask
	plt.imshow(mask)
	plt.show()

	# Perform delaunay triangulation on the points that are not the null vector, using the mask pixel positions as the x and y coordinates
	tri = Delaunay(np.array(np.nonzero(mask)).T)

	# Perform delaunay triangulation on the points that are not the null vector, x, y and z from the image as the coordinates
	# I think it will be something like this: tri = Delaunay(image[mask])
	#tri = Delaunay(image[np.nonzero(mask)].reshape(-1, 3))

	print("Number of triangles: ", len(tri.simplices))
	print("dtype: ", tri.points.dtype)
	print("Number of points: ", len(tri.points))
	print("Max x: ", np.max(tri.points[:, 0]), "Min x: ", np.min(tri.points[:, 0]))
	print("Max y: ", np.max(tri.points[:, 1]), "Min y: ", np.min(tri.points[:, 1]))
	# Show the triangulation and the original image in the background
	#plt.imshow(mask)
	#plt.triplot(tri.points[:, 1], tri.points[:, 0],  tri.simplices.copy())
	#plt.plot(tri.points[:, 0], tri.points[:, 1], 'o')
	#plt.show()

	print("Shape of tri.points: ", tri.points.shape)
	print("Shape of points: ", points.shape)

	print("Shape of tri.simplexes: ", tri.simplices.shape)


	vertices = np.array(image[tri.points[:, 0].astype(int), tri.points[:, 1].astype(int), :3] )
	#print("Size of vertices: ", vertices.shape)
	#print("Type of vertices: ", vertices.dtype)

	# Create a triangle mesh from the triangulation
	mesh = o3d.geometry.TriangleMesh()
	# Create vertices using the points from the triangulation as indeces to the "points" matrix
	mesh.vertices = o3d.utility.Vector3dVector(vertices)
	#mesh.vertices = o3d.utility.Vector3dVector(tri.points)
	# Create triangles using the simplices from the triangulation
	mesh.triangles = o3d.utility.Vector3iVector(tri.simplices[:, ])
	# Add edges

	# Paint the mesh in a color defined by the normals
	mesh.vertex_colors = o3d.utility.Vector3dVector(abs(normal_image[tri.points[:, 0].astype(int), tri.points[:, 1].astype(int)]))
	#tri_tens = o3d.core.Tensor(np.array(points[1:5]))
	
	# Plot 3d points tri.points with matplotlib


	# Show the mesh
	o3d.visualization.draw_geometries([mesh])



	# Get max and min values.
	# print("Max: ", np.max(points), "Min: ", np.min(points))
	# # Get the points that are not inf.
	# hit = depth_noisy != np.inf
	# # Get max and min values of the points that are not inf.
	# print("Max: ", np.max(points[hit]), "Min: ", np.min(points[hit]))
	# max = np.max(points[hit])
	# min = np.min(points[hit])
	# # Normalize the points.
	# points = (points - min)/(max - min)
	# fig, axs = plt.subplots(3,1)
	# axs[0].imshow(points[:,:,0])
	# axs[1].imshow(points[:,:,1])
	# axs[2].imshow(points[:,:,2])
	# plt.show()
	# print("Points shape: ", points.shape)
	# #grad = np.gradient(points)
	# grad = (np.gradient(points[:,:,0]), np.gradient(points[:,:,1]), np.gradient(points[:,:,2]))
	# #print("Grad shape: ", grad.shape)
	# # Create 3 plots for the 3 channels.
	# fig, axs = plt.subplots(3,1)
	# # Plot the first channel. Normalized.
	# max, min = np.max(grad[0]), np.min(grad[0])
	# im = np.zeros((3, 64, 1024))
	# im[0:2, :, :] = (grad[0] - min)/(max - min)
	# axs[0].imshow(np.transpose(im, (1, 2, 0)))
	# # Plot the second channel. Normalized.
	# max, min = np.max(grad[1]), np.min(grad[1])
	# im = np.zeros((3, 64, 1024))
	# im[0:2, :, :] = (grad[1] - min)/(max - min)
	# axs[1].imshow(np.transpose(im, (1, 2, 0)))
	# # Plot the third channel. Normalized.
	# max, min = np.max(grad[2]), np.min(grad[2])
	# im = np.zeros((3, 64, 1024))
	# im[0:2, :, :] = (grad[2] - min)/(max - min)
	# axs[2].imshow(np.transpose(im, (1, 2, 0)))
	# plt.show()
	# # plt.imshow(grad[0])
	# # plt.show()

	
	# print("Points shape: ", points.shape)
	# #ans['t_hit']
	# print("Hit shape: ", ans['t_hit'].shape)

	# gradient = get_gradient(depth_noisy)
	# # Gradient returns an array of shape (2, height, width)
	# print("Gradient min: ", np.min(gradient[0]), "Gradient max: ", np.max(gradient[0]))
	# # Set all inf values to 0.
	# print(gradient[0].shape)
	# np.nan_to_num(gradient[0], copy=False, posinf=0)
	# np.nan_to_num(gradient[1], copy=False, posinf=0)
	# # We visualize the two channels separately.
	# plt.imshow(gradient[0], cmap='gray')
	# plt.show()
	# plt.imshow(gradient[1], cmap='gray')
	# plt.show()
	# # And now in one image with RG channels, while B is zero.
	# data_rg = np.zeros((depth_noisy.shape[0], depth_noisy.shape[1], 3))
	# data_rg[:,:,0] = gradient[0]
	# data_rg[:,:,1] = gradient[1]
	# plt.imshow(data_rg)
	# plt.show()




	# with o3d.utility.VerbosityContextManager(
	# 		o3d.utility.VerbosityLevel.Debug) as cm:
	# 	mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
	# 		pcd, depth=6)
	# # print(mesh)
	# o3d.visualization.draw_geometries([mesh],
	# 								zoom=0.664,
	# 								front=[-0.4761, -0.4698, -0.7434],
	# 								lookat=[1.8900, 3.2596, 0.9284],
	# 								up=[0.2304, -0.8825, 0.4101])

	# print('visualize densities')
	# densities = np.asarray(densities)
	# density_colors = plt.get_cmap('plasma')(
	# 	(densities - densities.min()) / (densities.max() - densities.min()))
	# density_colors = density_colors[:, :3]
	# density_mesh = o3d.geometry.TriangleMesh()
	# density_mesh.vertices = mesh.vertices
	# density_mesh.triangles = mesh.triangles
	# density_mesh.triangle_normals = mesh.triangle_normals
	# density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)
	# o3d.visualization.draw_geometries([density_mesh],
	# 								zoom=0.664,
	# 								front=[-0.4761, -0.4698, -0.7434],
	# 								lookat=[1.8900, 3.2596, 0.9284],
	# 								up=[0.2304, -0.8825, 0.4101])
	# print("The end")

	
	# print(type(depth))
	# # Use matplotlib to visualize depth in full resolution.
	# plt.imshow(depth_noisy, cmap='gray' )
	# plt.show()
	# print("Min: ", np.min(depth_noisy), "Max: ", np.max(depth_noisy))
	# gradient = get_gradient(depth_noisy)
	# # Gradient returns an array of shape (2, height, width)
	# print("Gradient min: ", np.min(gradient[0]), "Gradient max: ", np.max(gradient[0]))
	# # We visualize the two channels separately.
	# plt.imshow(gradient[0], cmap='gray')
	# plt.show()
	# plt.imshow(gradient[1], cmap='gray')
	# plt.show()

	# # And now in one image with RG channels, while B is zero.
	# data_rg = np.zeros((depth_noisy.shape[0], depth_noisy.shape[1], 3))
	# data_rg[:,:,0] = gradient[0]
	# data_rg[:,:,1] = gradient[1]
	# plt.imshow(data_rg)
	# plt.show()


	# image = convert_to_cv2_image(depth)
	# image_noisy = convert_to_cv2_image(depth_noisy)

	

	# print(type(depth))
	# print(depth.dtype)

	# t0 = time()
	# cv2.imshow("depth", image)
	# cv2.imshow("depth_noisy", image_noisy)

	# #find edges
	# edges = find_img_edges(image_noisy)
	# cv2.imshow("edges", edges)

	# # Apply sobel filter to image
	# gradient = get_img_derivative(image_noisy)
	# cv2.imshow("gradient", gradient)


	# cv2.imshow("difference normalized", cv2.absdiff(image, image_noisy))

	# # Apply smoothing to differennce image.
	# kernel = np.ones((5, 5), np.float32) / 25
	# smoothed = cv2.filter2D(cv2.absdiff(image, image_noisy), -1, kernel)
	# cv2.imshow("difference smoothed", smoothed)
	# cv2.waitKey(0)

	# plt.show()


	

	

	# #Convert to 


	# # # Test 1000 times and get the average time.
	# # t0 = time()
	# # for _ in range(1000):
	# #     ans = scene.cast_rays(rays)
	# # t1 = time()

	# # print(f"Time: {(t1 - t0) / 1000:.3f} s")
	# # print("Giving a framerate of ", 1 / ((t1 - t0) / 1000), "fps")

	

	# # Visualize the results.
	# # plt.imshow(ans['t_hit'].numpy())
	# # plt.show()
	# # plt.imshow(np.abs(ans['primitive_normals'].numpy()))
	# # plt.show()
	# # plt.imshow(np.abs(ans['geometry_ids'].numpy()), vmax=3)
	# # plt.show()