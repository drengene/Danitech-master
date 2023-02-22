import open3d as o3d

import numpy as np
import matplotlib.pyplot as plt
from time import time, sleep
import cv2
from lmao.world import World
from lmao.lidar import Lidar
import quaternion
from scipy.spatial.transform import Rotation as R

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

if __name__ == "__main__":
	# Create meshes and convert to open3d.t.geometry.TriangleMesh .
	cube = o3d.geometry.TriangleMesh.create_box().translate([0, 0, 0])
	cube = o3d.t.geometry.TriangleMesh.from_legacy(cube)
	torus = o3d.geometry.TriangleMesh.create_torus().translate([0, 2, 2])
	torus = o3d.t.geometry.TriangleMesh.from_legacy(torus)
	sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.5).translate([0, 0, 2])
	sphere2 = o3d.geometry.TriangleMesh.create_sphere(radius=10).translate([2, 3, 1])

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
	world.add_mesh_to_scene([cube, torus, sphere])
	digital_twin = World()

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
	depth_noisy = ans['t_hit'].numpy()
	hit = ans['t_hit'].isfinite()
	points = tens_noisy[hit][:,:3] + tens_noisy[hit][:,3:]*ans['t_hit'][hit].reshape((-1,1))
	pcd = o3d.t.geometry.PointCloud(points)
	origin = o3d.geometry.PointCloud()
	origin.points = o3d.utility.Vector3dVector([[2, 3, 0]])
	origin.colors = o3d.utility.Vector3dVector([[1, 0, 0]])
	o3d.visualization.draw([pcd, origin])
	# Also draw a red point at origin.



	print(type(depth))
	# Use matplotlib to visualize depth in full resolution.
	plt.imshow(depth_noisy, cmap='gray' )
	plt.show()
	print("Min: ", np.min(depth_noisy), "Max: ", np.max(depth_noisy))


	image = convert_to_cv2_image(depth)
	image_noisy = convert_to_cv2_image(depth_noisy)

	

	print(type(depth))
	print(depth.dtype)

	t0 = time()
	cv2.imshow("depth", image)
	cv2.imshow("depth_noisy", image_noisy)


	cv2.imshow("difference normalized", cv2.absdiff(image, image_noisy))

	# Apply smoothing to differennce image.
	kernel = np.ones((5, 5), np.float32) / 25
	smoothed = cv2.filter2D(cv2.absdiff(image, image_noisy), -1, kernel)
	cv2.imshow("difference smoothed", smoothed)
	cv2.waitKey(0)

	plt.show()


	

	

	#Convert to 


	# # Test 1000 times and get the average time.
	# t0 = time()
	# for _ in range(1000):
	#     ans = scene.cast_rays(rays)
	# t1 = time()

	# print(f"Time: {(t1 - t0) / 1000:.3f} s")
	# print("Giving a framerate of ", 1 / ((t1 - t0) / 1000), "fps")

	

	# Visualize the results.
	# plt.imshow(ans['t_hit'].numpy())
	# plt.show()
	# plt.imshow(np.abs(ans['primitive_normals'].numpy()))
	# plt.show()
	# plt.imshow(np.abs(ans['geometry_ids'].numpy()), vmax=3)
	# plt.show()