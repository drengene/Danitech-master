import open3d as o3d

import numpy as np
import matplotlib.pyplot as plt
from time import time, sleep
import cv2
from lmao.world import World
from lmao.lidar import Lidar
import quaternion
from scipy.spatial.transform import Rotation as R

def convert_to_cv2_image(depth):
	depth[depth == np.inf] = 0
	depth = depth / np.max(depth)
	depth = depth * 255
	depth = depth.astype(np.uint8)
	depth = cv2.applyColorMap(depth, cv2.COLORMAP_JET)
	return depth

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
	world.add_mesh_to_scene([cube, torus, sphere2, sphere])

	# Create a lidar.
	lidar = Lidar()

	# Translate rays.
	lidar.translate_rays([2, 3, 0], change_original=True)
	#plt.ion()
	lidar.plot_rays(lidar.rays)
	sleep(1)
	# Rotate rays
	#q = quaternion.as_quat_array([0, 0.707, 0, 0.707])
	# q = quaternion.as_quat_array([0.707, 0.707, 0, 0]) # xyz = [pi/2, 0, 0] for quaternion library
	# rotated = lidar.rotate_rays(q)

	# rotated = lidar.rotate_rays_scipy([0, 0, 0.707, 0.707]) # xyz = [0, 0, pi/2] when x,y,z,w
	# lidar.plot_rays(rotated)
	# sleep(1)
	# lidar.plot_rays(lidar.rays)
	# # q = quaternion.as_quat_array([0.707,0, 0.707, 0]) # xyz = [0, pi/2, 0] for quaternion library
	# # rotated = lidar.rotate_rays(q)
	# rotated = lidar.rotate_rays_scipy([0, 0.707, 0, 0.707]) # xyz = [0, pi/2, 0] when x,y,z,w
	# lidar.plot_rays(rotated)
	# sleep(1)
	# lidar.plot_rays(lidar.rays)
	# # q = quaternion.from_euler_angles([ 0.707, 0, 0, 0.707]) # xyz = [0, 0, pi/2] for quaternion library
	# # rotated = lidar.rotate_rays(q)
	# rotated = lidar.rotate_rays_scipy([0.707, 0, 0, 0.707]) # xyz = [pi/2, 0, 0] when x,y,z,w
	# lidar.plot_rays(rotated)
	# sleep(1)


	# Test 1000 times and get the average time.
	t0 = time()
	for _ in range(1000):
		rotated = lidar.rotate_rays([0, 0.707, 0.707, 0])
	t1 = time()
	print(f"Time using quaternion library: {(t1 - t0):.3f} s")
	print("Giving a framerate of ", 1 / ((t1 - t0) / 1000), "fps")

	# Test 1000 times and get the average time.
	t0 = time()
	for _ in range(1000):
		rotated = lidar.rotate_rays_scipy([0, 0.707, 0, 0.707]) # xyz = [0, pi/2, 0] when x,y,z,w
	t1 = time()
	print(f"Time using scipy: {(t1 - t0):.3f} s")
	print("Giving a framerate of ", 1 / ((t1 - t0) / 1000), "fps")



	exit()

	t0 = time()
	# Create a 10 second animation with fps of 10
	for i in range(0, 500):
		# Create quaternion from rpy
		q = quaternion.from_euler_angles([(i/500)*2*np.pi, 0.0, 0.0]) # Roll, pitch, yaw
		rotated = lidar.rotate_rays(q)
		ans = world.cast_rays(rotated)
		depth = ans['t_hit'].numpy()
		image = convert_to_cv2_image(depth)
		print("Frame ", i, " took ", time() - t0, " seconds, giving us a frequency of ", 1 / (time() - t0), "fps")
		t0 = time()
		cv2.imshow("depth", image)
		cv2.waitKey(1)



	

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