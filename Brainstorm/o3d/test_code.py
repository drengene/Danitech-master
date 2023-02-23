from lmao.lidar import Lidar
from time import time, sleep
import numpy as np
import open3d as o3d




def test_rotation(n = 1000, q = [0.707, 0.0, 0.707, 0.0]):
	lidar = Lidar()
	lidar.translate_rays([2, 3, 0], change_original=True)

	print("Testing rotation of ", n, " rays, with quaternion ", q)

	# Test 1000 times and get the average time.
	t0 = time()
	for _ in range(n):
		rotated = lidar.rotate_rays(q)
	t1 = time()
	print("\nWith library 'quaternion'")
	print("n = ", n, ", Time using quaternion: ", (t1 - t0), " s")
	print("Giving a framerate of ", 1 / ((t1 - t0) / n), "fps")

	# Test 1000 times and get the average time.
	t0 = time()
	for _ in range(n):
		rotated = lidar.rotate_rays_scipy(q) # xyz = [0, pi/2, 0] when x,y,z,w
	t1 = time()
	print("\nWith library 'scipy'")
	print("n = ", n, ", Time using scipy: ", (t1 - t0), " s")
	print("Giving a framerate of ", 1 / ((t1 - t0) / n), "fps")


if __name__ == "__main__":
	# Create random quaternion on form [x, y, z, w]
	# q = np.random.rand(4)
	# q /= np.linalg.norm(q)
	# test_rotation(1000, q)


	eagle = o3d.data.EaglePointCloud()
	pcd = o3d.io.read_point_cloud(eagle.path)
	print(type(pcd))

	print(pcd)
	o3d.visualization.draw_geometries([pcd],
									zoom=0.664,
									front=[-0.4761, -0.4698, -0.7434],
									lookat=[1.8900, 3.2596, 0.9284],
									up=[0.2304, -0.8825, 0.4101])

	print('run Poisson surface reconstruction')
	with o3d.utility.VerbosityContextManager(
			o3d.utility.VerbosityLevel.Debug) as cm:
		mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
			pcd, depth=9)
	print(mesh)
	o3d.visualization.draw_geometries([mesh],
									zoom=0.664,
									front=[-0.4761, -0.4698, -0.7434],
									lookat=[1.8900, 3.2596, 0.9284],
									up=[0.2304, -0.8825, 0.4101])
