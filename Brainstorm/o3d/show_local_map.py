import rclpy

import sys

from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

from lmao.util.pclmao import extract_PointCloud2_data
from lmao.mapping import Map, get_normals


import time

import numpy as np

import matplotlib.pyplot as plt 

from PyQt5.QtWidgets import QFileDialog, QWidget, QApplication

import logging

import open3d as o3d
import pickle

def gen_map_with_our(xyz, depth, threshold_=0.25):
	map = Map()
	t0 = time.time()
	schmap = map.create_mesh(xyz, depth, threshold=threshold_)
	return schmap

def gen_map_poisson(xyz, depth, o3d_normals=False, sensor_normals=False, oct_depth=8, threads=-1):
	pcd = o3d.geometry.PointCloud()
	pcd.points = o3d.utility.Vector3dVector(xyz.reshape(-1, 3))
	if o3d_normals:
		print("Using o3d normals")
		pcd.estimate_normals()
	elif sensor_normals:
		print("Using bad normals")
		pcd.normals = o3d.utility.Vector3dVector((-xyz.reshape(-1, 3) / np.linalg.norm(xyz.reshape(-1, 3), axis=1)[:, None]).reshape(-1, 3))
	else:
		print("Using our normals")
		normals = get_normals(xyz)
		pcd.normals = o3d.utility.Vector3dVector(normals.reshape(-1, 3))
	poisson_mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=oct_depth, n_threads=threads)
	return poisson_mesh

def show_map(map):
	map.compute_vertex_normals()
	map.compute_triangle_normals()
	o3d.visualization.draw_geometries([map], lookat=[ -0.16383916888175371, -1.8080967901259908, -1.21373051831676759 ], front=[ 0.69304398273194945, 0.66553956986491336, 0.27703270374283168 ], up=[ -0.20636982027943943, -0.18504455065823375, 0.96081736638630988 ], zoom=0.259)

def evaluate_mesh(mesh, xyz):
	verts = len(np.asarray(mesh.vertices))

	scene = o3d.t.geometry.RaycastingScene()
	scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))

	points = xyz.reshape(-1, 3).astype(np.float32)

	dist = scene.compute_distance(points)
	avg_dist = np.mean(dist.numpy())
	rmse = np.sqrt(np.mean(np.square(dist.numpy())))

	return verts, rmse, avg_dist



def main(args=None):
	rclpy.init()


	bag_file = "/home/danitech/Documents/bags/island_boy_to_rule_them_all" #QFileDialog.getExistingDirectory(QWidget, 'Open file', "/home/junge/Documents/bags/island_boy_to_rule_them_all")
	bag_file = "/home/junge/Documents/bags/island_boy_to_rule_them_all"
	bag_file = "/home/junge/Documents/bags/rosbag2_2023_03_09-15_53_39"
	time0 = 0

	t0 = time.time()

	time_dict = {"poisson_sensor": [], "poisson_o3d": [], "poisson_our": [], "our": []}
	vertices_dict = {"poisson_sensor": [], "poisson_o3d": [], "poisson_our": [], "our": []}
	RMSE_dict = {"poisson_sensor": [], "poisson_o3d": [], "poisson_our": [], "our": []}
	avg_dist_dict = {"poisson_sensor": [], "poisson_o3d": [], "poisson_our": [], "our": []}

	with Reader(bag_file) as reader:
		print("We in da bag")
		#for connection in reader.connections:
			#print(connection.topic, connection.msgtype)
		for connection, timestamp, rawdata in reader.messages():
			# print(connection.topic, connection.msgtype)
			if connection.topic == "/wagon/base_scan/lidar_data" or connection.topic == "/points":
				for i in range(20):
					print("Iteration: ", i)
					msg = deserialize_cdr(rawdata, connection.msgtype)
					# print("Time to deserialize: ", time.time()-t0)
					# Read unstagger from unstagger.pkl
					with open("unstagger.pkl", "rb") as f:
						unstagger = pickle.load(f)
					
					data_dict = extract_PointCloud2_data(msg, visualize=False, unstagger_array=unstagger)
					# Data dict has keys x, y, z, range
					xyz = np.dstack((data_dict["x"], data_dict["y"], data_dict["z"]))
					# Shape of xyz: (1024, 128, 3)
					depth = data_dict["range"]
					depth = depth.reshape(depth.shape[0], -1)

					# Generate mesh with poisson and our normals
					t0 = time.time()
					mesh = gen_map_poisson(xyz, depth, o3d_normals=False, sensor_normals=False, oct_depth=8, threads=1)
					time_dict["poisson_our"].append(time.time()-t0)
					verts, rmse, avg_dist = evaluate_mesh(mesh, xyz)
					vertices_dict["poisson_our"].append(verts)
					RMSE_dict["poisson_our"].append(rmse)
					avg_dist_dict["poisson_our"].append(avg_dist)
					

					# Generate mesh with poisson and sensor normals
					t0 = time.time()
					mesh = gen_map_poisson(xyz, depth, o3d_normals=False, sensor_normals=True, oct_depth=8, threads=1)
					time_dict["poisson_sensor"].append(time.time()-t0)
					verts, rmse, avg_dist = evaluate_mesh(mesh, xyz)
					vertices_dict["poisson_sensor"].append(verts)
					RMSE_dict["poisson_sensor"].append(rmse)
					avg_dist_dict["poisson_sensor"].append(avg_dist)

					# Generate mesh with poisson and o3d normals
					t0 = time.time()
					mesh = gen_map_poisson(xyz, depth, o3d_normals=True, sensor_normals=False, oct_depth=8, threads=1)
					time_dict["poisson_o3d"].append(time.time()-t0)
					verts, rmse, avg_dist = evaluate_mesh(mesh, xyz)
					vertices_dict["poisson_o3d"].append(verts)
					RMSE_dict["poisson_o3d"].append(rmse)
					avg_dist_dict["poisson_o3d"].append(avg_dist)

					# Generate mesh with our method
					t0 = time.time()
					mesh = gen_map_with_our(xyz, depth, threshold_=0.25)
					time_dict["our"].append(time.time()-t0)
					verts, rmse, avg_dist = evaluate_mesh(mesh, xyz)
					vertices_dict["our"].append(verts)
					RMSE_dict["our"].append(rmse)
					avg_dist_dict["our"].append(avg_dist)
				# Save dict to pkl
				with open("test_local/time_dict.pkl", "wb") as f:
					pickle.dump(time_dict, f)
				with open("test_local/vertices_dict.pkl", "wb") as f:
					pickle.dump(vertices_dict, f)
				with open("test_local/RMSE_dict.pkl", "wb") as f:
					pickle.dump(RMSE_dict, f)
				with open("test_local/avg_dist_dict.pkl", "wb") as f:
					pickle.dump(avg_dist_dict, f)
				break


if __name__ == "__main__":
	# this(sys.argv[1], sys.argv[2])
	main()