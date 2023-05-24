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

def gen_map_poisson(xyz, depth, o3d_normals=False, bad_normals=False, oct_depth=8, threads=-1):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz.reshape(-1, 3))
    if o3d_normals:
        print("Using o3d normals")
        pcd.estimate_normals()
    elif bad_normals:
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


def main(args=None):
    rclpy.init()


    bag_file = "/home/danitech/Documents/bags/island_boy_to_rule_them_all" #QFileDialog.getExistingDirectory(QWidget, 'Open file', "/home/junge/Documents/bags/island_boy_to_rule_them_all")
    bag_file = "/home/junge/Documents/bags/island_boy_to_rule_them_all"
    bag_file = "/home/junge/Documents/bags/rosbag2_2023_03_09-15_53_39"
    time0 = 0

    t0 = time.time()

    time_mapping = 0
    mapping_count = 0

    time_poisson = 0
    poisson_count = 0

    with Reader(bag_file) as reader:
        print("We in da bag")
        #for connection in reader.connections:
            #print(connection.topic, connection.msgtype)
        for connection, timestamp, rawdata in reader.messages():
            # print(connection.topic, connection.msgtype)
            if connection.topic == "/wagon/base_scan/lidar_data" or connection.topic == "/points":
                for i in range(10):
                    # print("We in da lidar")
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

                    # Generate mesh with poisson
                    mesh = gen_map_poisson(xyz, depth, o3d_normals=False, bad_normals=False)
                    # Save mesh to file
                    o3d.io.write_triangle_mesh("mesh_poisson.ply", mesh)
                    # Generate mesh with our method
                    #mesh = gen_map_with_our(xyz, depth, threshold_=0.25)
                    # View
                    show_map(mesh)




                    # show_map(mesh)
                    scene = o3d.t.geometry.RaycastingScene()
                    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))

                    points = xyz.reshape(-1, 3).astype(np.float32)
                    print("Shape of points: ", points.shape)

                    dist = scene.compute_distance(points)
                    print("Shape of dist: ", dist.numpy().shape)

                    rmse = np.sqrt(np.mean(np.square(dist.numpy())))
                    print("RMSE: ", rmse)

                    #Shape of xyz: (1024, 128, 3)
                    #xyz = np.rot90(xyz, 1, (1, 0))
                    #depth = np.rot90(depth, 1, (1, 0))

                    # print("Shape of xyz: ", xyz.shape)
                    # print("Shape of depth: ", depth.shape)
                    break
                break


if __name__ == "__main__":
    # this(sys.argv[1], sys.argv[2])
    main()