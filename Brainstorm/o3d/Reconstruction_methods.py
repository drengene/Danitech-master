import open3d as o3d
from lmao.util.bag_loader import BagLoader
from lmao.util.mesh_test import check_properties
import numpy as np
from lmao.mapping import Map, get_normals
import matplotlib.pyplot as plt
import time

# Load bag file
bagloader = BagLoader(bagpath = '/home/junge/Documents/rosbag2_2023_03_09-15_53_39',topics = ['/points'])
xyz, depth = bagloader.get_pointcloud('/points')
# Check if any values are nan
print(np.isnan(xyz).any())
# Check if any values are inf
print(np.isinf(xyz).any())
# Check if any values are 0
print(np.count_nonzero(xyz == [0, 0, 0]))


normals = get_normals(xyz)
plt.imshow(normals)
plt.show()

plt.imshow(depth)
plt.show()

plt.imshow(xyz)
plt.show()

xyz = xyz.reshape(-1, 3)
normals = normals.reshape(-1, 3)
# Remove points that have [0, 0, 0] as normal
xyz = xyz[normals[:, 0] != 0]
normals = normals[normals[:, 0] != 0]
                  
pcd = o3d.t.geometry.PointCloud(xyz)

pcd = pcd.to_legacy()
pcd.normals = o3d.utility.Vector3dVector(normals.reshape(-1, 3))

# Show pcd
o3d.visualization.draw_geometries([pcd])
# Compute normals that are oriented towards [0, 0, 0]



# Create mesh using alpha shape
#pcd = o3d.utility.Vector3dVector(o3d.core.Tensor(xyz))
# mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, 1)
# o3d.visualization.draw_geometries([mesh])

# Create mesh using delaunay triangulation
#pcd = o3d.utility.Vector3dVector(o3d.core.Tensor(xyz))


			# "boundingbox_max" : [ 4.1213226318359375, 4.7637367248535156, 0.87641143798828125 ],
			# "boundingbox_min" : [ -5.9636983871459961, -6.5073537826538086, -2.3556690216064453 ],
			# "field_of_view" : 60.0,
			# "front" : [ 0.84732198172019557, -0.52728541423993913, 0.063368377157510217 ],
			# "lookat" : [ -0.5385805773511031, -2.7543633327825869, -0.50953180767962569 ],
			# "up" : [ 0.0, 0.0, 1.0 ],
			# "zoom" : 0.16

t0 = time.time()
mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=9, n_threads=0)
t1 = time.time()

#check_properties("Poisson", mesh)

print("Time to create mesh with Poisson: ", t1-t0)
print("Number of vertices: ", len(mesh.vertices))
print("Number of triangles: ", len(mesh.triangles))
o3d.visualization.draw_geometries([mesh], lookat = np.array(np.float64([ -0.5385805773511031, -2.7543633327825869, -0.50953180767962569 ])), 
                                  up = np.array(np.float64([ 0.0, 0.0, 1 ])), 
                                  front = np.array(np.float64([ 0.84732198172019557, -0.52728541423993913, 0.063368377157510217 ])), zoom = 0.16 )