import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from lmao.world import World
from time import time
import quaternion

# Example data (replace with your actual data)
t0 = time()
data = World.generate_lidar_rays(horizontal_lines=64, rays_per_line=1024)
q = quaternion.as_quat_array([0.707, 0.0, 0.707, 0.0])

# Rotate the lidar rays into the global coordinate frame
rays_local_flat = data.reshape(-1, 3)
rays_global_flat = quaternion.rotate_vectors(q, rays_local_flat)
rays_global = rays_global_flat.reshape(data.shape)
print("Time to generate and rotate rays: ", time() - t0)
print("Direction vectors length for raycasting all close to 1: ", World.check_unity(data))
World.plot_rays(rays_global)
