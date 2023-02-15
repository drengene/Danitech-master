import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from lmao.world import World

# Example data (replace with your actual data)
data = World.generate_lidar_rays(horizontal_lines=21, rays_per_line=100)
print("Direction vectors length for raycasting all close to 1: ", World.check_unity(data))
World.plot_rays(data)
