import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from lmao.world import World

# Example data (replace with your actual data)
data = World.generate_lidar_rays(horizontal_lines=21, rays_per_line=100)
print("Direction vectors length for raycasting all close to 1: ", World.check_unity(data))

# Extract x, y, and z coordinates from the data
x = data[:, :, 3].flatten()
y = data[:, :, 4].flatten()
z = data[:, :, 5].flatten()

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x, y, z, c='b', marker='+')

# Plot a sphere for the origin
u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
x = np.cos(u)*np.sin(v)
y = np.sin(u)*np.sin(v)
z = np.cos(v)
ax.plot_surface(x, y, z, color="r", alpha=0.5)


# Add labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.title('3D Scatter Plot')

# Show the plot
plt.show()