import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from lmao.world import World

# Example data (replace with your actual data)
data = World.generate_lidar_rays(10, 100)

# Extract x, y, and z coordinates from the data
x = data[:, :, 3].flatten()
y = data[:, :, 4].flatten()
z = data[:, :, 5].flatten()

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x, y, z)

# Add labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.title('3D Scatter Plot')

# Show the plot
plt.show()