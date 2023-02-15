from lmao.world import World
import matplotlib.pyplot as plt
import numpy as np

from scipy.spatial.transform import Rotation as R
from time import time

def plot_data(x, y, z, fig, show=False):
    # Create a 3D plot
    # fig = plt.figure()
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
    if show:
        plt.show()
    # plt.show()
    

def main():


    # Example data (replace with your actual data)
    data = World.generate_lidar_rays(horizontal_lines=64, rays_per_line=20)
    # print(data)

    # Extract x, y, and z coordinates from the data

    # rotate x y and z 90 degrees with quaternions
    r = R.from_quat([0.707, 0, 0.707, 0])


    # print(data.shape)

    # data[:,:,3:6] = r.apply(data[:,:,3:6])

    time0 = time()
    for i in range(data.shape[0]):
        # data[i,:,3:5] = r.apply(data[i,:,3:5])

        for j in range(data.shape[1]):
            data[i,j,3:6] = r.apply(data[i,j,3:6])

    # convert the two for loops into a one liner

    print("time: ", time() - time0)


    fig = plt.figure()
    World.plot_rays(data, fig)
    plt.show()
    
if __name__ == '__main__':
    main()