import glob
import os
import matplotlib.pyplot as plt
import pickle
import numpy as np

def load_latest_file(folder_path):
    # get a list of all the files in the folder
    files = glob.glob(os.path.join(folder_path, '*.pkl'))

    # sort the files by modification time (newest first)
    files.sort(key=os.path.getmtime, reverse=True)

    # load the latest file
    with open(files[0], 'rb') as f:
        data = pickle.load(f)

    return data


def poly_fit_plot(base_pose, wayposes):


    base_pose_t = np.arange(base_pose.shape[0])  # simple assumption that data was sampled in regular steps

    base_x = base_pose[:,0]
    base_y = base_pose[:,1]
    base_z = base_pose[:,2]

    # fit a 4th order polynomial to the base pose data
    base_pose_fitx = np.polyfit(base_pose_t, base_x, 4)
    base_pose_fity = np.polyfit(base_pose_t, base_y, 4)
    base_pose_fitz = np.polyfit(base_pose_t, base_z, 4)

    waypose_t = np.arange(wayposes.shape[0])  # simple assumption that data was sampled in regular steps
    way_x = wayposes[:,0]
    way_y = wayposes[:,1]
    way_z = wayposes[:,2]

    # fit a 4th order polynomial to the waypose data
    waypose_fitx = np.polyfit(waypose_t, way_x, 4)
    waypose_fity = np.polyfit(waypose_t, way_y, 4)
    waypose_fitz = np.polyfit(waypose_t, way_z, 4)

    # create a 3D plot of basepose_fit and waypose_fit
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    


def scatter_plot(base_pose, wayposes):


    # create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # plot the base poses as red circles
    ax.scatter(base_pose[:,0], base_pose[:,1], base_pose[:,2], c='r', marker='o')

    # plot the wayposes as blue triangles
    ax.scatter(wayposes[:,0], wayposes[:,1], wayposes[:,2], c='b', marker='^')

    # set the axis labels
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    # show the plot
    plt.show()


# specify the folder path where your pickle files are stored
folder_path = '/home/danitech/master_ws/src/Danitech-master/wagon_navigation/wagon_navigation/pose_data/'

# load the latest file in the folder
data = load_latest_file(folder_path)

# extract the data you need
base_pose = data['base_pose']
wayposes = data['wayposes']

#print(len(wayposes))

poly_fit_plot(base_pose, wayposes)
