import glob
import os
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline, Rbf
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

    #print(data)
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

    

    ax.plot(base_x, base_y, base_z, c='r', alpha=0.5)
    ax.plot(way_x, way_y, way_z, c='b', alpha=0.5)

     # set the axis labels
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    # even axes
    ax.set_box_aspect((np.ptp(base_x), np.ptp(base_y), np.ptp(base_z)))  # aspect ratio is 1:1:1 in data space

    # add legend
    ax.legend(['base_pose', 'wayposes'])
    # show the plot
    plt.show()
    
def calc_closest_dist_error(base_pose, wayposes):
    
    closest = np.zeros(wayposes.shape[0])

    print(closest.shape)
    for i in range(0, wayposes.shape[0]):
        diff = np.zeros(base_pose.shape[0])
        for j in range(0, base_pose.shape[0]):
            diff[j] = np.linalg.norm(base_pose[j] - wayposes[i])

        closest[i] = min(diff)



    return closest


def calc_rme(base_pose, wayposes):
    # Calculate the displacement between the two paths
    interpolated_path = interpolate_points(wayposes, np.shape(base_pose)[0])

    displacement = base_pose - interpolated_path

    # Calculate the squared Euclidean distance between each displacement vector
    squared_distances = np.sum(displacement**2, axis=1)

    # Calculate the mean of the squared distances
    mean_squared_distance = np.mean(squared_distances)

    # Calculate the RMSE
    rmse = np.sqrt(mean_squared_distance)

    print("RMSE:", rmse)

    return rmse


def interpolate_points(points, num_points):
            
    # Concatenate the points to form a 3x3 array
    # points = np.array([p1, p2, p3])

    # Calculate the distances between each pair of points
    distances = np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1))

    # Calculate the cumulative distance along the curve
    cumulative_distances = np.cumsum(distances)
    cumulative_distances = np.insert(cumulative_distances, 0, 0) # Add initial distance of 0

    # Create a cubic spline interpolation of the points
    interp = CubicSpline(cumulative_distances, points, bc_type='not-a-knot')

    # Generate points along the curve at the specified resolution
    s_vals = np.linspace(cumulative_distances[0], cumulative_distances[-1], num_points)

    # for idx, dist in enumerate(cumulative_distances[:-1], ):
    #     num_points = int(np.ceil((cumulative_distances[idx + 1] - dist)/resolution))
    #     # print(num_points)
    #     s_val = np.linspace(dist, cumulative_distances[idx + 1], num_points)
    #     s_vals = np.append(s_vals, s_val[1:])

    # Generate 10 points along the curve
    interp_points = interp(s_vals)

    return interp_points


def scatter_plot(base_pose, wayposes):


    # create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # plot the base poses as red circles
    ax.scatter(base_pose[:,0], base_pose[:,1], base_pose[:,2], c='r', marker='o')

    # plot the wayposes as numbered blue triangles
    ax.scatter(wayposes[:,0], wayposes[:,1], wayposes[:,2], c='b', marker='^')
    for i, (x, y, z) in enumerate(zip(base_pose[:,0], base_pose[:,1], base_pose[:,2])):
        if i % 100 == 0:
            ax.text(x, y, z, str(i), color="red", fontsize=12)

    # set the axis labels
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    # show the plot
    plt.show()


def find_first_closes_point(base_pose, wayposes):
    diff_old = np.inf
    for i in range(len(wayposes)):
        diff = np.linalg.norm(wayposes[0,0:3] - base_pose[i])
        # print("diff:", diff)
        if diff > diff_old:
            print("diff: ", diff, "diff_old: ", diff_old, "i: ", i)
        diff_old = diff

# specify the folder path where your pickle files are stored
folder_path = '/home/daniel/master_ws/src/Danitech-master/wagon_navigation/wagon_navigation/pose_data/'
#folder_path = '/home/danitech/master_ws/src/Danitech-master/wagon_navigation/wagon_navigation/pose_data/'

# load the latest file in the folder
data = load_latest_file(folder_path)

print(len(data['base_link_gt']))
exit()
# extract the data you need
base_pose = data['base_pose']
base_or = data['base_or']
wayposes = data['wayposes']

print(wayposes[0,:])

# combinethe first element of base pose and base or to get the first base pose
first_pose = np.concatenate((base_pose[0,:], base_or[0,:]))

wayposes[0,:] = first_pose
print(wayposes[0,:])
# print(wayposes)
#print(len(wayposes))
#print("first closest point:", find_first_closes_point(base_pose, wayposes))

calc_rme(base_pose, wayposes[:,0:3])
dists_xyz = calc_closest_dist_error(base_pose, wayposes[:-1,0:3])
print("xyz", dists_xyz)
dists_xy = calc_closest_dist_error(base_pose[:,0:2], wayposes[:-1,0:2])
print("xy", dists_xy)
print("mean xyz", np.mean(dists_xyz))
print("mean xy", np.mean(dists_xy))
print("diff xy xyz", dists_xy - dists_xyz)


interp_wayposes = interpolate_points(wayposes[:-1, 0:3], np.shape(base_pose)[0]*2)
dist_to_pose = calc_closest_dist_error(interp_wayposes[:,0:2], base_pose[:,0:2])
print("dist to pose", np.mean(dist_to_pose))

poly_fit_plot(base_pose, wayposes)
#scatter_plot(base_pose, wayposes)